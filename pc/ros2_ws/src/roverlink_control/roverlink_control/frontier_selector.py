#!/usr/bin/env python3
"""
ROS2 Humble: Frontier clustering + goal selection + plotting

Subs:
  /map            nav_msgs/OccupancyGrid
  /robot/pose     geometry_msgs/Pose2D      (theta in DEGREES)
  /frontier_next  std_msgs/Bool             (skip current goal -> next frontier)
  /planned_path   nav_msgs/Path             (optional; for plotting A* result)

Pubs:
  /frontier_goal  geometry_msgs/PoseStamped (goal for planner node)

Behavior:
- Clusters are computed on /map updates.
- A "best" frontier goal is chosen (nearest cluster representative to robot).
- Publishes next goal ONLY when:
    (a) reached previous goal, OR
    (b) /frontier_next is True, OR
    (c) current goal invalid / disappeared.
- Supports obstacle inflation (thickened walls) for safety and plotting.
- Logs "explored: no frontier detected" when nothing left.
"""

import time
import math
from collections import deque

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Pose2D, PoseStamped
from std_msgs.msg import Bool
from matplotlib.colors import ListedColormap, BoundaryNorm


# ---------- Visualization colormap ----------
# 0.0 = occupied (black), 0.5 = unknown (lightgray), 1.0 = free (white)
cmap = ListedColormap(["black", "lightgray", "white"])
norm = BoundaryNorm([0.0, 0.25, 0.75, 1.0], cmap.N)


def grid_from_occupancy(msg: OccupancyGrid) -> np.ndarray | None:
    w = msg.info.width
    h = msg.info.height
    data = np.asarray(msg.data, dtype=np.int16)
    if data.size != w * h:
        return None
    return data.reshape((h, w))


def neighbor_unknown_mask(unknown: np.ndarray) -> np.ndarray:
    h, w = unknown.shape
    padded = np.pad(unknown, 1, mode="constant", constant_values=False)
    nbr = np.zeros((h, w), dtype=bool)
    nbr |= padded[0:0 + h, 0:0 + w]
    nbr |= padded[0:0 + h, 1:1 + w]
    nbr |= padded[0:0 + h, 2:2 + w]
    nbr |= padded[1:1 + h, 0:0 + w]
    nbr |= padded[1:1 + h, 2:2 + w]
    nbr |= padded[2:2 + h, 0:0 + w]
    nbr |= padded[2:2 + h, 1:1 + w]
    nbr |= padded[2:2 + h, 2:2 + w]
    return nbr


def compute_frontier_mask(grid: np.ndarray) -> np.ndarray:
    # frontier cells are FREE cells adjacent to UNKNOWN
    free = (grid == 0)
    unknown = (grid == -1)
    return free & neighbor_unknown_mask(unknown)


def cluster_frontiers(frontier_mask: np.ndarray, min_cluster_size: int = 20):
    h, w = frontier_mask.shape
    visited = np.zeros_like(frontier_mask, dtype=bool)
    clusters = []

    nbrs = [(-1, -1), (-1, 0), (-1, 1),
            (0, -1),           (0, 1),
            (1, -1),  (1, 0),  (1, 1)]

    frontier_cells = np.argwhere(frontier_mask)
    for r0, c0 in frontier_cells:
        r0, c0 = int(r0), int(c0)
        if visited[r0, c0]:
            continue

        q = deque([(r0, c0)])
        visited[r0, c0] = True
        cluster = []

        while q:
            r, c = q.popleft()
            cluster.append((r, c))
            for dr, dc in nbrs:
                rr, cc = r + dr, c + dc
                if 0 <= rr < h and 0 <= cc < w and frontier_mask[rr, cc] and not visited[rr, cc]:
                    visited[rr, cc] = True
                    q.append((rr, cc))

        if len(cluster) >= min_cluster_size:
            clusters.append(cluster)

    return clusters


def cell_to_world(r, c, origin_x, origin_y, res):
    x = origin_x + (c + 0.5) * res
    y = origin_y + (r + 0.5) * res
    return x, y


def inflate_occupied_mask(occ: np.ndarray, radius_cells: int) -> np.ndarray:
    """
    Inflate occupied cells by radius (in grid cells) using a circular stencil.
    Numpy-only implementation.
    """
    if radius_cells <= 0:
        return occ.copy()

    h, w = occ.shape
    inflated = occ.copy()

    # precompute offsets in a disk
    offs = []
    r = radius_cells
    r2 = r * r
    for dr in range(-r, r + 1):
        for dc in range(-r, r + 1):
            if dr * dr + dc * dc <= r2:
                offs.append((dr, dc))

    occ_idx = np.argwhere(occ)
    for rr, cc in occ_idx:
        rr = int(rr); cc = int(cc)
        for dr, dc in offs:
            r2_ = rr + dr
            c2_ = cc + dc
            if 0 <= r2_ < h and 0 <= c2_ < w:
                inflated[r2_, c2_] = True

    return inflated


def build_map_image_from_masks(unknown: np.ndarray, free: np.ndarray, occ_inflated: np.ndarray) -> np.ndarray:
    img = np.zeros(unknown.shape, dtype=np.float32)
    img[unknown] = 0.5
    img[free] = 1.0
    img[occ_inflated] = 0.0
    return img


def cluster_representative_world(cluster_rc, origin_x, origin_y, res):
    """
    Paper uses a median frontier point.
    We approximate robustly:
    - compute median (r,c)
    - then pick the cluster cell closest to that median (ensures it's a real frontier cell)
    """
    rs = np.array([p[0] for p in cluster_rc], dtype=np.int32)
    cs = np.array([p[1] for p in cluster_rc], dtype=np.int32)

    r_med = int(np.median(rs))
    c_med = int(np.median(cs))

    # pick actual cluster cell closest to (r_med, c_med)
    best = None
    best_d2 = 1e18
    for (r, c) in cluster_rc:
        d2 = (r - r_med) ** 2 + (c - c_med) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best = (r, c)

    r_goal, c_goal = best
    return cell_to_world(r_goal, c_goal, origin_x, origin_y, res), (r_goal, c_goal)


class FrontierNode(Node):
    def __init__(self):
        super().__init__("frontier_selector_plotter")

        # ---------- topics ----------
        self.map_topic = "/map"
        self.pose_topic = "/robot/pose"
        self.frontier_next_topic = "/frontier_next"
        self.goal_topic = "/frontier_goal"
        self.path_topic = "/planned_path"   # change if your A* publishes on another name

        # ---------- parameters ----------
        self.declare_parameter("min_cluster_size", 20)
        self.declare_parameter("max_clusters_draw", 40)

        self.declare_parameter("robot_diameter_m", 0.3)
        self.declare_parameter("inflation_margin_m", 0.05)  # extra safety
        self.declare_parameter("use_inflated_for_frontiers", True)

        self.declare_parameter("goal_reached_tol_m", 0.35)  # when pose within this, goal reached
        self.declare_parameter("publish_cooldown_s", 0.5)   # avoid spamming

        self.declare_parameter("plot_show_inflated", False)
        self.plot_show_inflated = bool(self.get_parameter("plot_show_inflated").value)

        # ---------- retry looping ----------
        self.declare_parameter("max_full_cycles", 3)  # how many times to loop over all clusters
        self.declare_parameter("cycle_pause_s", 1.0)  # pause when wrapping back to rank 0

        self.max_full_cycles = int(self.get_parameter("max_full_cycles").value)
        self.cycle_pause_s = float(self.get_parameter("cycle_pause_s").value)

        self.cycle_count = 0
        self.last_cycle_wrap_t = 0.0
        self.exploration_done = False
        #------------

        self.min_cluster_size = int(self.get_parameter("min_cluster_size").value)
        self.max_clusters_draw = int(self.get_parameter("max_clusters_draw").value)
        self.robot_diameter_m = float(self.get_parameter("robot_diameter_m").value)
        self.inflation_margin_m = float(self.get_parameter("inflation_margin_m").value)
        self.use_inflated_for_frontiers = bool(self.get_parameter("use_inflated_for_frontiers").value)
        self.goal_reached_tol_m = float(self.get_parameter("goal_reached_tol_m").value)
        self.publish_cooldown_s = float(self.get_parameter("publish_cooldown_s").value)

        # ---------- state ----------
        self.map_msg: OccupancyGrid | None = None
        self.grid: np.ndarray | None = None
        self.grid_for_frontiers: np.ndarray | None = None
        self.pose: Pose2D | None = None

        self.clusters = []
        self.cluster_goals_world = []     # list of ((x,y),(r,c), cluster_size)
        self.ordered_indices = []         # indices into cluster_goals_world
        self.selected_rank = 0            # which one we are trying now

        self.inflated_occ = None          # bool mask for plotting/safety

        self.map_count = 0

        self.current_goal_world = None    # (x,y)
        self.current_goal_idx = None      # index into ordered_indices
        self.need_new_goal = True
        self.last_publish_t = 0.0
        self.explored_logged = False

        self.planned_path: Path | None = None

        # ---------- qos ----------
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # ---------- subs/pubs ----------
        self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, map_qos)
        self.create_subscription(Pose2D, self.pose_topic, self.on_pose, 50)
        self.create_subscription(Bool, self.frontier_next_topic, self.on_frontier_next, 10)
        self.create_subscription(Path, self.path_topic, self.on_path, 10)

        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)

        self.get_logger().info(
            f"Listening /map, /robot/pose, {self.frontier_next_topic}. Publishing goals on {self.goal_topic}."
        )

    def on_path(self, msg: Path):
        self.planned_path = msg



    def on_frontier_next(self, msg: Bool):
        if msg.data:
            self.get_logger().warn("[FRONTIER] /frontier_next received -> skipping to next frontier.")
            self._advance_rank(reason="frontier_next", success=False)
            self.current_goal_world = None
            self.current_goal_idx = None

    def on_pose(self, msg: Pose2D):
        self.pose = msg

        # Check goal reached
        if self.current_goal_world is not None:
            gx, gy = self.current_goal_world
            dx = gx - float(msg.x)
            dy = gy - float(msg.y)
            d = math.hypot(dx, dy)
            if d <= self.goal_reached_tol_m:
                self.get_logger().info(f"[FRONTIER] Goal reached (d={d:.2f}m). Will publish next frontier.")
                self.current_goal_world = None
                self.current_goal_idx = None
                self._advance_rank(reason="goal_reached", success=True)

    def on_map(self, msg: OccupancyGrid):
        self.map_msg = msg
        self.grid = grid_from_occupancy(msg)
        if self.grid is None:
            return

        info = msg.info
        ox = float(info.origin.position.x)
        oy = float(info.origin.position.y)
        res = float(info.resolution)

        # Inflate obstacles for safety + plotting
        occ = (self.grid > 50)
        unknown = (self.grid == -1)
        free = (self.grid == 0)

        inflation_radius_m = (self.robot_diameter_m * 0.5) + self.inflation_margin_m
        inflation_cells = int(math.ceil(inflation_radius_m / max(1e-9, res)))
        self.inflated_occ = inflate_occupied_mask(occ, inflation_cells)

        # Build a "safe" grid: mark inflated cells as occupied
        self.grid_for_frontiers = self.grid.copy()
        self.grid_for_frontiers[self.inflated_occ] = 100

        # Choose which grid to use for frontier detection
        grid_used = self.grid_for_frontiers if self.use_inflated_for_frontiers else self.grid

        frontier = compute_frontier_mask(grid_used)
        self.clusters = cluster_frontiers(frontier, min_cluster_size=self.min_cluster_size)

        # Build cluster representative goals (median-like)
        self.cluster_goals_world = []
        for cl in self.clusters:
            (x, y), (r, c) = cluster_representative_world(cl, ox, oy, res)
            self.cluster_goals_world.append(((x, y), (r, c), len(cl)))

        self.map_count += 1
        #self.get_logger().info(f"/map update {self.map_count}: clusters={len(self.clusters)}")
        if self.map_count % 10 == 0:
            self.get_logger().info(f"/map update {self.map_count}: clusters={len(self.clusters)}")

        # Re-rank clusters whenever map updates AND we have pose
        self._rank_clusters()

        # If current goal disappeared (no longer valid), force reselect
        if self.current_goal_world is not None and self.pose is not None:
            if not self._goal_still_valid():
                self.get_logger().warn("[FRONTIER] Current goal no longer valid after map update -> reselection.")
                self.need_new_goal = True
                self.current_goal_world = None
                self.current_goal_idx = None

    def _goal_still_valid(self) -> bool:
        if self.current_goal_world is None or self.map_msg is None or self.grid_for_frontiers is None:
            return False

        info = self.map_msg.info
        ox = float(info.origin.position.x)
        oy = float(info.origin.position.y)
        res = float(info.resolution)
        h, w = self.grid_for_frontiers.shape

        gx, gy = self.current_goal_world
        c = int((gx - ox) / res)
        r = int((gy - oy) / res)
        if r < 0 or r >= h or c < 0 or c >= w:
            return False
        # must remain free in the safe grid
        return self.grid_for_frontiers[r, c] == 0

    def _rank_clusters(self):
        self.ordered_indices = []
        if self.pose is None or not self.cluster_goals_world:
            return

        rx = float(self.pose.x)
        ry = float(self.pose.y)

        # Sort by Euclidean distance (paper uses a simple distance-based choice + move_base reachability checks)
        dlist = []
        for i, (xy, rc, sz) in enumerate(self.cluster_goals_world):
            x, y = xy
            d = math.hypot(x - rx, y - ry)
            # tie-break: larger clusters slightly preferred
            dlist.append((d, -sz, i))

        dlist.sort()
        self.ordered_indices = [t[2] for t in dlist]

        # If we had no goal yet, allow selection
        if not self.cluster_goals_world:
            self.need_new_goal = True

    def _mark_frontier_explored(self, reason: str):
        if not self.exploration_done:
            self.get_logger().warn(f"Frontier explored ({reason}).")
        self.exploration_done = True
        self.need_new_goal = False
        self.current_goal_world = None
        self.current_goal_idx = None

    def _advance_rank(self, reason: str, success: bool = False):
        # On success, reset cycle counter (optional but usually desired)
        if success:
            self.cycle_count = 0

        self.selected_rank += 1
        self.need_new_goal = True

        # If we have a list, wrap and count cycles
        if self.ordered_indices:
            if self.selected_rank >= len(self.ordered_indices):
                self.selected_rank = 0
                self.cycle_count += 1
                self.last_cycle_wrap_t = time.time()

                self.get_logger().warn(
                    f"[FRONTIER] Wrapped to rank 0 (cycle {self.cycle_count}/{self.max_full_cycles}) reason={reason}"
                )

                if self.cycle_count >= self.max_full_cycles:
                    self._mark_frontier_explored("max_cycles_reached_no_goal_succeeded")

    def maybe_publish_next_goal(self):
        if self.exploration_done:
            return

        # pause after wrapping back to 0
        if self.cycle_count > 0 and (time.time() - self.last_cycle_wrap_t) < self.cycle_pause_s:
            return

        if self.map_msg is None or self.pose is None:
            return

        if not self.need_new_goal:
            return

        now = time.time()
        if (now - self.last_publish_t) < self.publish_cooldown_s:
            return

        # No frontiers left
        # No frontiers left (only if none exist at all)
        if self.map_count > 0 and self.pose is not None:
            if not self.ordered_indices:
                if not self.explored_logged:
                    self.get_logger().warn("Explored: no frontier detected (or none left to try).")
                    self.explored_logged = True
                return

        self.explored_logged = False

        # Pick next ranked frontier goal
        idx = self.ordered_indices[self.selected_rank]
        (gx, gy), (gr, gc), sz = self.cluster_goals_world[idx]

        # Publish PoseStamped goal (map frame)
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = float(gx)
        goal.pose.position.y = float(gy)
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0  # yaw not set here; planner can handle orientation

        self.goal_pub.publish(goal)
        self.last_publish_t = now

        rx, ry = float(self.pose.x), float(self.pose.y)
        dist = math.hypot(gx - rx, gy - ry)

        self.get_logger().warn(
            f"[FRONTIER] Publishing goal rank={self.selected_rank}/{len(self.ordered_indices)-1} "
            f"cluster_idx={idx} size={sz} goal=({gx:.2f},{gy:.2f}) dist={dist:.2f}m "
            f"(infl_cells used for plot/frontier={self.use_inflated_for_frontiers})"
        )

        self.current_goal_world = (gx, gy)
        self.current_goal_idx = idx
        self.need_new_goal = False


def main():
    rclpy.init()
    node = FrontierNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # ---- Matplotlib init ----
    plt.ion()
    fig, ax = plt.subplots()
    plt.show(block=False)

    plot_hz = 5.0
    plot_dt = 1.0 / plot_hz
    last_plot = 0.0

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.05)

            # publish goal if needed
            node.maybe_publish_next_goal()

            if not plt.fignum_exists(fig.number):
                break

            now = time.time()
            if now - last_plot < plot_dt:
                continue
            last_plot = now

            if node.map_msg is None or node.grid is None or node.inflated_occ is None:
                fig.canvas.draw_idle()
                fig.canvas.flush_events()
                plt.pause(0.001)
                continue

            info = node.map_msg.info
            ox = float(info.origin.position.x)
            oy = float(info.origin.position.y)
            res = float(info.resolution)
            h, w = node.grid.shape

            unknown = (node.grid == -1)
            free = (node.grid == 0)
            #img = build_map_image_from_masks(unknown, free, node.inflated_occ)

            occ_for_plot = node.inflated_occ if node.plot_show_inflated else (node.grid > 50)
            img = build_map_image_from_masks(unknown, free, occ_for_plot)

            ax.clear()
            extent = [ox, ox + w * res, oy, oy + h * res]
            #ax.imshow(img, origin="lower", extent=extent, cmap=cmap, norm=norm)
            ax.imshow(img, origin="lower", extent=extent, vmin=0.0, vmax=1.0, cmap=cmap)

            #im = ax.imshow(img, origin="lower", extent=extent, cmap=cmap, norm=norm)
            #im.format_cursor_data = lambda val: ""  # prevents BoundaryNorm inverse hover crash

            # frontier points
            for cl in node.clusters[: node.max_clusters_draw]:
                xs = np.empty(len(cl), dtype=np.float32)
                ys = np.empty(len(cl), dtype=np.float32)
                for k, (r, c) in enumerate(cl):
                    x, y = cell_to_world(r, c, ox, oy, res)
                    xs[k] = x
                    ys[k] = y
                ax.scatter(xs, ys, s=3)

            # draw cluster representative points with labels (their rank order)
            if node.cluster_goals_world:
                # show all goals as x
                gx_all = [g[0][0] for g in node.cluster_goals_world]
                gy_all = [g[0][1] for g in node.cluster_goals_world]
                ax.scatter(gx_all, gy_all, s=70, marker="x")

                # label with order rank (0 is best)
                for rank, idx in enumerate(node.ordered_indices[:40]):
                    (gx, gy), _, _ = node.cluster_goals_world[idx]
                    ax.text(gx, gy, str(rank), fontsize=9)

            # planned path (optional)
            if node.planned_path is not None and len(node.planned_path.poses) > 1:
                px = [p.pose.position.x for p in node.planned_path.poses]
                py = [p.pose.position.y for p in node.planned_path.poses]
                ax.plot(px, py, linewidth=2)

            # current goal highlight
            if node.current_goal_world is not None:
                gx, gy = node.current_goal_world
                ax.scatter([gx], [gy], s=140, marker="*", linewidths=2)

            # robot pose
            if node.pose is not None:
                rx, ry = float(node.pose.x), float(node.pose.y)
                th = math.radians(float(node.pose.theta))  # Pose2D.theta is degrees (your setup)
                ax.scatter([rx], [ry], s=120, marker="o")
                ax.arrow(rx, ry, 0.4 * math.cos(th), 0.4 * math.sin(th),
                         width=0.02, length_includes_head=True)

            ax.set_title(f"Frontier clusters: {len(node.clusters)} | /map updates: {node.map_count}")
            ax.set_xlabel("x [m]")
            ax.set_ylabel("y [m]")

            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            plt.pause(0.001)

    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
