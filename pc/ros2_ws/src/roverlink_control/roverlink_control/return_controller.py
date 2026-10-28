#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan

import time
import json
import math
import os

from rclpy.qos import qos_profile_sensor_data


POSE_HISTORY_FILE = "pose_history.json"
POSE_RETURN_FILE = "pose_history_return.json"


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def angle_wrap(a: float) -> float:
    """Wrap angle to [-pi, +pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class ReturnPathController(Node):
    """
    Records /robot/pose while manual driving.
    When /trigger_return True, loads recorded path and publishes cmd_vel (normalized -1..+1)
    to drive back along the path.

    Adds collision avoidance ONLY during RETURN mode using /scan (LaserScan).
    """

    FOLLOW_PATH = 0
    AVOID_TURN = 1
    AVOID_DRIVE = 2

    def __init__(self):
        super().__init__("return_path_controller")

        # ---------- Parameters ----------
        self.declare_parameter("pose_topic", "/robot/pose")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("trigger_topic", "/trigger_return")
        self.declare_parameter("scan_topic", "/scan")

        self.declare_parameter("pose_file", POSE_HISTORY_FILE)
        self.declare_parameter("return_pose_file", POSE_RETURN_FILE)

        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("sampling_step", 20)
        self.declare_parameter("waypoint_tolerance_m", 0.35)
        self.declare_parameter("goal_tolerance_m", 0.05)

        self.declare_parameter("track_width_m", 0.35)
        self.declare_parameter("max_wheel_speed_mps", 0.75)

        # Rate limiting (normalized units per second)
        self.declare_parameter("max_forward_rate", 2.0)
        self.declare_parameter("max_turn_rate", 4.0)

        # Controller gains
        self.declare_parameter("K_v", 3.5)
        self.declare_parameter("K_w", 2.2)

        # Behavior tuning
        self.declare_parameter("turn_in_place_threshold_rad", 0.6)
        self.declare_parameter("max_forward", 1.0)
        self.declare_parameter("max_turn", 1.0)
        self.declare_parameter("invert_turn", False)
        self.declare_parameter("avoid_turn_sign", 1.0)  # -1 means "LEFT" => negative angular.z, thisis for turning only
        self.avoid_turn_sign = float(self.get_parameter("avoid_turn_sign").value)


        # --------- Collision avoidance tuning ----------
        self.declare_parameter("safety_distance_m", 0.45)      # collision threshold
        self.declare_parameter("front_cone_deg", 45.0)         # front collision cone ±deg
        self.declare_parameter("side_cone_deg", 75.0)          # side scoring cone size (each side)
        self.declare_parameter("avoid_turn_cmd", 0.70)         # normalized turn during AVOID_TURN
        self.declare_parameter("avoid_drive_fwd_cmd", 0.35)    # normalized forward during AVOID_DRIVE
        self.declare_parameter("avoid_drive_turn_cmd", 0.25)   # normalized gentle arc during AVOID_DRIVE
        self.declare_parameter("clear_hysteresis_m", 0.05)     # must be this much above safety to be "clear"
        self.declare_parameter("scan_stale_s", 0.5)            # ignore scan older than this
        self.declare_parameter("rejoin_margin_m", 0.20)        # must have this extra clearance vs waypoint distance
        self.declare_parameter("max_rejoin_waypoints", 50)     # check up to N upcoming waypoints, was 25
        self.declare_parameter("avoid_timeout_s", 8.0)         # if stuck too long, flip direction once

                # --- Anti-oscillation + commit ---
        self.declare_parameter("avoid_min_turn_time_s", 1)   # must turn at least this long before leaving AVOID_TURN
        self.declare_parameter("avoid_cooldown_s", 1.5)        # after leaving avoidance, ignore new avoidance triggers
        self.declare_parameter("avoid_commit_fwd_s", 1.5)      # after leaving avoidance, force some forward motion, was 1.5
        self.declare_parameter("avoid_commit_fwd_cmd", 0.25)   # forward during commit

        # --- "bearing free" steering ---
        self.declare_parameter("bearing_free_margin_m", 0.7) # similar to rejoin_margin but used for bearing checks, was 0.2
        self.declare_parameter("bearing_max_abs_deg", 120.0)  # only consider waypoints within +/- this bearing

        self.avoid_min_turn_time_s = float(self.get_parameter("avoid_min_turn_time_s").value)
        self.avoid_cooldown_s = float(self.get_parameter("avoid_cooldown_s").value)
        self.avoid_commit_fwd_s = float(self.get_parameter("avoid_commit_fwd_s").value)
        self.avoid_commit_fwd_cmd = float(self.get_parameter("avoid_commit_fwd_cmd").value)

        self.bearing_free_margin_m = float(self.get_parameter("bearing_free_margin_m").value)
        self.bearing_max_abs_rad = math.radians(float(self.get_parameter("bearing_max_abs_deg").value))

        self.avoid_cooldown_until = 0.0
        self.commit_until = 0.0


        # ---------- Read parameters ----------
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.trigger_topic = str(self.get_parameter("trigger_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)

        self.pose_file = str(self.get_parameter("pose_file").value)
        self.return_pose_file = str(self.get_parameter("return_pose_file").value)

        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.sampling_step = int(self.get_parameter("sampling_step").value)
        self.waypoint_tolerance_m = float(self.get_parameter("waypoint_tolerance_m").value)
        self.goal_tolerance_m = float(self.get_parameter("goal_tolerance_m").value)

        self.track_width_m = float(self.get_parameter("track_width_m").value)
        self.max_wheel_speed_mps = float(self.get_parameter("max_wheel_speed_mps").value)

        self.max_forward_rate = float(self.get_parameter("max_forward_rate").value)
        self.max_turn_rate = float(self.get_parameter("max_turn_rate").value)

        self.K_v = float(self.get_parameter("K_v").value)
        self.K_w = float(self.get_parameter("K_w").value)

        self.turn_in_place_threshold = float(self.get_parameter("turn_in_place_threshold_rad").value)
        self.max_forward = float(self.get_parameter("max_forward").value)
        self.max_turn = float(self.get_parameter("max_turn").value)
        self.invert_turn = bool(self.get_parameter("invert_turn").value)

        self.safety_distance_m = float(self.get_parameter("safety_distance_m").value)
        self.front_cone_rad = math.radians(float(self.get_parameter("front_cone_deg").value))
        self.side_cone_rad = math.radians(float(self.get_parameter("side_cone_deg").value))
        self.avoid_turn_cmd = float(self.get_parameter("avoid_turn_cmd").value)
        self.avoid_drive_fwd_cmd = float(self.get_parameter("avoid_drive_fwd_cmd").value)
        self.avoid_drive_turn_cmd = float(self.get_parameter("avoid_drive_turn_cmd").value)
        self.clear_hysteresis_m = float(self.get_parameter("clear_hysteresis_m").value)
        self.scan_stale_s = float(self.get_parameter("scan_stale_s").value)
        self.rejoin_margin_m = float(self.get_parameter("rejoin_margin_m").value)
        self.max_rejoin_waypoints = int(self.get_parameter("max_rejoin_waypoints").value)
        self.avoid_timeout_s = float(self.get_parameter("avoid_timeout_s").value)

        # ---------- ROS pubs/subs ----------
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.pose_sub = self.create_subscription(Pose2D, self.pose_topic, self.pose_callback, 50)
        self.trigger_sub = self.create_subscription(Bool, self.trigger_topic, self.trigger_callback, 10)

        # LaserScan: use sensor QoS
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data
        )

        # ---------- State ----------
        self.pose_history = []
        self.return_pose_history = []

        self.have_pose = False
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # radians (Pose2D.theta is DEGREES per your note)

        self.return_mode = False
        self.waypoints = []
        self.waypoint_index = 0
        self.return_timer = None

        # Command smoothing state
        self.prev_forward = 0.0
        self.prev_turn = 0.0
        self.prev_time = time.time()

        # Scan storage
        self.have_scan = False
        self.last_scan_time = 0.0
        self.scan_angle_min = 0.0
        self.scan_angle_inc = 0.0
        self.scan_range_max = 12.0
        self.scan_ranges = []

        # Avoidance state machine
        self.avoid_state = self.FOLLOW_PATH
        self.avoid_dir = 0          # +1 left, -1 right
        self.avoid_started_t = 0.0
        self.avoid_flipped = False
        self._warned_no_scan = False
        self._last_info_log_t = 0.0

        self.get_logger().info("ReturnPathController ready. Recording pose history in manual mode.")

    # ---------------- Pose recording ----------------
    def pose_callback(self, msg: Pose2D):
        self.x = float(msg.x)
        self.y = float(msg.y)

        # IMPORTANT: you said Pose2D.theta is DEGREES
        self.theta = math.radians(float(msg.theta))
        self.have_pose = True

        if not self.return_mode:
            self.pose_history.append({"x": self.x, "y": self.y, "theta": self.theta})
        else:
            self.return_pose_history.append({"x": self.x, "y": self.y, "theta": self.theta})

    # ---------------- Scan ----------------
    def scan_callback(self, msg: LaserScan):
        self.have_scan = True
        self.last_scan_time = time.time()
        self.scan_angle_min = float(msg.angle_min)
        self.scan_angle_inc = float(msg.angle_increment)
        self.scan_range_max = float(msg.range_max)
        self.scan_ranges = list(msg.ranges)

    def _scan_is_fresh(self) -> bool:
        return self.have_scan and ((time.time() - self.last_scan_time) <= self.scan_stale_s) and len(self.scan_ranges) > 0

    def _range_at_angle(self, ang: float) -> float:
        """
        Return laser range at given angle (radians) in scan frame.
        Handles wrap and clamps to valid indices.
        """
        if not self._scan_is_fresh():
            return float("inf")

        # Normalize angle into scan span using wrap-around (common when angle_min=0..2pi)
        # Compute index:
        # idx = round((ang - angle_min)/inc) modulo N
        n = len(self.scan_ranges)
        inc = self.scan_angle_inc if abs(self.scan_angle_inc) > 1e-12 else 1e-12

        # Wrap to [0, 2pi)
        a = ang % (2.0 * math.pi)
        a0 = self.scan_angle_min % (2.0 * math.pi)

        # delta in [0,2pi)
        d = (a - a0) % (2.0 * math.pi)
        idx = int(round(d / inc)) % n

        r = self.scan_ranges[idx]
        if r is None or not math.isfinite(r) or r <= 0.0:
            return self.scan_range_max
        return float(r)

    def _sector_stat(self, center: float, half_width: float, mode: str = "min") -> float:
        """
        Sector statistic around 'center' with ±half_width.
        mode: "min" or "mean"
        """
        if not self._scan_is_fresh():
            return float("inf") if mode == "min" else 0.0

        # Sample across sector (every k points) to be cheap
        n = len(self.scan_ranges)
        inc = self.scan_angle_inc if abs(self.scan_angle_inc) > 1e-12 else 1e-12

        # Number of samples based on angular width
        span = max(inc, 2.0 * half_width)
        steps = int(max(5, min(80, span / inc)))  # clamp steps
        if steps <= 0:
            steps = 5

        vals = []
        for i in range(-steps, steps + 1):
            a = center + (i / steps) * half_width
            r = self._range_at_angle(a)
            # cap to range_max for scoring
            r = min(r, self.scan_range_max)
            vals.append(r)

        if not vals:
            return float("inf") if mode == "min" else 0.0

        if mode == "min":
            return min(vals)
        else:
            return sum(vals) / float(len(vals))

    def _front_blocked(self) -> bool:
        front_min = self._sector_stat(center=0.0, half_width=self.front_cone_rad, mode="min")
        return front_min < self.safety_distance_m

    def _choose_avoid_direction(self) -> int:
        """
        Pick the side with larger average clearance.
        Returns +1 for left, -1 for right.
        Uses side cones centered at +90deg and -90deg (or 270deg).
        """
        # Left side around +90deg
        left_mean = self._sector_stat(center=math.pi / 2.0, half_width=self.side_cone_rad, mode="mean")
        # Right side around -90deg (wrap handled by _range_at_angle)
        right_mean = self._sector_stat(center=-math.pi / 2.0, half_width=self.side_cone_rad, mode="mean")

        # If equal-ish, default to left
        dir_ = +1 if left_mean >= right_mean else -1
        self.get_logger().warn(
            f"[AVOID] Side choice: left_mean={left_mean:.2f}m right_mean={right_mean:.2f}m => "
            f"{'LEFT' if dir_ > 0 else 'RIGHT'}"
        )
        return dir_

    def _bearing_to_waypoint(self, wx: float, wy: float) -> (float, float):
        dx = wx - self.x
        dy = wy - self.y
        dist = math.hypot(dx, dy)
        desired_heading = math.atan2(dy, dx)
        rel = angle_wrap(desired_heading - self.theta)  # relative to robot forward
        return rel, dist

    def _waypoint_visible(self, wx: float, wy: float) -> bool:
        rel_ang, dist = self._bearing_to_waypoint(wx, wy)

        # Only consider waypoints not too far behind
        if abs(rel_ang) > self.bearing_max_abs_rad:
            return False

        laser_r = self._range_at_angle(rel_ang)
        return laser_r > (dist + self.bearing_free_margin_m)

    def _find_rejoin_index(self) -> int:
        """
        Search upcoming waypoints for the nearest visible one.
        Returns waypoint index to jump to, or -1 if none.
        """
        if not self.waypoints or self.waypoint_index >= len(self.waypoints):
            return -1

        end = min(len(self.waypoints), self.waypoint_index + self.max_rejoin_waypoints)
        best_i = -1
        best_dist = float("inf")

        for i in range(self.waypoint_index, end):
            wx, wy = self.waypoints[i]
            rel_ang, dist = self._bearing_to_waypoint(wx, wy)
            if dist < 1e-6:
                return i
            if self._waypoint_visible(wx, wy):
                # prefer nearer visible waypoint
                if dist < best_dist:
                    best_dist = dist
                    best_i = i

        # If best is the current waypoint, try the next one (helps stop "stuck at same wp")
        if best_i == self.waypoint_index and (best_i + 1) < len(self.waypoints):
            best_i = best_i + 1


        return best_i

    def _exit_avoidance_to_follow(self, new_index: int, reason: str):
        self.waypoint_index = int(new_index)
        self.avoid_state = self.FOLLOW_PATH

        now = time.time()
        self.avoid_cooldown_until = now + self.avoid_cooldown_s
        self.commit_until = now + self.avoid_commit_fwd_s

        self.get_logger().warn(
            f"[AVOID] Exit -> FOLLOW (reason={reason}) wp_index={self.waypoint_index} "
            f"cooldown={self.avoid_cooldown_s:.2f}s commit={self.avoid_commit_fwd_s:.2f}s"
        )


    # ---------------- Saving/loading ----------------
    def save_pose_history(self):
        try:
            with open(self.pose_file, "w") as f:
                json.dump(self.pose_history, f, indent=2)
            self.get_logger().info(f"Saved forward pose history: {len(self.pose_history)} samples -> {self.pose_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to save pose history: {e}")

    def save_return_history(self):
        try:
            with open(self.return_pose_file, "w") as f:
                json.dump(self.return_pose_history, f, indent=2)
            self.get_logger().info(
                f"Saved return pose history: {len(self.return_pose_history)} samples -> {self.return_pose_file}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to save return pose history: {e}")

    def load_return_path(self):
        if not os.path.exists(self.pose_file):
            self.get_logger().error(f"No pose file found: {self.pose_file}")
            return False

        try:
            with open(self.pose_file, "r") as f:
                data = json.load(f)

            if not data:
                self.get_logger().error("Pose file is empty.")
                return False

            data_rev = data[::-1]
            step = max(1, int(self.sampling_step))
            sampled = data_rev[::step]
            self.waypoints = [(float(p["x"]), float(p["y"])) for p in sampled]

            # Optional origin target
            self.waypoints.append((0.0, 0.0))

            self.waypoint_index = 0
            self.get_logger().info(f"Loaded return path: {len(self.waypoints)} waypoints (step={step}).")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to load return path: {e}")
            return False

    # ---------------- Trigger ----------------
    def trigger_callback(self, msg: Bool):
        if msg.data and not self.return_mode:
            self.get_logger().info("Trigger received: switching to RETURN mode.")

            self.save_pose_history()

            ok = self.load_return_path()
            if not ok:
                self.get_logger().error("Return mode aborted (no valid path).")
                return

            self.return_mode = True

            # Reset avoidance state
            self.avoid_state = self.FOLLOW_PATH
            self.avoid_dir = 0
            self.avoid_flipped = False
            self.commit_until = 0.0
            self.avoid_cooldown_until = 0.0
            


            # Start return control loop
            period = 1.0 / max(1e-6, self.control_rate_hz)
            self.return_timer = self.create_timer(period, self.return_control_loop)

            self.publish_stop(repeats=5, sleep_s=0.02)
            self.get_logger().info("RETURN mode started. (Stop joystick publisher to avoid cmd_vel conflicts.)")

    # ---------------- cmd_vel publishing helpers ----------------
    def publish_cmd(self, forward_norm: float, turn_norm: float):
        msg = Twist()
        msg.linear.x = float(clamp(forward_norm, -1.0, 1.0))
        msg.angular.z = float(clamp(turn_norm, -1.0, 1.0))
        self.cmd_pub.publish(msg)

    def publish_stop(self, repeats: int = 10, sleep_s: float = 0.02):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        for _ in range(max(1, repeats)):
            self.cmd_pub.publish(msg)
            time.sleep(max(0.0, sleep_s))

    # ---------------- Return control loop ----------------
    def return_control_loop(self):
        if not self.have_pose:
            return

        if not self._scan_is_fresh() and self.return_mode and not self._warned_no_scan:
            self.get_logger().warn(
                "[SCAN] No fresh /scan data yet (or stale). Collision avoidance will be inactive until scan arrives."
            )
            self._warned_no_scan = True

        if self.waypoint_index >= len(self.waypoints):
            self.publish_stop(repeats=8, sleep_s=0.01)
            self.save_return_history()
            self.get_logger().info("Return complete. Stopping.")
            self.return_mode = False
            if self.return_timer is not None:
                self.return_timer.cancel()
            return

        # ---- Collision avoidance: only in RETURN mode ----
        if self.return_mode and self._scan_is_fresh():
            blocked = self._front_blocked()

            # Start avoidance if blocked while following
            if blocked and self.avoid_state == self.FOLLOW_PATH and time.time() >= self.avoid_cooldown_until:
                self.avoid_dir = self._choose_avoid_direction()
                self.avoid_state = self.AVOID_TURN
                self.avoid_started_t = time.time()
                self.avoid_flipped = False
                self.commit_until = 0.0
                self.get_logger().warn(
                    f"[AVOID] FRONT BLOCKED (<{self.safety_distance_m:.2f}m). "
                    f"Entering AVOID_TURN dir={'LEFT' if self.avoid_dir > 0 else 'RIGHT'}"
                )

            # If in avoidance too long, flip ONCE (prevents endless stuck, avoids oscillation)
            if self.avoid_state in (self.AVOID_TURN, self.AVOID_DRIVE):
                if (time.time() - self.avoid_started_t) > self.avoid_timeout_s and not self.avoid_flipped:
                    self.avoid_dir *= -1
                    self.avoid_flipped = True
                    self.avoid_started_t = time.time()
                    self.get_logger().error(
                        f"[AVOID] Timeout. Flipping direction ONCE to {'LEFT' if self.avoid_dir > 0 else 'RIGHT'}"
                    )

        # ---- Decide control based on state ----
        forward = 0.0
        turn = 0.0

        # ===================== AVOIDANCE STATES =====================
        if self.return_mode and self._scan_is_fresh() and self.avoid_state != self.FOLLOW_PATH:
            front_min = self._sector_stat(center=0.0, half_width=self.front_cone_rad, mode="min")
            front_clear = front_min >= (self.safety_distance_m + self.clear_hysteresis_m)

            if self.avoid_state == self.AVOID_TURN:
                # Turn in chosen direction until front is clear (with hysteresis)
                forward = 0.0
                #turn = clamp(self.avoid_dir * self.avoid_turn_cmd, -1.0, 1.0)
                turn = clamp(self.avoid_turn_sign * self.avoid_dir * self.avoid_turn_cmd, -1.0, 1.0)


                if front_clear and (time.time() - self.avoid_started_t) >= self.avoid_min_turn_time_s:
                    # If any untravelled waypoint bearing is free, go to the closest one
                    rejoin_i = self._find_rejoin_index()
                    if rejoin_i != -1:
                        self._exit_avoidance_to_follow(rejoin_i, "turn_front_clear_waypoint_free")
                    else:
                        self.avoid_state = self.AVOID_DRIVE
                        self.get_logger().warn("[AVOID] Front clear but no waypoint bearing is free. Switching to AVOID_DRIVE.")


            elif self.avoid_state == self.AVOID_DRIVE:
                # Drive forward with a gentle arc in chosen direction until a waypoint becomes visible
                if not front_clear:
                    # If front becomes blocked again, go back to turning (still same direction)
                    self.avoid_state = self.AVOID_TURN
                    self.get_logger().warn("[AVOID] Front blocked again during drive. Switching back to AVOID_TURN.")
                    forward = 0.0
                    #turn = clamp(self.avoid_dir * self.avoid_turn_cmd, -1.0, 1.0)
                    turn = clamp(self.avoid_turn_sign * self.avoid_dir * self.avoid_turn_cmd, -1.0, 1.0)
                    
                    

                else:
                    forward = clamp(self.avoid_drive_fwd_cmd, 0.0, 1.0)
                    #turn = clamp(self.avoid_dir * self.avoid_drive_turn_cmd, -1.0, 1.0)
                    turn = clamp(self.avoid_turn_sign * self.avoid_dir * self.avoid_drive_turn_cmd, -1.0, 1.0)


                    rejoin_i = self._find_rejoin_index()
                                         
                    if rejoin_i != -1:
                        # Turn toward that waypoint (by exiting to FOLLOW, commit window will push forward)
                        self._exit_avoidance_to_follow(rejoin_i, "drive_waypoint_bearing_free")


            # Throttled info log
            nowt = time.time()
            if nowt - self._last_info_log_t > 0.7:
                self._last_info_log_t = nowt
                self.get_logger().info(
                    f"[AVOID] state={self.avoid_state} dir={'L' if self.avoid_dir > 0 else 'R'} "
                    f"front_min={front_min:.2f}m wp_index={self.waypoint_index}"
                )

        # ===================== NORMAL PATH FOLLOWING =====================
        else:
            goal_x, goal_y = self.waypoints[self.waypoint_index]
            dx = goal_x - self.x
            dy = goal_y - self.y
            dist = math.hypot(dx, dy)

            is_last = (self.waypoint_index == len(self.waypoints) - 1)
            tol = self.goal_tolerance_m if is_last else self.waypoint_tolerance_m

            if dist <= tol:
                self.waypoint_index += 1
                return

            desired_heading = math.atan2(dy, dx)
            heading_error = angle_wrap(desired_heading - self.theta)

            if self.invert_turn:
                heading_error = -heading_error

            # Normalized control -> convert to wheel speeds -> back to normalized forward/turn
            v_des = clamp(self.K_v * dist, 0.0, 1.0) * self.max_wheel_speed_mps

            omega_max = (2.0 * self.max_wheel_speed_mps) / max(1e-6, self.track_width_m)
            omega_des = clamp(self.K_w * heading_error, -omega_max, omega_max)

            # Smooth curve behaviour
            align = 1.0 - (abs(heading_error) / max(1e-6, self.turn_in_place_threshold))
            align = clamp(align, 0.0, 1.0)
            v_des *= align

            v_r = v_des + omega_des * (self.track_width_m / 2.0)
            v_l = v_des - omega_des * (self.track_width_m / 2.0)

            u_r = clamp(v_r / self.max_wheel_speed_mps, -1.0, 1.0)
            u_l = clamp(v_l / self.max_wheel_speed_mps, -1.0, 1.0)

            forward = clamp((u_l + u_r) * 0.5, -1.0, 1.0)
            turn = clamp((u_l - u_r) * 0.5, -1.0, 1.0)
            now = time.time()
            if self.return_mode and now < self.commit_until:
                forward = max(forward, self.avoid_commit_fwd_cmd)
                self.get_logger().info(f"[COMMIT] forcing forward={forward:.2f} (until {self.commit_until:.2f})")


            # Throttled info
            nowt = time.time()
            if nowt - self._last_info_log_t > 0.7:
                self._last_info_log_t = nowt
                self.get_logger().info(
                    f"[FOLLOW] wp={self.waypoint_index}/{len(self.waypoints)} dist={dist:.2f} "
                    f"head_err={heading_error:.2f} fwd={forward:.2f} turn={turn:.2f}"
                )

        # ---------------- Rate limiting (anti-jerk + anti-oscillation) ----------------
        now = time.time()
        dt = max(1e-3, now - self.prev_time)

        df = clamp(forward - self.prev_forward, -self.max_forward_rate * dt, self.max_forward_rate * dt)
        dw = clamp(turn - self.prev_turn, -self.max_turn_rate * dt, self.max_turn_rate * dt)

        forward = self.prev_forward + df
        turn = self.prev_turn + dw

        self.prev_forward = forward
        self.prev_turn = turn
        self.prev_time = now

        self.publish_cmd(forward, turn)

    # ---------------- Shutdown ----------------
    def stop_everything(self):
        try:
            self.publish_stop(repeats=15, sleep_s=0.01)
        except Exception:
            pass


def main():
    rclpy.init()
    node = ReturnPathController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt: publishing stop and shutting down...")
        node.stop_everything()
    finally:
        try:
            if node.pose_history and not node.return_mode:
                node.save_pose_history()
                node.save_return_history()
        except Exception:
            pass

        node.stop_everything()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
