#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D

import smbus   # use smbua2 if needed

# ---------- Utility Functions ----------
def angle_diff(a):
    """Wrap angle 'a' to [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

# ---------- Main Node ----------
class MD25TrajectoryFollower(Node):
    def __init__(self):
        super().__init__('md25_trajectory_follower')

        # -------- USER PARAMETERS --------
        self.i2c_bus_id = 1
        self.md25_addr = 0x58
        self.track_width = 0.35
        self.max_wheel_speed = 0.5
        self.K_v = 0.8
        self.K_omega = 2.0
        self.v_max = 0.4
        self.omega_max = 2.0
        self.waypoint_tolerance = 0.05
        self.goal_tolerance = 0.03
        self.control_dt = 0.05  # 20 Hz

        # Robot state
        self.pose_received = False
        self.x = 0.0
        self.y = 0.0
        self.theta_deg = 0.0

        # ---- Optimized Waypoints via OR-Tools ----
        self.waypoints = self.get_optimized_waypoints()
        self.waypoint_index = 0
        self.get_logger().info(f"Optimized Waypoints: {self.waypoints}")

        # ---- Setup I2C & MD25 ----
        self.bus = smbus.SMBus(self.i2c_bus_id)
        self._init_md25()

        # ---- Setup ROS interfaces ----
        self.pose_sub = self.create_subscription(
            Pose2D, '/robot/pose', self.pose_callback, 10
        )
        self.control_timer = self.create_timer(self.control_dt, self.control_loop)

        self.get_logger().info("MD25 Trajectory Follower initialised.")

    # ------------------------ GET OPTIMIZED WAYPOINTS ------------------------
    def get_optimized_waypoints(self):
        from ortools.constraint_solver import pywrapcp, routing_enums_pb2

        def dist(a, b):
            return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

        try:
            num_points = int(input("Enter number of waypoints: "))
            points = []
            for i in range(num_points):
                x = float(input(f"Enter X of point {i+1}: "))
                y = float(input(f"Enter Y of point {i+1}: "))
                points.append((x, y))

            if num_points < 2:
                self.get_logger().warn("Not enough waypoints, using as-is.")
                return points

            dist_matrix = [[dist(a, b) for b in points] for a in points]
            manager = pywrapcp.RoutingIndexManager(len(points), 1, 0)
            routing = pywrapcp.RoutingModel(manager)

            def callback(i, j):
                return int(dist_matrix[manager.IndexToNode(i)][manager.IndexToNode(j)])
            t = routing.RegisterTransitCallback(callback)
            routing.SetArcCostEvaluatorOfAllVehicles(t)

            search_params = pywrapcp.DefaultRoutingSearchParameters()
            search_params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

            solution = routing.SolveWithParameters(search_params)
            if solution is None:
                self.get_logger().error("TSP optimization failed! Using waypoints as given.")
                return points

            route = []
            index = routing.Start(0)
            while not routing.IsEnd(index):
                route.append(manager.IndexToNode(index))
                index = solution.Value(routing.NextVar(index))

            optimized = [points[i] for i in route[:-1]]
            return optimized

        except Exception as e:
            self.get_logger().error(f"Error getting waypoints: {e}")
            return []

    # ------------------------ MD25 INITIALIZATION ------------------------
    def _init_md25(self):
        current_mode = self.bus.read_byte_data(self.md25_addr, 15)
        self.get_logger().info(f"MD25 current mode: {current_mode}")

        self.bus.write_byte_data(self.md25_addr, 16, 49)
        self.bus.write_byte_data(self.md25_addr, 16, 50)
        time.sleep(0.1)

        self.bus.write_byte_data(self.md25_addr, 15, 0)
        time.sleep(0.1)

        current_mode = self.bus.read_byte_data(self.md25_addr, 15)
        self.get_logger().info(f"MD25 mode set to: {current_mode}")

        self.stop_motors()

    def _write_motor_bytes(self, m1_byte, m2_byte):
        m1_byte = clamp(int(m1_byte), 0, 255)
        m2_byte = clamp(int(m2_byte), 0, 255)
        self.bus.write_byte_data(self.md25_addr, 0, m1_byte)
        self.bus.write_byte_data(self.md25_addr, 1, m2_byte)

    def stop_motors(self):
        self._write_motor_bytes(128, 128)

    # ------------------------ ROS CALLBACK ------------------------
    def pose_callback(self, msg: Pose2D):
        self.x = msg.x
        self.y = msg.y
        self.theta_deg = msg.theta
        self.pose_received = True

    # ------------------------ CONTROL LOOP ------------------------
    def control_loop(self):
        if not self.pose_received:
            self.stop_motors()
            return

        if self.waypoint_index >= len(self.waypoints):
            self.stop_motors()
            return

        goal_x, goal_y = self.waypoints[self.waypoint_index]
        dx = goal_x - self.x
        dy = goal_y - self.y
        dist = math.hypot(dx, dy)

        is_final = (self.waypoint_index == len(self.waypoints) - 1)
        tol = self.goal_tolerance if is_final else self.waypoint_tolerance

        if dist < tol:
            self.get_logger().info(f"Reached waypoint {self.waypoint_index}: ({goal_x:.2f}, {goal_y:.2f})")
            self.waypoint_index += 1
            if self.waypoint_index >= len(self.waypoints):
                self.get_logger().info("All waypoints reached. Stopping.")
                self.stop_motors()
                return
            goal_x, goal_y = self.waypoints[self.waypoint_index]
            dx = goal_x - self.x
            dy = goal_y - self.y
            dist = math.hypot(dx, dy)

        heading_to_goal = math.atan2(dy, dx)
        theta_rad = math.radians(self.theta_deg)
        heading_error = angle_diff(heading_to_goal - theta_rad)

        v = self.K_v * dist
        omega = self.K_omega * heading_error

        if abs(heading_error) > math.radians(60.0):
            v *= 0.3

        v = clamp(v, -self.v_max, self.v_max)
        omega = clamp(omega, -self.omega_max, self.omega_max)

        if is_final and dist < self.goal_tolerance:
            self.stop_motors()
            return

        v_r = v + (omega * self.track_width / 2.0)
        v_l = v - (omega * self.track_width / 2.0)

        speed_norm_r = clamp(v_r / self.max_wheel_speed, -1.0, 1.0)
        speed_norm_l = clamp(v_l / self.max_wheel_speed, -1.0, 1.0)

        m1_byte = 128 + 127 * speed_norm_l
        m2_byte = 128 + 127 * speed_norm_r
        self._write_motor_bytes(m1_byte, m2_byte)

# ------------------------ MAIN ------------------------
def main(args=None):
    rclpy.init(args=args)
    node = MD25TrajectoryFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motors()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
