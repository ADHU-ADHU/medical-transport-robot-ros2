#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import smbus
import time
import atexit
import json
import os
import math

from scipy.ndimage import gaussian_filter1d  # >>> ADDED

ADDR = 0x58
STOP = 128
MAX_FWD = 175
MAX_REV = 81
RANGE = MAX_FWD - STOP
POSE_HISTORY_FILE = "pose_history.json"
POSE_RETURN_FILE = "pose_history_return.json"


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def angle_diff(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class MD25Controller(Node):
    def __init__(self):
        super().__init__("md25_combined_controller")

        # ---------------- GENERAL SETUP ----------------
        self.bus = smbus.SMBus(1)
        self.bus.write_byte_data(ADDR, 15, 0)
        self.bus.write_byte_data(ADDR, 16, 50)
        self.bus.write_byte_data(ADDR, 16, 49)
        self.bus.write_byte_data(ADDR, 14, 5)

        # ------- for return motor speed max and min 
        self.rt_speed_max = 175
        self.rt_speed_min = 81

        self.get_logger().info("MD25 initialized in Mode 0")

        # --------- Gaussian Filter ---- #
        self.sigma = 100  # Gaussian std dev for path smoothing

        # ---------------- MANUAL DRIVING SETUP ----------------
        self.sub = self.create_subscription(Twist, "cmd_vel", self.cmd_callback, 10)
        self.pose_history = []
        self.pose_sub = self.create_subscription(Pose2D, "/robot/pose", self.pose_callback, 10)

        self.last_msg_time = time.time()
        self.watchdog_timer = self.create_timer(0.05, self.watchdog)

        # ---------------- RETURN MODE SETUP ----------------
        self.return_mode = False
        self.return_pose_history = []
        self.return_timer = None

        self.sampling_step = 10
        self.track_width = 0.35
        self.max_wheel_speed = 0.5
        self.K_v = 2 # 0.8
        self.K_omega = 2.2  # 0.2
        #self.v_max = 0.4
        #self.omega_max = 2.0
        self.waypoint_tolerance = 0.05
        self.goal_tolerance = 0.03
        self.waypoints = []
        self.waypoint_index = 0

        #  Laser scan
        self.obstacle_distance_threshold = 0.3  # meters form topic /scan ranges:   max range is 12 metres
        self.obstacle_detected = False
        self.scan_sub = self.create_subscription( LaserScan, "/scan", self.scan_callback, 10 )

        # ---------------- TRIGGER RETURN ----------------
        self.trigger_sub = self.create_subscription(Bool, "/trigger_return", self.trigger_callback, 10)

        atexit.register(self.safe_stop)
        atexit.register(self.save_return_history)  # >>> ADDED

        self.get_logger().info("MD25 Combined Control Node Initialized")

    # -------------------------------------------------------------------
    # MODE SWITCH
    # -------------------------------------------------------------------
    def trigger_callback(self, msg: Bool):
        if msg.data and not self.return_mode:
            self.get_logger().info("Trigger received! Switching to return mode...")
            # self.bus.write_byte_data(ADDR, 14, 1)
            self.return_mode = True

            self.watchdog_timer.cancel()
            self.save_pose_history()

            self.load_return_path()
            self.return_timer = self.create_timer(0.05, self.return_control_loop)
            self.get_logger().info("Return mode started.")

    # -------------------------------------------------------------------
    # MANUAL MODE FUNCTIONS
    # -------------------------------------------------------------------
    def safe_stop(self):
        try:
            self.bus.write_byte_data(ADDR, 0, STOP)
            self.bus.write_byte_data(ADDR, 1, STOP)
        except Exception as e:
            self.get_logger().error(f"Motor stop failure: {e}")

    def set_motors(self, left, right):
        try:
            self.bus.write_byte_data(ADDR, 0, left)
            self.bus.write_byte_data(ADDR, 1, right)
        except Exception as e:
            self.get_logger().error(f"Failed to set motor speeds: {e}")

    def scale(self, v):
        return max(MAX_REV, min(MAX_FWD, int(STOP + v * RANGE)))

    def _write_motor_bytes(self, m1_byte, m2_byte):
        """Directly send motor speed bytes to MD25 (0-255 range)."""
        m1_byte = clamp(int(m1_byte), self.rt_speed_min, self.rt_speed_max) # speed for motor left
        m2_byte = clamp(int(m2_byte), self.rt_speed_min, self.rt_speed_max) # Speed for motor right

        try:
            self.bus.write_byte_data(ADDR, 0, m1_byte)  # Left motor
            self.bus.write_byte_data(ADDR, 1, m2_byte)  # Right motor
        except Exception as e:
            self.get_logger().error(f"Failed to write motor bytes: {e}")

    def watchdog(self):
        if time.time() - self.last_msg_time > 0.2:
            self.set_motors(STOP, STOP)

    def cmd_callback(self, msg: Twist):
        if self.return_mode:  # Ignore commands in return mode
            return
        self.last_msg_time = time.time()

        fwd = max(-1, min(1, msg.linear.x))
        turn = max(-1, min(1, msg.angular.z))
        left = self.scale(fwd + turn)
        right = self.scale(fwd - turn)

        self.set_motors(left, right)

    def pose_callback(self, msg: Pose2D):
        if not self.return_mode:
            self.pose_history.append({'x': msg.x, 'y': msg.y, 'theta': msg.theta})
        else:
            self.return_pose_history.append({'x': msg.x, 'y': msg.y, 'theta': msg.theta})
        self.x, self.y, self.theta_deg = msg.x, msg.y, msg.theta

    def save_pose_history(self):
        try:
            with open(POSE_HISTORY_FILE, "w") as f:
                json.dump(self.pose_history, f, indent=4)
            self.get_logger().info("Forward path saved.")
        except Exception as e:
            self.get_logger().error(f"Failed to save forward path: {e}")
    
    def scan_callback(self, msg: LaserScan):        
        """
        Checking the front 180° (-90° to +90°) / (0 - 180°) for obstacles closer than threshold.
        """
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges

        half_fov = math.radians(90.0)  # 180° total (±90°)
        obstacle = False

        for i, r in enumerate(ranges):
            angle = angle_min + i * angle_increment
            if -half_fov <= angle <= half_fov:
                if math.isfinite(r) and r > 0.0 and r < self.obstacle_distance_threshold:
                    obstacle = True
                    break

        self.obstacle_detected = obstacle


    # >>> ADDED
    def save_return_history(self):
        """
        Save the return pose history to POSE_RETURN_FILE.
        Called both when the return is complete and on shutdown / KeyboardInterrupt.
        """
        if not self.return_pose_history:
            return  # nothing to save
        try:
            with open(POSE_RETURN_FILE, "w") as f:
                json.dump(self.return_pose_history, f, indent=4)
            self.get_logger().info("Return path history saved.")
        except Exception as e:
            self.get_logger().error(f"Failed to save return path history: {e}")

    # -------------------------------------------------------------------
    # RETURN MODE FUNCTIONS
    # -------------------------------------------------------------------
    def load_return_path(self):
        try:
            with open(POSE_HISTORY_FILE, "r") as f:
                data = json.load(f)

            if not data:
                self.get_logger().error("Pose history is empty, cannot build return path.")
                return

            # Reverse so we start returning from the last forward pose
            data_rev = data[::-1]

            # --- Gaussian smoothing BEFORE sampling ---  >>> CHANGED/ADDED
            x_vals = [p["x"] for p in data_rev]
            y_vals = [p["y"] for p in data_rev]

            
            x_gauss = gaussian_filter1d(x_vals, self.sigma)
            y_gauss = gaussian_filter1d(y_vals, self.sigma)

            # Keep first and last point unchanged
            x_gauss[0], x_gauss[-1] = x_vals[0], x_vals[-1]
            y_gauss[0], y_gauss[-1] = y_vals[0], y_vals[-1]

            smoothed_path = [
                {"x": float(x_gauss[i]), "y": float(y_gauss[i])}
                for i in range(len(data_rev))
            ]

            # Now sample the smoothed path
            path = smoothed_path[::self.sampling_step]

            # Final pose (origin)
            path.append({"x": 0.0, "y": 0.0})

            self.waypoints = [(p["x"], p["y"]) for p in path]
            self.get_logger().info("Return path loaded (smoothed + sampled).")
        except Exception as e:
            self.get_logger().error(f"Failed to load return path: {e}")

    def return_control_loop(self):

        # --- Obstacle stop logic at the beginning ---
        if self.obstacle_detected:
            self.get_logger().warn(" ⚠️🛑 Obstacle detected in front. Pausing return motion.")
            self.safe_stop()
            return

        if self.waypoint_index >= len(self.waypoints):
            self.safe_stop()
            self.save_return_history()  # >>> CHANGED (use the new helper)
            self.get_logger().info("Return complete. Motors stopped.")
            return

        goal_x, goal_y = self.waypoints[self.waypoint_index]
        dx, dy = goal_x - self.x, goal_y - self.y
        dist = math.hypot(dx, dy)

        if dist < (self.goal_tolerance if self.waypoint_index == len(self.waypoints) - 1 else self.waypoint_tolerance):
            self.waypoint_index += 1
            return

        heading_to_goal = math.atan2(dy, dx)
        heading_error = angle_diff(heading_to_goal - math.radians(self.theta_deg))

        v = self.K_v * dist
        omega = self.K_omega * heading_error
        v_r = v + (omega * self.track_width / 2)
        v_l = v - (omega * self.track_width / 2)

        # ---- Normalize to [-1, 1] and convert to bytes ----
        speed_norm_r = clamp(v_r / self.max_wheel_speed, -1.0, 1.0)
        speed_norm_l = clamp(v_l / self.max_wheel_speed, -1.0, 1.0)

        m1_byte = 128 + 127 * speed_norm_l   # Left
        m2_byte = 128 + 127 * speed_norm_r   # Right

        self._write_motor_bytes(m1_byte, m2_byte)


def main():
    rclpy.init()
    node = MD25Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, shutting down...")
    finally:
        # >>> ADDED: always try to save the return history
        try:
            node.save_return_history()
        except Exception as e:
            node.get_logger().error(f"Error while saving return history on shutdown: {e}")

        node.safe_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
