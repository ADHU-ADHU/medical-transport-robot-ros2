#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import smbus
import time
import atexit

ADDR = 0x58
STOP = 128


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class MD25Controller(Node):
    def __init__(self):
        super().__init__("md25_controller")

        # --------- Parameters you care about ----------
        self.declare_parameter("speed_limit_steps", 47)   # STOP +/- steps (47 -> 175/81) for max forward and max reverse speed
        self.declare_parameter("deadband", 0.05)          # To ignore tiny cmd_vel noise
        self.declare_parameter("invert_turn", True)       # set True if turning is reversed
        self.declare_parameter("watchdog_timeout_s", 0.2) # How long can the robot move without receiving a cmd_vel message before we force stop
                                                          # So if no cmd_vel is received in 0.2 sec then tje robot stops 
        self.declare_parameter("watchdog_period_s", 0.05) # How often the above condition is checked.

        self.speed_limit_steps = int(self.get_parameter("speed_limit_steps").value)
        self.speed_limit_steps = int(clamp(self.speed_limit_steps, 1, 127))

        self.deadband = float(self.get_parameter("deadband").value)
        self.invert_turn = bool(self.get_parameter("invert_turn").value)

        self.watchdog_timeout_s = float(self.get_parameter("watchdog_timeout_s").value)
        self.watchdog_period_s = float(self.get_parameter("watchdog_period_s").value)

        # Derived bounds
        self.max_fwd_byte = int(clamp(STOP + self.speed_limit_steps, 0, 255))
        self.max_rev_byte = int(clamp(STOP - self.speed_limit_steps, 0, 255))

        # I2C / MD25 init
        self.bus = smbus.SMBus(1)
        try:
            self.bus.write_byte_data(ADDR, 15, 0)   # mode 0
            #self.bus.write_byte_data(ADDR, 16, 32)  # reset encoders
            self.bus.write_byte_data(ADDR, 16, 50)  # disable 2s timeout
            self.bus.write_byte_data(ADDR, 16, 49)  # automatic speed regulation

            self.get_logger().info(
                f"MD25 ready. Bytes: REV={self.max_rev_byte}, STOP={STOP}, FWD={self.max_fwd_byte}"
            )
        except Exception as e:
            self.get_logger().error(f"MD25 init failed: {e}")

        # Subscriber
        self.sub = self.create_subscription(Twist, "cmd_vel", self.cmd_callback, 10)

        # Watchdog + safe shutdown
        self.last_msg_time = time.time()
        self.create_timer(self.watchdog_period_s, self.watchdog)
        atexit.register(self.safe_stop)

    def safe_stop(self):
        # To stop motors safely.
        try:
            self.bus.write_byte_data(ADDR, 0, STOP)
            self.bus.write_byte_data(ADDR, 1, STOP)
            print("Motors stopped safely.")
        except Exception as e:
            try:
                self.get_logger().error(f"Failed to stop motors: {e}")
            except Exception:
                print(f"Failed to stop motors: {e}")

    def set_motors(self, left: int, right: int):
        """Write motor bytes 0..255."""
        left_i = int(clamp(int(left), 0, 255))
        right_i = int(clamp(int(right), 0, 255))
        try:
            self.bus.write_byte_data(ADDR, 0, left_i)
            self.bus.write_byte_data(ADDR, 1, right_i)
        except Exception as e:
            self.get_logger().error(f"Failed to set motor speeds: {e}")

    def scale_norm_to_byte(self, u: float) -> int:
        """
        Maping normalized command u in [-1, +1] to MD25 byte:
          u=-1 -> STOP - speed_limit_steps
          u= 0 -> STOP
          u=+1 -> STOP + speed_limit_steps
        """
        u = clamp(u, -1.0, 1.0)
        value = int(round(STOP + u * self.speed_limit_steps))
        return int(clamp(value, self.max_rev_byte, self.max_fwd_byte))

    def watchdog(self):
        """Stop motors if cmd_vel hasn't been received recently."""
        if time.time() - self.last_msg_time > self.watchdog_timeout_s:
            self.set_motors(STOP, STOP)

    def cmd_callback(self, msg: Twist):
        """
          msg.linear.x  in [-1, +1]
          msg.angular.z in [-1, +1]
        """
        self.last_msg_time = time.time()

        forward = float(msg.linear.x)
        turn = float(msg.angular.z)
        if self.invert_turn:
            turn = -turn

        # Deadband
        if abs(forward) < self.deadband:
            forward = 0.0
        if abs(turn) < self.deadband:
            turn = 0.0

        # Differential mixing
        left_cmd = clamp(forward - turn, -1.0, 1.0)
        right_cmd = clamp(forward + turn, -1.0, 1.0)

        # Convert to bytes
        left_byte = self.scale_norm_to_byte(left_cmd)
        right_byte = self.scale_norm_to_byte(right_cmd)

        self.set_motors(left_byte, right_byte)

        # Comment out if too chatty
        self.get_logger().info(
            f"cmd_norm fwd={forward:.2f} turn={turn:.2f} -> L={left_cmd:.2f} R={right_cmd:.2f} -> bytes {left_byte},{right_byte}"
        )


def main():
    rclpy.init()
    node = MD25Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt caught. Stopping motors and shutting down...")
    finally:
        node.safe_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
