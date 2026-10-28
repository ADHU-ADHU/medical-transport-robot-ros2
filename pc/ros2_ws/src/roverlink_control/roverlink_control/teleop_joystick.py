#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

import pygame
import time


def apply_deadzone(value, deadzone=0.1):
    """Apply deadzone to joystick axis value."""
    return 0.0 if abs(value) < deadzone else value


class PygameTeleop(Node):
    def __init__(self):
        super().__init__('pygame_teleop')

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.return_pub = self.create_publisher(Bool, "/trigger_return", 10)

        # --- Behavior settings ---
        self.deadzone = 0.10
        self.rate_hz = 100
        self.max_speed = 1.0
        self.max_turn_speed = 1.0

        # When return is triggered, stop publishing cmd_vel until toggled back
        self.manual_enabled = True

        # Debounce LB button
        self.lb_prev = 0
        self.last_lb_time = 0.0
        self.lb_debounce_s = 0.25

        # Initialize pygame and joystick
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() < 1:
            raise RuntimeError("No joystick connected!")

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        self.screen = pygame.display.set_mode((140, 80))
        pygame.display.set_caption("ROS2 Teleop (LB toggles Return / Manual)")
        pygame.key.set_repeat(1, 10)

        self.clock = pygame.time.Clock()
        self.running = True

        self.get_logger().info("Teleop started. LB1 toggles: trigger_return + stop cmd_vel, press again to resume.")

    def publish_stop(self, repeats=8, sleep_s=0.01):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        for _ in range(max(1, repeats)):
            self.pub.publish(msg)
            time.sleep(max(0.0, sleep_s))

    def handle_lb_toggle(self):
        """On LB rising edge: toggle manual mode; if disabling, trigger return and stop cmd_vel."""
        now = time.monotonic()

        # Debounce
        if now - self.last_lb_time < self.lb_debounce_s:
            return
        self.last_lb_time = now

        if self.manual_enabled:
            # Switch to return mode: stop manual cmd_vel publishing
            self.get_logger().info("LB pressed: Triggering RETURN and stopping cmd_vel publishing.")
            self.return_pub.publish(Bool(data=True))
            self.publish_stop(repeats=10, sleep_s=0.01)
            self.manual_enabled = False
        else:
            # Switch back to manual mode
            self.get_logger().info("LB pressed: Resuming MANUAL cmd_vel publishing.")
            self.publish_stop(repeats=5, sleep_s=0.01)
            self.manual_enabled = True

    def process_events(self):
        pygame.event.pump()

        # --- LB1 edge detection ---
        lb = int(self.joystick.get_button(4))  # LB1
        if lb == 1 and self.lb_prev == 0:
            self.handle_lb_toggle()
        self.lb_prev = lb

        # If manual mode disabled, do not publish cmd_vel
        if not self.manual_enabled:
            return

        # Read joystick axes (values -1..+1)
        joy_y_left = apply_deadzone(self.joystick.get_axis(1), self.deadzone)
        joy_x_right = apply_deadzone(self.joystick.get_axis(3), self.deadzone)

        # Forward/back (invert Y)
        lx = -joy_y_left * self.max_speed

        # Turn
        az = joy_x_right * self.max_turn_speed

        # Publish
        msg = Twist()
        msg.linear.x = float(lx)
        msg.angular.z = float(az)
        self.pub.publish(msg)

    def run(self):
        while rclpy.ok() and self.running:
            self.process_events()
            self.clock.tick(self.rate_hz)


def main():
    rclpy.init()
    node = PygameTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt: stopping cmd_vel and exiting.")
        node.publish_stop(repeats=12, sleep_s=0.01)
    finally:
        try:
            node.publish_stop(repeats=12, sleep_s=0.01)
        except Exception:
            pass
        pygame.quit()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
