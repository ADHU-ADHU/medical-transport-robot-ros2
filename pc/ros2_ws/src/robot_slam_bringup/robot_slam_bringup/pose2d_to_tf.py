#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, TransformStamped
from tf2_ros import TransformBroadcaster

class Pose2DTfBroadcaster(Node):
    def __init__(self):
        super().__init__('pose2d_tf_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Pose2D,
            '/robot/pose',          # Pose2D from your robot
            self.pose_callback,
            10
        )

    def pose_callback(self, msg: Pose2D):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Position (meters)
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        # Orientation: theta is in DEGREES, convert to radians and then to quaternion
        yaw = math.radians(msg.theta)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sy
        t.transform.rotation.w = cy

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = Pose2DTfBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
