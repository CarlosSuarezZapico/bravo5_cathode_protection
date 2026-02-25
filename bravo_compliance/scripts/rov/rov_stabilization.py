#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class OdomToPidNode(Node):

    def __init__(self):
        super().__init__('odom_to_pid_node')

        # Subscriber to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/auip/vehicle_interface/odometry',
            self.odom_callback,
            10
        )

        # Publisher to PID request
        self.pid_pub = self.create_publisher(
            PoseStamped,
            '/auip/pid/request',
            10
        )

        self.get_logger().info('Odom to PID node started.')
        self.msg = None

    def odom_callback(self, msg: Odometry):
        if self.msg == None:
            self.msg = msg

        pid_msg = PoseStamped()

        # Copy header (timestamp + frame_id)
        pid_msg.header = self.msg.header
        pid_msg.header.stamp = self.get_clock().now().to_msg()

        # Copy pose directly
        pid_msg.pose = self.msg.pose.pose

        self.pid_pub.publish(pid_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToPidNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

