#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math


class DistanceNode(Node):
    def __init__(self):
        super().__init__('distance_node')

        # Subscribers
        self.create_subscription(
            Pose, '/turtle1/pose', self.cb_turtle1, 10)
        self.create_subscription(
            Pose, '/turtle2/pose', self.cb_turtle2, 10)

        # Publishers
        self.dist_pub = self.create_publisher(
            Float32, '/distance', 10)
        self.cmd_pub1 = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10)
        self.cmd_pub2 = self.create_publisher(
            Twist, '/turtle2/cmd_vel', 10)

        self.pose1 = None
        self.pose2 = None
        self.threshold = 1.5

        self.get_logger().info("Distance node running")

    def cb_turtle1(self, msg):
        self.pose1 = msg
        self.evaluate()

    def cb_turtle2(self, msg):
        self.pose2 = msg
        self.evaluate()

    def evaluate(self):
        if self.pose1 is None or self.pose2 is None:
            return

        # Compute Euclidean distance
        dist = math.sqrt(
            (self.pose1.x - self.pose2.x) ** 2 +
            (self.pose1.y - self.pose2.y) ** 2
        )

        dist_msg = Float32()
        dist_msg.data = dist
        self.dist_pub.publish(dist_msg)

        stop = Twist()

        # Safety: distance
        if dist < self.threshold:
            self.get_logger().warn(f"Turtles too close: {dist:.2f}")
            self.cmd_pub1.publish(stop)
            self.cmd_pub2.publish(stop)

        # Safety: boundaries
        if self.out_of_bounds(self.pose1):
            self.get_logger().warn("Turtle1 near boundary")
            self.cmd_pub1.publish(stop)

        if self.out_of_bounds(self.pose2):
            self.get_logger().warn("Turtle2 near boundary")
            self.cmd_pub2.publish(stop)

    def out_of_bounds(self, pose):
        return (
            pose.x < 1.0 or pose.x > 10.0 or
            pose.y < 1.0 or pose.y > 10.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = DistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
