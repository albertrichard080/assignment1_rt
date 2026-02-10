#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class UINode(Node):
    def __init__(self):
        super().__init__('ui_node')

        self.pub_turtle1 = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10)
        self.pub_turtle2 = self.create_publisher(
            Twist, '/turtle2/cmd_vel', 10)

        self.get_logger().info("UI Node ready")

    def send_command(self, turtle, lin_vel, ang_vel):
        cmd = Twist()
        cmd.linear.x = lin_vel
        cmd.angular.z = ang_vel

        if turtle == "turtle1":
            publisher = self.pub_turtle1
        elif turtle == "turtle2":
            publisher = self.pub_turtle2
        else:
            self.get_logger().error("Invalid turtle name")
            return

        # Publish ONCE
        publisher.publish(cmd)
        self.get_logger().info(
            f"Command sent to {turtle} (lin={lin_vel}, ang={ang_vel})")

        # Move for 1 second
        time.sleep(1.0)

        # Stop
        publisher.publish(Twist())
        self.get_logger().info(f"{turtle} stopped")


def main(args=None):
    rclpy.init(args=args)
    node = UINode()

    try:
        while rclpy.ok():
            turtle = input(
                "Choose turtle (turtle1 / turtle2): ").strip()

            if turtle not in ["turtle1", "turtle2"]:
                print("Invalid turtle name. Try again.")
                continue

            try:
                lin = float(input("Linear velocity (x): "))
                ang = float(input("Angular velocity (z): "))
            except ValueError:
                print("Please enter numeric values for velocities.")
                continue

            node.send_command(turtle, lin, ang)

    except KeyboardInterrupt:
        print("\nUI Node terminated by user")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
