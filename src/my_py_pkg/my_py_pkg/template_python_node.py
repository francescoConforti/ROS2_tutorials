#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class MyCustomNode(Node):  # CHANGE NAME

    def __init__(self):
        super().__init__("node_name")  # CHANGE NAME


def main(args=None):
    rclpy.init(args=args)
    node = MyCustomNode()  # CHANGE NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
