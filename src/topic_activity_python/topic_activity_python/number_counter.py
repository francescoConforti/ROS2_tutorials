#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool


class NumberCounterNode(Node):

    def __init__(self):
        super().__init__("number_counter")
        self.count = 0
        self.subscriber_ = self.create_subscription(Int64, "number",
                                                    self.callback_number, 10)
        self.publisher_ = self.create_publisher(Int64, "number_count", 10)
        self.reset_counter_service_ = self.create_service(
            SetBool, "reset_counter", self.callback_reset_counter)
        self.get_logger().info("Number Counter has been started.")

    def callback_number(self, msg):
        self.count += msg.data
        self.get_logger().info("Received " + str(msg.data))
        self.publish_count()

    def publish_count(self):
        msg = Int64()
        msg.data = self.count
        self.publisher_.publish(msg)

    def callback_reset_counter(self, request, response):
        self.count = 0
        response.success = True
        response.message = "counter reset"
        self.get_logger().info("!!! counter reset. !!!")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
