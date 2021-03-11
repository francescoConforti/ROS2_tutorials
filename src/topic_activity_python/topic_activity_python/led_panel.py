#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from my_robot_interfaces.msg import LedStates
from my_robot_interfaces.srv import SetLed


class LedPanelNode(Node):

    def __init__(self):
        super().__init__("led_panel")
        self.declare_parameter("led_vals", [False, False, False])
        self.led_on = self.get_parameter("led_vals").value
        self.led_states_publisher = self.create_publisher(LedStates, "led_states", 10)
        self.server_ = self.create_service(
            SetLed, "set_led", self.callback_set_led)
        self.timer_ = self.create_timer(0.5, self.publish_led_states)
        self.get_logger().info("Led Panel has been started")

    def publish_led_states(self):
        msg = LedStates()
        for i in range(3):
            msg.is_on[i] = self.led_on[i]
        self.led_states_publisher.publish(msg)

    def callback_set_led(self, request, response):
        response.success = True
        if request.led_number < 0 or request.led_number < 0:
            return False
        self.led_on[request.led_number] = request.state
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LedPanelNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
