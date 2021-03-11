#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial

from my_robot_interfaces.srv import SetLed


class BatteryNode(Node):

    def __init__(self):
        super().__init__("battery")
        self.isEmpty = False
        self.timer_ = self.create_timer(2, self.call_set_led)
        self.get_logger().info("Battery has been started")

    def call_set_led(self):
        self.isEmpty = not self.isEmpty    # simulate battery charge / discharge
        led_client = self.create_client(SetLed, "set_led")
        while not led_client.wait_for_service(1.0):
            self.get_logger().warn("waiting for server Set Led .....")

        request = SetLed.Request()
        request.led_number = 2
        request.state = self.isEmpty

        future = led_client.call_async(request)
        future.add_done_callback(partial(self.callback_set_led, wasEmpty = self.isEmpty))

    def callback_set_led(self, future, wasEmpty):
        try:
            response = future.result()
            self.get_logger().info("Response received, success: " + str(response.success))
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    # def get_current_time_seconds(self):
    #     secs, nsecs = self.get_clock().now().seconds_nanoseconds()
    #     return secs + nsecs / 1000000000.0


def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
