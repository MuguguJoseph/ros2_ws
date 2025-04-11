#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import LedStateArray
from my_robot_interfaces.srv import SetLed


class LedPanelNode(Node):
    def __init__(self):
        super().__init__("led_panel")
        self.led_states = [0, 0, 0]
        self.led_states_publisher_ = self.create_publisher(
            LedStateArray, "led_states", 10
        )
        self.timer_ = self.create_timer(4, self.publish_led_states)
        self.set_led_service = self.create_service(
            SetLed, "set_led", self.callback_set_led
        )
        self.get_logger().info("Led Panel Node has been started")

    def callback_set_led(self, request, response):
        led_number = request.led_number
        states = request.states

        if led_number > len(self.led_states) or led_number <= 0:
            response.success = False
            self.get_logger().error("Invalid led number")
            return response

        if states not in [0, 1]:
            response.success = False
            self.get_logger().error("Invalid led state")
            return response

        self.led_states[led_number - 1] = states
        response.success = True
        self.publish_led_states()
        return response

    def publish_led_states(self):
        msg = LedStateArray()
        msg.led_states = self.led_states
        self.led_states_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LedPanelNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
