#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool


class pubnsub(Node):
    def __init__(self):
        super().__init__("Publisher_Subscriber")
        self.subscriber_ = self.create_subscription(
            Int64, "number", self.Subcallback_, 10
        )
        self.publisher2_ = self.create_publisher(Int64, "number_count", 10)
        self.get_logger().info("Number Publisher2 has been started")
        self.counter_ = 0
        self.server_ = self.create_service(
            SetBool, "reset_counter", self.callback_reset_counter_server
        )

    def callback_reset_counter_server(self, request, response):

        if request.data:
            self.counter_ = 0
            response.success = True
            response.message = "Counter reset to 0"
        else:
            response.success = False
            response.message = "Counter reset failed"
        return response

    def Subcallback_(self, msg):
        xyz = Int64()
        self.get_logger().info(f"Number received: {msg.data}")
        self.counter_ += msg.data
        xyz.data = self.counter_
        self.publisher2_.publish(xyz)
        self.get_logger().info(f"Published: {xyz.data}")


def main(args=None):
    rclpy.init(args=args)
    node = pubnsub()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
