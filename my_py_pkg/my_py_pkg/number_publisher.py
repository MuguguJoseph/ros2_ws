#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64


class MyIntegerPub(Node):
    def __init__(self):
        super().__init__("Integer_Publisher")
        self.declare_parameter("test321", 7)
        self.declare_parameter("publish_freq", 4.0)
        self.i = self.get_parameter("test321").value
        self.xyz = self.get_parameter("publish_freq").value
        self.publisher_ = self.create_publisher(Int64, "number", 10)
        self.timer_ = self.create_timer(1.0 / self.xyz, self.publish_number)
        self.get_logger().info("Number Publisher has been started")

    def publish_number(self):
        msg = Int64()
        msg.data = self.i
        self.publisher_.publish(msg)

    # self.get_logger().info(f"Published: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = MyIntegerPub()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
