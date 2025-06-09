#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String


class MyRobotsNewsStation(Node):
    def __init__(self):
        super().__init__("robots_news_station")
        self.declare_parameter("robot_name", "Joe the Robot")
        self.robot_name = self.get_parameter("robot_name").value
        self.publisher_ = self.create_publisher(String, "dhjwndnm", 10)
        self.get_logger().info("Robots News Station has been started")
        self.timer_ = self.create_timer(0.5, self.publishing)

    def publishing(self):
        msg = String()
        msg._data = (
            "Hello World, " + self.robot_name + " ,from the Robot's News Station"
        )
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MyRobotsNewsStation()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
