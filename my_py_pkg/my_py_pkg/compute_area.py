#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import ComputeRectangleArea


class ComputeArea(Node):
    def __init__(self):
        super().__init__("compute_area")
        self.get_logger().info("Compute Area Node has been started")
        self.server_ = self.create_service(
            ComputeRectangleArea, "compute_area", self.callback_compute_area
        )

    def callback_compute_area(self, request, response):
        self.get_logger().info("Received request to compute area")
        response.area = request.length * request.width
        self.get_logger().info(f"Computed area: {response.area}")
        self.get_logger().info(
            str(request.length) + "x" + str(request.width) + "=" + str(response.area)
        )  # This line will cause an error because request.length and request.width are floats, not strings
        return response


def main(args=None):
    rclpy.init(args=args)  # change name
    node = ComputeArea()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
