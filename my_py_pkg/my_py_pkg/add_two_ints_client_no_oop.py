#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts


def main(args=None):
    rclpy.init(args=args)
    node = Node("add_two_ints_client")
    client_ = node.create_client(AddTwoInts, "addtwoints")
    while not client_.wait_for_service(1.0):
        node.get_logger().warn("Waiting for add_two_ints_server")

    # since you've already created the client
    # now design the request
    request = AddTwoInts.Request()
    request.a = 5
    request.b = 9

    future_ = client_.call_async(request)
    rclpy.spin_until_future_complete(node, future_)
    # if the above statement executes this means the response is obtained
    # or the service call failed ,either of the two,so
    try:
        response = future_.result()
        node.get_logger().info(
            str(request.a) + " + " + str(request.b) + " = " + str(response.sum)
        )

    except Exception as e:
        node.get_logger().error("Service call failed %r" % (e,))


if __name__ == "__main__":
    main()
