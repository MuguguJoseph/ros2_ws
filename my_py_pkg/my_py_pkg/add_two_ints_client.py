#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts
from functools import partial

# this is used to bind the arguments of the callback function
# this is used to enable us to add more arguments to the callback function


class AddTwoIntsNode(Node):
    def __init__(self):
        super().__init__("add_two_ints_client")
        # this is the name of the node
        self.get_logger().info("AddTwoIntsClient has been started")
        self.call_add_two_ints_server(5, 9)
        self.call_add_two_ints_server(10, 20)
        self.call_add_two_ints_server(100, 200)
        # this is the method that will be called when the server is called

    # Design a method that connects with the server node
    # this is achieved by specifying the service type and the service/server name

    def call_add_two_ints_server(self, a, b):
        client_ = self.create_client(AddTwoInts, "addtwoints")
        while not client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for add_two_ints_server")

        # since you've already created the client
        # now design the request
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future_ = client_.call_async(request)
        # we must create a  spin to wait for the server to provide us with a response
        # the future object stores the response from the server
        # we are going to use the done_callback method to check if the server has provided us with a response
        future_.add_done_callback(partial(self.callback_add_two_ints, a=a, b=b))

    def callback_add_two_ints(self, future_, a, b):
        # if the above statement executes this means the response is obtained
        # or the service call failed ,either of the two,so
        try:
            response = future_.result()
            self.get_logger().info(str(a) + " + " + str(b) + " = " + str(response.sum))

        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
