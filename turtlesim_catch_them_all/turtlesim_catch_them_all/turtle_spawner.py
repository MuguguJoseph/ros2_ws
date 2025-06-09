#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from functools import partial
import random
import math


class TurtleSpawner(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.spawn_turtle_timer_ = self.create_timer(2.0, self.spawn_new_turtle)
        self.turtle_name_prefix = "turtle"
        self.turtle_counter = 0

    def spawn_new_turtle(self):
        self.turtle_counter += 1
        name = self.turtle_name_prefix + str(self.turtle_counter)
        x = random.uniform(0.0, 11.0)
        y = random.uniform(0.0, 11.0)
        theta = random.uniform(0.0, 2 * math.pi)
        self.call_Spawn_server(name, x, y, theta)
        self.get_logger().info("Spawning turtle: " + name)

    def call_Spawn_server(self, turtle_name, x, y, theta):
        client_ = self.create_client(Spawn, "spawn")
        while not client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Spawn server")

        # since you've already created the client
        # now design the request
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = turtle_name

        future_ = client_.call_async(request)
        # we must create a  spin to wait for the server to provide us with a response
        # the future object stores the response from the server
        # we are going to use the done_callback method to check if the server has provided us with a response
        future_.add_done_callback(
            partial(
                self.callback_call_spawn,
                turtle_name=turtle_name,
                x=x,
                y=y,
                theta=theta,
            )
        )

    def callback_call_spawn(self, future_, turtle_name, x, y, theta):
        # if the above statement executes this means the response is obtained
        # or the service call failed ,either of the two,so
        try:
            response = future_.result()
            self.get_logger().info("Turtle" + response.name + " is now alive")

        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
