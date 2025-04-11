#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLed

from functools import partial


class BatteryNode(Node):
    def __init__(self):
        super().__init__("battery")
        self.battery_state = "full"
        self.last_time_battery_state_changed = self.get_current_time_seconds()
        # Assigns the current time of 0 seconds once the Node is run as this calls
        # the constructor once automatically
        self.battery_timer = self.create_timer(0.1, self.check_battery_state)
        # checks the battery state every 0.1 seconds
        self.get_logger().info("Battery Node has been started")

    def get_current_time_seconds(self):
        # this method makes the use of the timer functionality of the ROS2
        # to get the current time in seconds
        # it uses the get_clock method to get the current time in seconds
        # and nanoseconds and returns the sum of both
        # to get the current time in seconds
        secs, nsecs = self.get_clock().now().seconds_nanoseconds()

        return secs + nsecs / 1000000000

    def check_battery_state(self):
        time_now = self.get_current_time_seconds()
        # the variable time_now is assigned the current time everytime the
        # callback is called which is every 0.1 seconds

        if self.battery_state == "full":
            if time_now - self.last_time_battery_state_changed > 4.0:
                self.battery_state = "empty"
                self.get_logger().info("Battery is empty!!Charging battery...")
                self.last_time_battery_state_changed = time_now
                self.battery_status_client(3, 1)
        else:
            if time_now - self.last_time_battery_state_changed > 6.0:
                self.battery_state = "full"
                self.get_logger().info("Battery is now full again!!")
                self.last_time_battery_state_changed = time_now
                self.battery_status_client(3, 0)

    # copy and paste the client template here
    def battery_status_client(self, led_number, states):
        client_ = self.create_client(SetLed, "set_led")
        while not client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Set LED server")

        # since you've already created the client
        # now design the request
        request = SetLed.Request()
        request.led_number = led_number
        request.states = states

        future_ = client_.call_async(request)
        # we must create a  spin to wait for the server to provide us with a response
        # the future object stores the response from the server
        # we are going to use the done_callback method to check if the server has provided us with a response
        future_.add_done_callback(
            partial(self.callback_set_led_service, led_number=led_number, states=states)
        )
        # this is used to bind the arguments of the callback function

    # the callback is to sort how you are going to handle the response once the client makes
    # and gets the response from the server
    def callback_set_led_service(self, future_, led_number, states):

        # if the above statement executes this means the response is obtained
        # or the service call failed ,either of the two,so
        try:
            response = future_.result()
            # this is the response from the server
            self.get_logger().info(str(response.success))
            # this is the response from the server

        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
