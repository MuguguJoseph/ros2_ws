#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math


class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.pose_ = None
        self.declare_parameter("target_x", 5.0)
        self.declare_parameter("target_y", 2.0)
        self.target_x = self.get_parameter("target_x").value
        self.target_y = self.get_parameter("target_y").value
        self.pose_subscriber_ = self.create_subscription(
            Pose, "turtle1/pose", self.callback_turtle_pose, 10
        )
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        # we want to create a loop and for that we need a control loop timer
        self.control_loop_timer = self.create_timer(0.01, self.control_loop)

    def callback_turtle_pose(self, msg):
        self.pose_ = msg

    def control_loop(self):
        if self.pose_ is None:
            return

        dist_x = self.target_x - self.pose_.x
        dist_y = self.target_y - self.pose_.y
        distance = math.sqrt(dist_x * dist_x + dist_y * dist_y)

        msg = Twist()
        if distance > 0.5:
            # position
            msg.linear.x = 2 * distance
            # orientation
            goal_theta = math.atan2(dist_y, dist_x)  # gets the destination angle
            diff = goal_theta - self.pose_.theta
            if diff > math.pi:
                diff -= 2 * math.pi
            elif diff < -math.pi:
                diff += 2 * math.pi
            msg.angular.z = 6 * diff

        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        # publish the message

        self.cmd_vel_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
