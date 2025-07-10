#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from moveit_py.planning import MoveGroup  # Simplified API for MoveIt


class XArmMotionPlanner(Node):
    def __init__(self):
        super().__init__("xarm_motion_planner")
        self.get_logger().info("XArm Motion Planner Node started.")

        # --- Configure your MoveIt planning group and robot description ---
        # Replace 'xarm7' with the actual name of your planning group in MoveIt config
        # This is usually the name of your robot, e.g., 'xarm7' or 'arm'
        self.planning_group_name = "xarm7"
        self.robot_description_topic = "robot_description"  # Standard topic for URDF

        # Initialize MoveGroup
        # This connects to the MoveIt planning services
        self.move_group = MoveGroup(self.planning_group_name, node=self)
        self.get_logger().info(f"MoveGroup '{self.planning_group_name}' initialized.")

        # --- Define known joint angles for your poses ---
        # IMPORTANT: The order of these joint angles MUST match the order expected by MoveIt
        # for your planning group. You can verify this in your MoveIt config or by
        # inspecting the 'joint_states' topic published by your robot.

        # Example joint names for xArm7 (verify with your URDF/MoveIt config)
        self.joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "joint7",
        ]

        # Home position joint angles (example - replace with your actual values)
        # Usually all zeros or a safe retracted pose.
        self.home_joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Pick position joint angles (example - replace with your actual values for conveyor 1)
        self.pick_joint_angles = [
            0.5,
            -0.2,
            0.8,
            -1.0,
            0.0,
            1.2,
            0.5,
        ]

        # Place position joint angles (example - replace with your actual values for conveyor 2)
        self.place_joint_angles = [
            -0.5,
            0.3,
            -0.7,
            1.1,
            0.0,
            -1.3,
            -0.5,
        ]

        # --- Give MoveIt some time to initialize ---
        # In a real scenario, you might want to wait for MoveIt services to be available
        self.get_logger().info("Waiting for MoveIt to be ready...")
        time.sleep(5)

        # --- Start the motion sequence ---
        self.motion_sequence_timer = self.create_timer(1.0, self.start_motion_sequence)
        self.motion_sequence_started = False

    def start_motion_sequence(self):
        """Timer callback to start the motion sequence once."""
        if not self.motion_sequence_started:
            self.motion_sequence_started = True
            self.motion_sequence_timer.cancel()  # Cancel the timer after first execution
            self.get_logger().info("Starting robot motion sequence...")
            self.execute_motion_sequence()

    def execute_motion_sequence(self):
        """Executes the predefined sequence of robot movements."""

        # 1. Move to Home Position
        self.get_logger().info("Moving to Home position...")
        if not self.move_to_joint_target(self.home_joint_angles, "Home"):
            self.get_logger().error(
                "Failed to move to Home position. Aborting sequence."
            )
            return
        time.sleep(2)  # Pause for stability

        # 2. Move to Pick Position (Conveyor 1)
        self.get_logger().info("Moving to Pick position (Conveyor 1)...")
        if not self.move_to_joint_target(self.pick_joint_angles, "Pick"):
            self.get_logger().error(
                "Failed to move to Pick position. Aborting sequence."
            )
            return
        self.get_logger().info(
            "Reached Pick position. (Simulate gripper close here if applicable)"
        )
        # Add your gripper closing logic here if you have one
        time.sleep(2)  # Pause for gripper action

        # 3. Move to Place Position (Conveyor 2)
        self.get_logger().info("Moving to Place position (Conveyor 2)...")
        if not self.move_to_joint_target(self.place_joint_angles, "Place"):
            self.get_logger().error(
                "Failed to move to Place position. Aborting sequence."
            )
            return
        self.get_logger().info(
            "Reached Place position. (Simulate gripper open here if applicable)"
        )
        # Add your gripper opening logic here if you have one
        time.sleep(2)  # Pause for gripper action

        # 4. Return to Home Position
        self.get_logger().info("Returning to Home position...")
        if not self.move_to_joint_target(self.home_joint_angles, "Home"):
            self.get_logger().error("Failed to return to Home position.")
            return
        self.get_logger().info("Motion sequence completed successfully!")

    def move_to_joint_target(self, target_joint_angles: list, target_name: str) -> bool:
        """
        Plans and executes a movement to the specified joint angles.
        :param target_joint_angles: A list of joint angles for the target pose.
        :param target_name: A descriptive name for the target pose (for logging).
        :return: True if the movement was successful, False otherwise.
        """
        self.get_logger().info(f"Attempting to plan to {target_name}...")

        # Set the joint target
        self.move_group.set_joint_value_target(target_joint_angles)

        # Plan the motion
        plan_success, trajectory = self.move_group.plan()

        if plan_success:
            self.get_logger().info(
                f"Planning to {target_name} successful. Executing..."
            )
            # Execute the planned trajectory
            execute_success = self.move_group.execute(trajectory)
            if execute_success:
                self.get_logger().info(f"Execution to {target_name} successful.")
                return True
            else:
                self.get_logger().error(f"Execution to {target_name} failed.")
                return False
        else:
            self.get_logger().error(
                f"Planning to {target_name} failed. No valid trajectory found."
            )
            return False


def main(args=None):
    rclpy.init(args=args)
    node = XArmMotionPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down XArm Motion Planner node.")
        rclpy.shutdown()


if __name__ == "__main__":
    main()
