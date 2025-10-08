#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from opcua import Client, ua
import time
import threading
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool  # Use standard Bool message type for activation signal


class MinimalOpcUaClient(Node):
    def __init__(self):
        super().__init__("minimal_opcua_client")
        self.get_logger().info("Starting OPC UA Client Node...")

        # --- ROS2 Parameters for Configuration ---
        self.declare_parameter(
            "opcua_server_url", "opc.tcp://mugugu:53530/OPCUA/SimulationServer"
        )
        # opc.tcp://mugugu:53530/OPCUA/SimulationServer
        # opc.tcp://192.168.0.1:4840
        self.declare_parameter("opcua_user", "")
        self.declare_parameter("opcua_password", "")

        # Node IDs for boolean variables
        self.declare_parameter("activate_plc_node_id", 'ns=3;s="activate_plc"')
        # s="robot"."activate_plc"
        # ns=3;s="activate_plc"
        self.declare_parameter(
            "activate_robot_node_id",
            'ns=3;s="activate_robot"',
        )
        # s="robot"."activate_robot"
        # ns=3;s="activate_robot"
        self.declare_parameter("gripper_arm_node_id", 'ns=3;s="gripper_arm"')
        # ns=3;s="robot"."gripper_arm"
        # ns=3;s="gripper_arm"

        # Node IDs for xArm7 Joint Positions
        self.joint_pos_node_ids = []
        for i in range(7):
            node_param_name = f"opcua_joint_pos_{i+1}_node_id"
            default_node_id = f'ns=3;s= "xArm.Joint{i+1}_Pos"'
            # s="robot"."xArm.Joint_{i+1}Pos"
            # s="robot"."xArm.Joint{i+1}_Pos"
            # s= "xArm.Joint{i+1}_Pos"
            self.declare_parameter(node_param_name, default_node_id)
            self.joint_pos_node_ids.append(self.get_parameter(node_param_name).value)

        self.opcua_url = self.get_parameter("opcua_server_url").value
        self.opcua_user = self.get_parameter("opcua_user").value
        self.opcua_password = self.get_parameter("opcua_password").value

        # Boolean variable nodes
        self.activate_plc_node_id = self.get_parameter("activate_plc_node_id").value
        self.activate_robot_node_id = self.get_parameter("activate_robot_node_id").value
        self.gripper_arm_node_id = self.get_parameter("gripper_arm_node_id").value

        # Current boolean states
        self.activate_plc_state = True
        self.activate_robot_state = False
        self.gripper_arm_state = False

        # --- NEW: ROS2 Publisher for activate_robot_state ---
        self.activate_robot_pub = self.create_publisher(
            Bool, "activate_robot_signal", 10
        )
        self.get_logger().info("Created publisher for activate_robot_signal topic")

        # --- ROS2 Subscribers ---
        self.joint_state_subscriber = self.create_subscription(
            JointState, "joint_states", self.joint_state_callback, 10
        )

        # --- OPC UA Client Setup ---
        self.client = Client(self.opcua_url)
        if self.opcua_user and self.opcua_password:
            self.client.set_user(self.opcua_user)
            self.client.set_password(self.opcua_password)

        self.connected = False
        self.opcua_connection_lock = threading.Lock()
        self.latest_joint_positions = [0.0] * 7  # Store in radians
        self.latest_drive_joint = None  # NEW: store drive_joint value
        self.joint_states_received_first_time = False

        # Track previous state to avoid spamming the topic
        self.previous_activate_robot_state = None

        # --- Start OPC UA Connection Thread ---
        self.opcua_thread = threading.Thread(target=self._opcua_worker_thread)
        self.opcua_thread.daemon = True
        self.opcua_thread.start()

    def joint_state_callback(self, msg: JointState):
        """Handle incoming JointState messages (in radians)."""
        if len(msg.position) >= 7:
            self.latest_joint_positions = list(msg.position[:7])
            self.joint_states_received_first_time = True
            self.get_logger().debug(
                f"Received joint positions (radians): {self.latest_joint_positions}"
            )

        # Extract drive_joint if available (it's at index 7, the 8th element)
        if len(msg.position) >= 8:
            self.latest_drive_joint = msg.position[7]  # FIXED: Changed from [8] to [7]
            self.get_logger().debug(
                f"Successfully received drive_joint value: {self.latest_drive_joint:.6f}"
            )
        else:
            self.get_logger().debug(
                f"drive_joint not available - message has only {len(msg.position)} positions"
            )

    def _radians_to_degrees(self, radians):
        """Convert radians to degrees."""
        return math.degrees(radians)

    def _opcua_worker_thread(self):
        """Thread to manage OPC UA connection and operations."""
        while rclpy.ok():
            with self.opcua_connection_lock:
                if not self.connected:
                    try:
                        self.client.connect()
                        self.connected = True
                        self.get_logger().info("Connected to OPC UA server.")

                        # Initialize boolean nodes
                        self.activate_plc_node = self.client.get_node(
                            self.activate_plc_node_id
                        )
                        self.activate_robot_node = self.client.get_node(
                            self.activate_robot_node_id
                        )
                        self.gripper_arm_node = self.client.get_node(
                            self.gripper_arm_node_id
                        )

                    except Exception as e:
                        self.connected = False
                        self.get_logger().error(f"Connection failed: {e}. Retrying...")
                        time.sleep(5)
                else:
                    try:
                        # Perform all OPC UA operations
                        self._write_joint_states_to_opcua()
                        self._write_activate_plc()
                        self._read_activate_robot()
                        self._write_gripper_arm()  # now writes float drive_joint value

                        # Simple call to check connection
                        self.client.get_endpoints()
                    except Exception as e:
                        self.get_logger().error(f"OPC UA error: {e}")
                        self.connected = False
            time.sleep(2)

    def _write_joint_states_to_opcua(self):
        """Write joint states to OPC UA server (converted from radians to degrees)."""
        if not self.connected or not self.joint_states_received_first_time:
            return

        try:
            log_message_rad = "Joint Positions (radians): "
            log_message_deg = "Joint Positions (degrees): "

            for i, position_rad in enumerate(self.latest_joint_positions):
                # Convert radians to degrees
                position_deg = self._radians_to_degrees(position_rad)

                # Write degrees to OPC UA server
                node_id = self.joint_pos_node_ids[i]
                joint_node = self.client.get_node(node_id)
                dv_joint_pos = ua.DataValue(
                    ua.Variant(float(position_deg), ua.VariantType.Float)
                )
                joint_node.set_value(dv_joint_pos)

                log_message_rad += f"J{i+1}={position_rad:.4f} "
                log_message_deg += f"J{i+1}={position_deg:.4f}° "

            self.get_logger().info(log_message_rad.strip())
            self.get_logger().info(log_message_deg.strip() + " [Written to OPC UA]")

        except Exception as e:
            self.get_logger().error(f"JointState write error: {e}")
            raise

    def _write_activate_plc(self):
        """Write activate_plc boolean to OPC UA server."""
        try:
            dv = ua.DataValue(
                ua.Variant(self.activate_plc_state, ua.VariantType.Boolean)
            )
            self.activate_plc_node.set_value(dv)
            self.get_logger().debug(f"Wrote activate_plc: {self.activate_plc_state}")
        except Exception as e:
            self.get_logger().error(f"activate_plc write error: {e}")

    def _read_activate_robot(self):
        """Read activate_robot boolean from OPC UA server and publish to ROS2 topic."""
        try:
            new_state = self.activate_robot_node.get_value()

            # Only update and publish if state changed
            if new_state != self.activate_robot_state:
                self.activate_robot_state = new_state
                self.get_logger().debug(
                    f"Read activate_robot: {self.activate_robot_state}"
                )

                # Publish to ROS2 topic
                self._publish_activate_robot_state()

                # Log state changes for debugging
                if self.activate_robot_state:
                    self.get_logger().info(
                        "Robot activation signal received! Publishing to activate_robot_signal topic."
                    )
                else:
                    self.get_logger().info("Robot deactivation signal received!")

        except Exception as e:
            self.get_logger().error(f"activate_robot read error: {e}")

    def _publish_activate_robot_state(self):
        """Publish the activate_robot_state to ROS2 topic."""
        msg = Bool()
        msg.data = self.activate_robot_state
        # If the state has changed, publish it
        self.activate_robot_pub.publish(msg)
        self.get_logger().debug(
            f"Published activate_robot_state: {self.activate_robot_state} to activate_robot_signal topic"
        )

    def _write_gripper_arm(self):
        """Write drive_joint value to OPC UA server (float)."""
        if self.latest_drive_joint is None:
            self.get_logger().debug("No drive_joint value yet, skipping OPC UA write")
            return

        try:
            dv = ua.DataValue(
                ua.Variant(float(self.latest_drive_joint), ua.VariantType.Float)
            )
            self.gripper_arm_node.set_value(dv)
            self.get_logger().info(
                f"Successfully wrote drive_joint={self.latest_drive_joint:.4f} "
                f"to OPC UA node {self.gripper_arm_node_id}"
            )
        except Exception as e:
            self.get_logger().error(f"drive_joint write error: {e}")

    def set_activate_plc(self, state: bool):
        """Public method to set the activate_plc state."""
        with self.opcua_connection_lock:
            self.activate_plc_state = state
            if self.connected:
                self._write_activate_plc()

    def get_activate_robot(self) -> bool:
        """Public method to get the activate_robot state."""
        return self.activate_robot_state

    def get_gripper_arm(self) -> bool:
        """Public method to get the gripper_arm state."""
        return self.gripper_arm_state

    def on_shutdown(self):
        """Clean up resources on node shutdown."""
        if self.connected:
            self.get_logger().info("Disconnecting from OPC UA server...")
            try:
                self.client.disconnect()
            except Exception as e:
                self.get_logger().warning(f"Error disconnecting: {e}")
        self.get_logger().info("Node shutting down.")


def main(args=None):
    rclpy.init(args=args)
    node = MinimalOpcUaClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
