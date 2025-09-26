#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from opcua import Client, ua
import time
import threading
from sensor_msgs.msg import JointState


class MinimalOpcUaClient(Node):
    def __init__(self):
        super().__init__("minimal_opcua_client")
        self.get_logger().info("Starting  OPC UA Client Node...")

        # --- ROS2 Parameters for Configuration ---
        self.declare_parameter(
            "opcua_server_url", "opc.tcp://mugugu:53530/OPCUA/SimulationServer"
        )
        # opc.tcp://mugugu:53530/OPCUA/SimulationServer
        # opc.tcp://192.168.0.1:4840
        self.declare_parameter("opcua_user", "")
        self.declare_parameter("opcua_password", "")

        # Node IDs for boolean variables
        self.declare_parameter("activate_plc_node_id", 's="robot"."activate_plc"')
        # s="robot"."activate_plc"
        # ns=3;s="activate_plc"
        self.declare_parameter(
            "activate_robot_node_id",
            'ns=3;s="activate_robot"',
            # s="robot"."activate_robot"
            # ns=3;s="activate_robot"
        )
        self.declare_parameter("gripper_arm_node_id", 'ns=3;s="robot"."gripper_arm"')
        # ns=3;s="robot"."gripper_arm"
        #'ns=3;s="gripper_arm"

        # Node IDs for xArm7 Joint Positions
        self.joint_pos_node_ids = []
        for i in range(7):
            node_param_name = f"opcua_joint_pos_{i+1}_node_id"
            default_node_id = f'ns=3;s="xArm.Joint{i+1}_Pos"'
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
        self.latest_joint_positions = [0.0] * 7
        self.joint_states_received_first_time = False

        # --- Start OPC UA Connection Thread ---
        self.opcua_thread = threading.Thread(target=self._opcua_worker_thread)
        self.opcua_thread.daemon = True
        self.opcua_thread.start()

    def joint_state_callback(self, msg: JointState):
        """Handle incoming JointState messages."""
        if len(msg.position) >= 7:
            self.latest_joint_positions = list(msg.position[:7])
            self.joint_states_received_first_time = True

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
                        self._write_gripper_arm()

                        # Simple call to check connection
                        self.client.get_endpoints()
                    except Exception as e:
                        self.get_logger().error(f"OPC UA error: {e}")
                        self.connected = False
            time.sleep(2)

    def _write_joint_states_to_opcua(self):
        """Write joint states to OPC UA server."""
        if not self.connected or not self.joint_states_received_first_time:
            return

        try:
            log_message = "Successfully wrote Joint Positions: "
            for i, position in enumerate(self.latest_joint_positions):
                node_id = self.joint_pos_node_ids[i]
                joint_node = self.client.get_node(node_id)
                dv_joint_pos = ua.DataValue(
                    ua.Variant(float(position), ua.VariantType.Float)
                )
                joint_node.set_value(dv_joint_pos)
                log_message += f"Joint{i+1}={position:.4f} "

            self.get_logger().info(log_message.strip())
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
            self.get_logger().info(
                f"Successfully wrote boolean '{self.activate_plc_state}' to '{self.activate_plc_node}'"
            )
            self.get_logger().debug(f"Wrote activate_plc: {self.activate_plc_state}")
        except Exception as e:
            self.get_logger().error(f"activate_plc write error: {e}")

    def _read_activate_robot(self):
        """Read activate_robot boolean from OPC UA server."""
        try:
            self.activate_robot_state = self.activate_robot_node.get_value()
            self.get_logger().debug(f"Read activate_robot: {self.activate_robot_state}")

            # Here you would add logic to act on the activate_robot_state
            if self.activate_robot_state:
                self.get_logger().info("Robot activation signal received!")
        except Exception as e:
            self.get_logger().error(f"activate_robot read error: {e}")

    def _write_gripper_arm(self):
        """Read gripper_arm boolean from OPC UA server."""
        try:
            dv = ua.DataValue(
                ua.Variant(self.gripper_arm_state, ua.VariantType.Boolean)
            )
            self.get_logger().info(
                f"Successfully wrote boolean '{self.gripper_arm_state}' to '{self.gripper_arm_node}'"
            )
            self.get_logger().debug(
                f"Wrote gripper_arm_state: {self.gripper_arm_state}"
            )
        except Exception as e:
            self.get_logger().error(f"gripper_arm_write write error: {e}")

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
