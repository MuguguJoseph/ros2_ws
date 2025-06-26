#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from opcua import Client, ua
import time
import threading
from sensor_msgs.msg import (
    JointState,
)  # Importing JointState message type for subscription


class MinimalOpcUaClient(Node):
    def __init__(self):
        super().__init__("minimal_opcua_client")
        self.get_logger().info(
            "Starting Minimal OPC UA Client Node (Connect & Write JointState Only)..."
        )

        # --- ROS2 Parameters for Configuration ---
        self.declare_parameter("opcua_server_url", "opc.tcp://192.168.0.1:4840")

        # Node IDs for xArm7 Joint Positions (assuming 7 joints)
        # YOU MUST UPDATE THESE WITH YOUR ACTUAL NODE IDs FROM THE SERVER!
        self.joint_pos_node_ids = []
        for i in range(7):  # For 7 joints of xArm7
            node_param_name = f"opcua_joint_pos_{i+1}_node_id"
            # Example symbolic name. Adjust 'ns=X;s="YourPrefix.JointN_Pos"' as per your server config.
            default_node_id = f'ns=3;s="robot"."xArm.Joint_{i+1}Pos"'
            self.declare_parameter(node_param_name, default_node_id)
            self.joint_pos_node_ids.append(self.get_parameter(node_param_name).value)

        self.declare_parameter(
            "opcua_user", ""
        )  # Optional: Leave empty if no authentication
        self.declare_parameter(
            "opcua_password", ""
        )  # Optional: Leave empty if no authentication

        self.opcua_url = self.get_parameter("opcua_server_url").value
        self.opcua_user = self.get_parameter("opcua_user").value
        self.opcua_password = self.get_parameter("opcua_password").value

        # --- ROS2 Subscribers ---
        # Only JointState subscriber remains
        self.joint_state_subscriber = self.create_subscription(
            JointState, "joint_states", self.joint_state_callback, 10
        )
        self.get_logger().info('Subscribed to "/joint_states" topic.')

        # --- OPC UA Client Setup ---
        self.client = Client(self.opcua_url)
        if self.opcua_user and self.opcua_password:
            self.client.set_user(self.opcua_user)
            self.client.set_password(self.opcua_password)

        self.connected = False
        self.opcua_connection_lock = threading.Lock()  # For thread-safe client access

        # --- Store latest JointState data ---
        # Assuming 7 joints for xArm7, initialize with zeros
        self.latest_joint_positions = [0.0] * 7
        self.joint_states_received_first_time = (
            False  # Flag to ensure we don't send default 0s initially
        )

        # --- Start OPC UA Connection and Operations Thread ---
        self.opcua_thread = threading.Thread(target=self._opcua_worker_thread)
        self.opcua_thread.daemon = True  # Allows thread to exit when main program exits
        self.opcua_thread.start()

        self.get_logger().info(
            "Minimal OPC UA Client Node initialized. Attempting connection..."
        )

    # Removed pose_callback as it's no longer needed

    def joint_state_callback(self, msg: JointState):
        """Callback function to handle incoming JointState messages."""
        if len(msg.position) >= 7:  # Ensure we have enough data for 7 joints
            self.latest_joint_positions = list(
                msg.position[:7]
            )  # Take the first 7 positions
            self.joint_states_received_first_time = True
            self.get_logger().debug(
                f"Received JointState positions: {self.latest_joint_positions}"
            )
        else:
            self.get_logger().warn(
                f"JointState message has less than 7 positions ({len(msg.position)}). Skipping update."
            )

    def _opcua_worker_thread(self):
        """Thread to manage OPC UA connection and perform all OPC UA operations."""
        while rclpy.ok():  # Keep running as long as the ROS2 node is active
            with self.opcua_connection_lock:  # Ensure only one thread accesses client at a time
                if not self.connected:
                    try:
                        self.get_logger().info(
                            f"Attempting to connect to OPC UA server at {self.opcua_url}..."
                        )
                        self.client.connect()
                        self.get_logger().info(
                            "Successfully connected to OPC UA server."
                        )
                        self.connected = True
                        # Perform initial joint state write after connecting
                        self._write_joint_states_to_opcua()
                    except Exception as e:
                        self.connected = False
                        self.get_logger().error(
                            f"Failed to connect to OPC UA server: {e}. Retrying in 5 seconds..."
                        )
                        time.sleep(5)  # Wait before next connection attempt
                else:
                    # If connected, periodically perform joint state write and check connection
                    try:
                        self._write_joint_states_to_opcua()
                        # A simple call to check if connection is active without doing much else
                        self.client.get_endpoints()
                        self.get_logger().debug("OPC UA connection remains active.")
                    except Exception as e:
                        self.get_logger().error(
                            f"OPC UA connection lost: {e}. Attempting to reconnect..."
                        )
                        self.connected = (
                            False  # Mark as disconnected to trigger reconnection
                        )

            time.sleep(
                2
            )  # Perform operations and check connection status every 2 seconds

    # Removed _perform_boolean_operations and _write_pose_data_to_opcua as they are no longer needed.

    def _write_joint_states_to_opcua(self):
        """Writes the latest JointState positions to the OPC UA server."""
        if (
            not self.connected
        ):  # Add this check as it's now directly called from worker thread
            self.get_logger().warn(
                "Not connected to OPC UA server, skipping JointState write."
            )
            return

        if not self.joint_states_received_first_time:
            self.get_logger().debug(
                "No JointState data received yet, skipping OPC UA JointState write."
            )
            return

        try:
            log_message = "Successfully wrote Joint Positions: "
            # Ensure the number of Node IDs matches the number of joint positions we expect
            if len(self.latest_joint_positions) != len(self.joint_pos_node_ids):
                self.get_logger().error(
                    f"Mismatch: {len(self.latest_joint_positions)} joint positions vs {len(self.joint_pos_node_ids)} configured Node IDs."
                )
                return

            for i, position in enumerate(self.latest_joint_positions):
                node_id = self.joint_pos_node_ids[i]
                joint_node = self.client.get_node(node_id)

                # Joint positions are float64 in ROS, map to Float (float32) in OPC UA
                dv_joint_pos = ua.DataValue(
                    ua.Variant(float(position), ua.VariantType.Float)
                )
                joint_node.set_value(dv_joint_pos)

                log_message += f"Joint{i+1}={position:.4f} "

            self.get_logger().info(log_message.strip())

        except Exception as e:
            self.get_logger().error(
                f"Error writing JointState data to OPC UA server: {e}"
            )
            raise  # Re-raise to ensure connection handling if writing fails

    def on_shutdown(self):
        """Clean up resources on node shutdown."""
        if self.connected:
            self.get_logger().info("Disconnecting from OPC UA server...")
            with self.opcua_connection_lock:
                try:
                    self.client.disconnect()
                    self.get_logger().info("OPC UA client disconnected.")
                except Exception as client_e:
                    self.get_logger().warning(
                        f"Error disconnecting OPC UA client: {client_e}"
                    )
        self.get_logger().info("Minimal OPC UA Client Node shutting down.")


def main(args=None):
    rclpy.init(args=args)
    node = MinimalOpcUaClient()
    try:
        rclpy.spin(node)  # Keeps the node alive and processes callbacks
    except KeyboardInterrupt:
        pass  # Allows graceful exit on Ctrl+C
    finally:
        node.on_shutdown()  # Call shutdown handler
        rclpy.shutdown()  # Shuts down ROS2 client library


if __name__ == "__main__":
    main()
