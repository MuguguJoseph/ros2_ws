#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from opcua import Client, ua
import time
import threading
from turtlesim.msg import Pose  # Importing Pose message type for subscription


class MinimalOpcUaClient(Node):
    def __init__(self):
        super().__init__("minimal_opcua_client")
        self.get_logger().info(
            "Starting Minimal OPC UA Client Node (Connect, Read/Write Boolean, Write Pose)..."
        )

        # --- ROS2 Parameters for Configuration ---
        self.declare_parameter(
            "opcua_server_url", "opc.tcp://mugugu:53530/OPCUA/SimulationServer"
        )
        self.declare_parameter(
            "opcua_boolean_write_node_id", "ns=3;i=4"
        )  # Node ID for the boolean value to WRITE
        self.declare_parameter(
            "opcua_boolean_read_node_id", "ns=3;i=5"
        )  # Node ID for the boolean value to READ
        self.declare_parameter(
            "opcua_pose_x_node_id", 'ns=3;s="TurtlePose.x"'
        )  # NEW: Node ID for Pose X
        self.declare_parameter(
            "opcua_pose_y_node_id", 'ns=3;s="TurtlePose.y"'
        )  # NEW: Node ID for Pose Y
        self.declare_parameter(
            "opcua_pose_theta_node_id", 'ns=3;s="TurtlePose.theta"'
        )  # NEW: Node ID for Pose Theta
        self.declare_parameter(
            "opcua_user", ""
        )  # Optional: Leave empty if no authentication
        self.declare_parameter(
            "opcua_password", ""
        )  # Optional: Leave empty if no authentication

        self.opcua_url = self.get_parameter("opcua_server_url").value
        self.opcua_boolean_write_node_id = self.get_parameter(
            "opcua_boolean_write_node_id"
        ).value
        self.opcua_boolean_read_node_id = self.get_parameter(
            "opcua_boolean_read_node_id"
        ).value
        self.opcua_pose_x_node_id = self.get_parameter(
            "opcua_pose_x_node_id"
        ).value  # NEW
        self.opcua_pose_y_node_id = self.get_parameter(
            "opcua_pose_y_node_id"
        ).value  # NEW
        self.opcua_pose_theta_node_id = self.get_parameter(
            "opcua_pose_theta_node_id"
        ).value  # NEW
        self.opcua_user = self.get_parameter("opcua_user").value
        self.opcua_password = self.get_parameter("opcua_password").value

        # --- ROS2 Subscribers ---
        self.pose_subscriber = self.create_subscription(
            Pose, "turtle1/pose", self.pose_callback, 10
        )
        self.get_logger().info('Subscribed to "turtle1/pose" topic.')

        # --- OPC UA Client Setup ---
        self.client = Client(self.opcua_url)
        if self.opcua_user and self.opcua_password:
            self.client.set_user(self.opcua_user)
            self.client.set_password(self.opcua_password)

        self.connected = False
        self.opcua_connection_lock = threading.Lock()  # For thread-safe client access

        self.boolean_toggle_state = False  # Initial state of the boolean to write

        # --- Store latest Pose data ---
        self.latest_pose_x = 0.0
        self.latest_pose_y = 0.0
        self.latest_pose_theta = 0.0
        self.pose_received_first_time = (
            False  # Flag to ensure we don't send default 0s initially
        )

        # --- Start OPC UA Connection and Operations Thread ---
        self.opcua_thread = threading.Thread(target=self._opcua_worker_thread)
        self.opcua_thread.daemon = True  # Allows thread to exit when main program exits
        self.opcua_thread.start()

        self.get_logger().info(
            "Minimal OPC UA Client Node initialized. Attempting connection..."
        )

    def pose_callback(self, msg: Pose):
        """Callback function to handle incoming Pose messages."""
        self.latest_pose_x = msg.x
        self.latest_pose_y = msg.y
        self.latest_pose_theta = msg.theta
        self.pose_received_first_time = (
            True  # Mark that we've received at least one pose
        )
        self.get_logger().debug(
            f"Received Pose: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}"
        )
        # The writing of this data to OPC UA happens periodically in _opcua_worker_thread

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
                        # Perform initial operations after connecting
                        self._perform_opcua_operations()
                    except Exception as e:
                        self.connected = False
                        self.get_logger().error(
                            f"Failed to connect to OPC UA server: {e}. Retrying in 5 seconds..."
                        )
                        time.sleep(5)  # Wait before next connection attempt
                else:
                    # If connected, periodically perform operations and check connection
                    try:
                        self._perform_opcua_operations()
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

    def _perform_opcua_operations(self):
        """Performs all OPC UA write (boolean, pose) and read (boolean) operations."""
        if not self.connected:
            self.get_logger().warn(
                "Not connected to OPC UA server, skipping all OPC UA operations."
            )
            return

        try:
            # --- Boolean Write Operation ---
            write_boolean_node = self.client.get_node(self.opcua_boolean_write_node_id)
            self.boolean_toggle_state = not self.boolean_toggle_state
            dv_write_bool = ua.DataValue(
                ua.Variant(self.boolean_toggle_state, ua.VariantType.Boolean)
            )
            write_boolean_node.set_value(dv_write_bool)
            self.get_logger().info(
                f"Successfully wrote boolean '{self.boolean_toggle_state}' to '{self.opcua_boolean_write_node_id}'"
            )

            # --- Boolean Read Operation ---
            read_boolean_node = self.client.get_node(self.opcua_boolean_read_node_id)
            read_boolean_value = read_boolean_node.get_value()
            self.get_logger().info(
                f"Successfully read boolean '{read_boolean_value}' from '{self.opcua_boolean_read_node_id}'"
            )

            # --- Pose Write Operation (NEW) ---
            self._write_pose_data_to_opcua()

        except Exception as e:
            self.get_logger().error(f"Error during OPC UA operations: {e}")
            raise  # Re-raise to trigger disconnection logic in worker thread

    def _write_pose_data_to_opcua(self):  # NEW Method for Pose writing
        """Writes the latest Pose data (x, y, theta) to the OPC UA server."""
        if not self.pose_received_first_time:
            self.get_logger().debug(
                "No Pose data received yet, skipping OPC UA Pose write."
            )
            return

        try:
            # Get the Node objects for Pose components
            x_node = self.client.get_node(self.opcua_pose_x_node_id)
            y_node = self.client.get_node(self.opcua_pose_y_node_id)
            theta_node = self.client.get_node(self.opcua_pose_theta_node_id)

            # Create DataValue objects (assuming PLC variables are Real/Float)
            dv_x = ua.DataValue(ua.Variant(self.latest_pose_x, ua.VariantType.Float))
            dv_y = ua.DataValue(ua.Variant(self.latest_pose_y, ua.VariantType.Float))
            dv_theta = ua.DataValue(
                ua.Variant(self.latest_pose_theta, ua.VariantType.Float)
            )

            # Write values to the respective nodes
            x_node.set_value(dv_x)
            y_node.set_value(dv_y)
            theta_node.set_value(dv_theta)

            self.get_logger().info(
                f"Successfully wrote Pose: x={self.latest_pose_x:.2f}, y={self.latest_pose_y:.2f}, theta={self.latest_pose_theta:.2f} to OPC UA."
            )

        except Exception as e:
            self.get_logger().error(f"Error writing Pose data to OPC UA server: {e}")
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
