#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from opcua import Client, ua
import time
import threading


class MinimalOpcUaClient(Node):
    def __init__(self):
        super().__init__("minimal_opcua_client")
        self.get_logger().info("Starting Minimal OPC UA Client Node (Connect Only)...")

        # --- ROS2 Parameters for Configuration ---
        # These can be set when you run the node
        self.declare_parameter(
            "opcua_server_url", "opc.tcp://mugugu:53530/OPCUA/SimulationServer"
        )
        self.declare_parameter(
            "opcua_user", ""
        )  # Optional: Leave empty if no authentication
        self.declare_parameter(
            "opcua_password", ""
        )  # Optional: Leave empty if no authentication

        self.opcua_url = self.get_parameter("opcua_server_url").value
        self.opcua_user = self.get_parameter("opcua_user").value
        self.opcua_password = self.get_parameter("opcua_password").value

        # --- OPC UA Client Setup ---
        self.client = Client(self.opcua_url)
        if self.opcua_user and self.opcua_password:
            self.client.set_user(self.opcua_user)
            self.client.set_password(self.opcua_password)

        self.connected = False
        self.opcua_connection_lock = threading.Lock()  # For thread-safe client access

        # --- Start OPC UA Connection Thread ---
        self.opcua_thread = threading.Thread(target=self._opcua_worker_thread)
        self.opcua_thread.daemon = True  # Allows thread to exit when main program exits
        self.opcua_thread.start()

        self.get_logger().info(
            "Minimal OPC UA Client Node initialized. Attempting connection..."
        )

    def _opcua_worker_thread(self):
        """Thread to manage OPC UA connection."""
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
                        # No read/write operations here
                    except Exception as e:
                        self.connected = False
                        self.get_logger().error(
                            f"Failed to connect to OPC UA server: {e}. Retrying in 5 seconds..."
                        )
                        time.sleep(5)  # Wait before next connection attempt
                else:
                    # If connected, just periodically check the connection is still alive
                    try:
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

            time.sleep(2)  # Check connection status every 2 seconds

    # The _perform_test_operations method has been removed as it's no longer needed.

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
