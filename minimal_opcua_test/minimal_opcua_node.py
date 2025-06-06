#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from opcua import Client, ua
import time
import threading


class MinimalOpcUaClient(Node):
    def __init__(self):
        super().__init__("minimal_opcua_client")
        self.get_logger().info("Starting Minimal OPC UA Client Node...")

        # --- ROS2 Parameters for Configuration ---
        # These can be set when you run the node (see Phase 5)
        self.declare_parameter(
            "opcua_server_url", "opc.tcp://mugugu:53530/OPCUA/SimulationServer"
        )  # REPLACE THIS IP!
        self.declare_parameter(
            "opcua_write_node_id", 'ns=4;s="MyTestDB.WriteValue"'
        )  # UPDATE THIS NODE ID!
        self.declare_parameter(
            "opcua_read_node_id", 'ns=4;s="MyTestDB.ReadValue"'
        )  # UPDATE THIS NODE ID!
        self.declare_parameter(
            "opcua_user", ""
        )  # Optional: Leave empty if no authentication
        self.declare_parameter(
            "opcua_password", ""
        )  # Optional: Leave empty if no authentication

        self.opcua_url = self.get_parameter("opcua_server_url").value
        self.opcua_write_node_id = self.get_parameter("opcua_write_node_id").value
        self.opcua_read_node_id = self.get_parameter("opcua_read_node_id").value
        self.opcua_user = self.get_parameter("opcua_user").value
        self.opcua_password = self.get_parameter("opcua_password").value

        # --- OPC UA Client Setup ---
        self.client = Client(self.opcua_url)
        if self.opcua_user and self.opcua_password:
            self.client.set_user(self.opcua_user)
            self.client.set_password(self.opcua_password)

        self.connected = False
        self.opcua_connection_lock = threading.Lock()  # For thread-safe client access

        self.write_value_counter = 0.0  # Value to write to the server

        # --- Start OPC UA Connection and Test Thread ---
        self.opcua_thread = threading.Thread(target=self._opcua_worker_thread)
        self.opcua_thread.daemon = True  # Allows thread to exit when main program exits
        self.opcua_thread.start()

        self.get_logger().info(
            "Minimal OPC UA Client Node initialized. Attempting connection..."
        )

    def _opcua_worker_thread(self):
        """Thread to manage OPC UA connection and perform read/write tests."""
        while rclpy.ok():  # Keep running as long as the ROS2 node is active
            with self.opcua_connection_lock:  # Ensure only one thread accesses client at a time
                if not self.connected:  # false and not cancel each other to ouput TRUE
                    try:
                        self.get_logger().info(
                            f"Attempting to connect to OPC UA server at {self.opcua_url}..."
                        )
                        self.client.connect()
                        self.get_logger().info(
                            "Successfully connected to OPC UA server."
                        )
                        self.connected = True
                        # After successful connection, perform initial read/write
                        self._perform_test_operations()
                    except Exception as e:
                        self.connected = False
                        self.get_logger().error(
                            f"Failed to connect to OPC UA server: {e}. Retrying in 5 seconds..."
                        )
                        time.sleep(5)  # Wait before next connection attempt
                else:
                    # If connected, periodically perform read/write operations
                    try:
                        self._perform_test_operations()
                    except Exception as e:
                        self.get_logger().error(
                            f"Error during OPC UA operations: {e}. Assuming connection lost, attempting reconnect..."
                        )
                        self.connected = (
                            False  # Mark as disconnected to trigger reconnection
                        )

            time.sleep(2)  # Perform test operations every 2 seconds

    def _perform_test_operations(self):
        """Performs a write and a read operation on the OPC UA server."""
        if not self.connected:  # understood the condition
            self.get_logger().warn(
                "Not connected to OPC UA server, skipping test operations."
            )
            return

        try:
            # 1. Get the Node objects
            write_node = self.client.get_node(self.opcua_write_node_id)
            read_node = self.client.get_node(self.opcua_read_node_id)

            # 2. Write a value to the write node
            self.write_value_counter += 0.1  # Increment value for demonstration
            # Assuming the PLC variable is a Real (Float)
            dv_write = ua.DataValue(
                ua.Variant(self.write_value_counter, ua.VariantType.Float)
            )  # ua.Variant(): This is a constructor from the opcua.ua module
            # self.write_value_counter: This is the actual Python float value you want to send to the PLC.
            # ua.VariantType.Float: This specifies that the data type of the value being sent is a 32-bit
            # floating-point number, which corresponds to the Real data type in Siemens PLCs
            # ua.DataValue(): This wraps the ua.Variant into an OPC UA DataValue
            write_node.set_value(dv_write)
            # .set_value(): This is the method used to write a value to an OPC UA node.
            # This line sends the self.write_value_counter (encapsulated in dv_write) from your ROS2
            #  program to the RosWriteValue variable in your PLC.
            self.get_logger().info(
                f"Successfully wrote {self.write_value_counter:.2f} to '{self.opcua_write_node_id}'"
            )

            # 3. Read a value from the read node
            read_value = read_node.get_value()
            self.get_logger().info(
                f"Successfully read {read_value} from '{self.opcua_read_node_id}'"
            )

        except Exception as e:
            self.get_logger().error(f"Error during OPC UA read/write operations: {e}")
            raise  # Re-raise to trigger disconnection logic in worker thread

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
