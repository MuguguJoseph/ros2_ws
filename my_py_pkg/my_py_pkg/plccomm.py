#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import snap7
import sys


class PlcCommunicationNode(Node):
    def __init__(self):
        super().__init__("plc_communication_node")

        # Define PLC connection parameters
        self.plc_ip = "192.168.2.10"  # Update with your PLC's actual IP
        self.plc_rack = 0
        self.plc_slot = 1

        # Create a snap7 client instance
        self.plc = snap7.client.Client()

        # Try to connect to the PLC
        self.connect_to_plc()

        # Set up a timer to check the connection every 5 seconds
        self.timer = self.create_timer(5.0, self.check_connection_status)

    def connect_to_plc(self):
        """Attempts to connect to the PLC."""
        try:
            self.plc.connect(self.plc_ip, self.plc_rack, self.plc_slot)
            if self.plc.get_connected():
                self.get_logger().info(
                    f"✅ Successfully connected to PLC at {self.plc_ip}"
                )
            else:
                self.get_logger().warn("⚠️ Connection attempt failed, will retry...")
        except Exception as e:
            self.get_logger().error(f"❌ Connection failed: {e}")
            sys.exit(1)  # Stop the node if connection fails initially

    def check_connection_status(self):
        """Checks if the PLC is still connected."""
        if self.plc.get_connected():
            self.get_logger().info("🔄 Connection to PLC is active.")
        else:
            self.get_logger().warn(
                "⚠️ Lost connection to PLC! Attempting to reconnect..."
            )
            self.connect_to_plc()  # Reattempt connection

    def destroy_node(self):
        """Disconnects from the PLC when shutting down."""
        self.get_logger().info("🔌 Disconnecting from PLC...")
        self.plc.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PlcCommunicationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Node interrupted, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
