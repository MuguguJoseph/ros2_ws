#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import snap7
import sys
import socket
import struct
import time  # To add delays between retries


class PlcCommunicationNode(Node):
    def __init__(self):
        super().__init__("plc_communication_node")

        # Define PLC connection parameters
        self.plc_ip = "192.168.2.10"  # Update with your PLC's actual IP
        self.plc_rack = 0
        self.plc_slot = 1
        self.plc_port = 2000  # Ensure this matches the PLC configuration

        # Create a snap7 client instance
        self.plc = snap7.client.Client()

        # Try to connect to the PLC
        self.connect_to_plc()

        # Create a TCP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(5)  # Set a timeout for connection attempts

        # Set up a timer to check the connection every 5 seconds
        self.timer = self.create_timer(5.0, self.check_connection_status)

        # Timer to receive data every second
        self.timer = self.create_timer(1.0, self.receive_data)

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

    def receive_data(self):
        """Receives the byte stream from the PLC and decodes it into an integer."""
        try:
            data = self.sock.recv(2)  # Expecting 2 bytes (integer)

            if not data:
                self.get_logger().warn(
                    "⚠️ No data received. PLC might have disconnected."
                )
                return

            if len(data) < 2:
                self.get_logger().warn(f"⚠️ Incomplete data received: {data}")
                return

            # Decode byte stream into an integer (Big-Endian 16-bit integer)
            int_value = struct.unpack(">h", data)[0]
            self.get_logger().info(f"📥 Received integer from PLC: {int_value}")

        except socket.timeout:
            self.get_logger().warn("⏳ Timeout while waiting for data.")
        except Exception as e:
            self.get_logger().error(f"❌ Error receiving data: {e}")

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
