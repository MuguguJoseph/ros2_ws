#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import struct
import time  # To add delays between retries


class PlcIntegerReceiver(Node):
    def __init__(self):
        super().__init__("plc_integer_receiver")

        # PLC connection settings
        self.plc_ip = "192.168.2.10"  # Update with the correct IP
        self.plc_port = 2000  # Ensure this matches the PLC configuration

        # Create a TCP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(5)  # Set a timeout for connection attempts

        # Attempt to connect to the PLC
        self.connect_to_plc()

        # Timer to receive data every second
        self.timer = self.create_timer(1.0, self.receive_data)

    def connect_to_plc(self):
        """Attempts to establish a connection to the PLC with retries."""
        attempts = 5  # Number of retries
        for attempt in range(1, attempts + 1):
            try:
                self.get_logger().info(
                    f"🔄 Attempting to connect to PLC (Try {attempt}/{attempts})..."
                )
                self.sock.connect((self.plc_ip, self.plc_port))
                self.get_logger().info(
                    f"✅ Successfully connected to PLC at {self.plc_ip}:{self.plc_port}"
                )
                return  # Exit function after a successful connection
            except socket.timeout:
                self.get_logger().error("⏳ Connection timed out. Retrying...")
            except ConnectionRefusedError:
                self.get_logger().error(
                    "❌ Connection refused! Ensure the PLC is listening on this port."
                )
            except Exception as e:
                self.get_logger().error(f"⚠️ Unexpected error: {e}")

            time.sleep(2)  # Wait before retrying

        self.get_logger().fatal(
            "🚨 Could not connect to PLC after multiple attempts. Exiting..."
        )
        self.destroy_node()  # Shutdown node if connection fails

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
        self.get_logger().info("🔌 Closing TCP connection...")
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PlcIntegerReceiver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
