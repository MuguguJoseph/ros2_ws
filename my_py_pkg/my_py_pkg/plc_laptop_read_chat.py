import snap7
import sys


def connect_to_plc(plc_ip):
    """Connect to the PLC."""
    plc = snap7.client.Client()
    try:
        # Connect to PLC at specified IP (Port 102 is default for S7 communication)
        plc.connect(plc_ip, 0, 1)  # rack=0, slot=1 for S7-1200
        print("Successfully connected to PLC!")
        return plc
    except Exception as e:
        print(f"Connection failed: {e}")
        sys.exit(1)


def read_data_from_plc(plc, db_number, start_byte, size):
    """Reads data from a specified data block in the PLC."""
    try:
        # Read 2 bytes of data from the specified Data Block (DB)
        data = plc.read_area(snap7.types.Areas.DB, db_number, start_byte, size)
        return data
    except Exception as e:
        print(f"Failed to read data: {e}")
        return None


def decode_serialized_data(data):
    """Decodes the 2-byte serialized message into an integer value."""
    if len(data) != 2:
        print("Error: Expected 2 bytes of data.")
        return None

    # Convert 2-byte data to a signed 16-bit integer (big-endian byte order)
    # For unsigned, use 'signed=False' in int.from_bytes
    try:
        integer_value = int.from_bytes(
            data, byteorder="big", signed=True
        )  # Change to 'signed=False' for unsigned
        return integer_value
    except Exception as e:
        print(f"Error in decoding: {e}")
        return None


def main():
    """Main function."""
    # Replace '192.168.0.1' with your PLC's IP address
    PLC_IP = "192.168.2.10"  # Change this to your actual PLC IP
    plc = connect_to_plc(PLC_IP)

    # Example: Read 2 bytes starting from DB1 at byte offset 0
    db_number = 1  # Data Block number (DB1)
    start_byte = 0  # Start byte offset
    size = 2  # Read 2 bytes

    # Read 2 bytes from the specified data block
    data = read_data_from_plc(plc, db_number, start_byte, size)
    if data:
        print(f"Raw data read from PLC: {data.hex()}")

        # Decode the 2-byte serialized data into an integer value
        integer_value = decode_serialized_data(data)
        if integer_value is not None:
            print(f"Decoded integer value: {integer_value}")

    # Disconnect from PLC
    plc.disconnect()
    print("Disconnected from PLC")


if __name__ == "__main__":
    main()
