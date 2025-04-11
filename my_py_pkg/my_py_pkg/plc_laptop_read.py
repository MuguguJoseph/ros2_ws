import snap7
from snap7.util import *
import sys


def connect_to_plc(plc_ip):
    # Connect to the PLC and return the client object
    plc = snap7.client.Client()
    try:
        plc.connect(plc_ip, rack=0, slot=1)  # S7-1200 parameters
        print(f"Connected to PLC at {plc_ip}")
        return plc
    except Exception as e:
        print(f"Connection failed: {e}")
        return None


def read_serialized_integer(plc, db_number, byte_offset, size=2):
    """
    Read a serialized integer from a data block
    :param plc: PLC client object
    :param db_number: Data block number
    :param byte_offset: Byte offset in the data block
    :param size: Size of integer in bytes (2 for INT, 4 for DINT)
    :return: Decoded integer value or None if failed
    """

    try:
        # Read raw bytes from the PLC
        data = plc.db_read(db_number, byte_offset, size)

        # Convert byte array to integer (using big-endian format typical for Siemens PLCs)
        if size == 2:
            value = get_int(data, 0)
        elif size == 4:
            value = get_dint(data, 0)
        else:
            raise ValueError("Unsupported integer size. Use 2 (INT) or 4 (DINT) bytes.")

        return value

    except Exception as e:
        print(f"Error reading/decoding data: {e}")
        return None


def main():
    #######################################################
    # CONFIGURATION PARAMETERS - ADJUST THESE TO MATCH YOUR PLC
    PLC_IP = "192.168.2.10"
    DB_NUMBER = 1  # Data block number to read from
    BYTE_OFFSET = 0  # Starting byte offset in the data block
    INTEGER_SIZE = 2  # Size of integer in bytes (2 for INT, 4 for DINT)
    #######################################################

    plc = connect_to_plc(PLC_IP)

    if plc is None:
        sys.exit(1)
        # If the connection to the PLC fails, the connect_to_plc function returns None.
    try:
        # Read and decode the integer value
        value = read_serialized_integer(plc, DB_NUMBER, BYTE_OFFSET, INTEGER_SIZE)

        if value is not None:
            print(f"Decoded integer value: {value}")

    finally:
        # Ensure proper disconnection
        plc.disconnect()
        print("Disconnected from PLC")


if __name__ == "__main__":
    main()
