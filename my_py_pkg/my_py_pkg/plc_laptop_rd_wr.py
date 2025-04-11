import snap7
from snap7.util import *
import sys


def connect_to_plc(plc_ip):
    """Connect to the PLC and return the client object"""
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
    :return: Decoded integer value or None if failed
    """
    try:
        data = plc.db_read(db_number, byte_offset, size)
        if size == 2:
            return get_int(data, 0)
        elif size == 4:
            return get_dint(data, 0)
        else:
            raise ValueError("Unsupported integer size. Use 2 (INT) or 4 (DINT) bytes.")
    except Exception as e:
        print(f"Read error: {e}")
        return None


def write_serialized_integer(plc, db_number, byte_offset, value, size=2):
    """
    Write an integer to a data block
    :return: True if successful, False if failed
    """
    try:
        # Create byte array of appropriate size
        data = bytearray(size)

        # Pack the value into the byte array
        if size == 2:
            set_int(data, 0, value)
        elif size == 4:
            set_dint(data, 0, value)
        else:
            raise ValueError("Unsupported integer size. Use 2 (INT) or 4 (DINT) bytes.")

        # Write to PLC
        plc.db_write(db_number, byte_offset, data)
        return True
    except Exception as e:
        print(f"Write error: {e}")
        return False


def main():
    #######################################################
    # CONFIGURATION PARAMETERS
    PLC_IP = "192.168.2.10"
    DB_NUMBER = 1  # Data block number
    BYTE_OFFSET = 0  # Byte offset in data block
    INTEGER_SIZE = 2  # 2 for INT (16-bit), 4 for DINT (32-bit)
    #######################################################

    plc = connect_to_plc(PLC_IP)
    if not plc:
        sys.exit(1)

    try:
        # Read initial value
        initial_value = read_serialized_integer(
            plc, DB_NUMBER, BYTE_OFFSET, INTEGER_SIZE
        )
        if initial_value is not None:
            print(f"Current value at DB{DB_NUMBER}.DBB{BYTE_OFFSET}: {initial_value}")

        # Example write operation
        try:
            new_value = int(
                input("Enter new integer value to write (or press Enter to skip): ")
            )
        except ValueError:
            print("No valid input - skipping write operation")
            new_value = None

        if new_value is not None:
            if write_serialized_integer(
                plc, DB_NUMBER, BYTE_OFFSET, new_value, INTEGER_SIZE
            ):
                print("Write successful! Verifying...")
                # Read back to confirm
                updated_value = read_serialized_integer(
                    plc, DB_NUMBER, BYTE_OFFSET, INTEGER_SIZE
                )
                if updated_value == new_value:
                    print(f"Verified new value: {updated_value}")
                else:
                    print("Verification failed! Value mismatch")

    finally:
        plc.disconnect()
        print("Disconnected from PLC")


if __name__ == "__main__":
    main()
