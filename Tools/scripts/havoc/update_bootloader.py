import struct
import serial
import argparse

class MAVLinkBase:
    MAVLINK_STX = 0xFE
    MAVLINK_MSG_ID_COMMAND_LONG = 76
    SYSTEM_ID = 1
    COMPONENT_ID = 1

    def __init__(self, port, baud_rate):
        try:
            self.port = serial.Serial(port, baud_rate, timeout=1, write_timeout=1)
            print(f"Connected to {port} at {baud_rate} baud.")
        except serial.SerialException as e:
            raise RuntimeError(f"Could not open port {port}: {e}")

    def calculate_checksum(self, message, crc_extra):
        checksum = 0xFFFF
        for byte in message:
            tmp = byte ^ (checksum & 0xFF)
            tmp = (tmp ^ (tmp << 4)) & 0xFF
            checksum = (checksum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
        checksum = (checksum ^ crc_extra) & 0xFFFF
        return checksum

    def build_command_long_message(self, command, params):
        payload = struct.pack('<fffffffHBBB', *params, command, 0, self.SYSTEM_ID, self.COMPONENT_ID)
        header = struct.pack('<BBBBBB', self.MAVLINK_STX, len(payload), 0, self.SYSTEM_ID, self.COMPONENT_ID, self.MAVLINK_MSG_ID_COMMAND_LONG)
        checksum = self.calculate_checksum(header[1:] + payload, 152)
        return header + payload + struct.pack('<H', checksum)

    def send_message(self, message):
        try:
            self.port.write(message)
        except serial.SerialException as e:
            raise RuntimeError(f"Failed to send message: {e}")

    def receive_response(self, expected_bytes=6):
        try:
            response = self.port.read(expected_bytes)
            if len(response) < expected_bytes:
                raise RuntimeError("Timeout: Did not receive a complete response.")
            return response
        except serial.SerialException as e:
            raise RuntimeError(f"Failed to read response: {e}")

    def close(self):
        if self.port.is_open:
            self.port.close()
            print("Connection closed.")


class BootloaderUpdater(MAVLinkBase):
    MAV_CMD_FLASH_BOOTLOADER = 42650
    MAGIC_NUMBER = 290876.0

    def update_bootloader(self):
        params = [0.0, 0.0, 0.0, 0.0, self.MAGIC_NUMBER, 0.0, 0.0]
        message = self.build_command_long_message(self.MAV_CMD_FLASH_BOOTLOADER, params)

        print("Sending bootloader update command...")
        self.send_message(message)

        print("Waiting for response...")
        response = self.receive_response()

        # Check response for success (simple example based on MAVLink structure)
        if response[0] == self.MAVLINK_STX and response[-1] == 0x00:
            print("Bootloader update command executed successfully.")
        else:
            raise RuntimeError("Bootloader update failed: Invalid response received.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Bootloader Updater for ArduPilot devices.")
    parser.add_argument('--port', required=True, help="Serial port where the ArduPilot device is connected (e.g., /dev/ttyUSB0, COM3)")
    parser.add_argument('--baud', type=int, default=115200, help="Baud rate for serial communication (default: 115200)")
    args = parser.parse_args()

    try:
        updater = BootloaderUpdater(args.port, args.baud)
        try:
            updater.update_bootloader()
        finally:
            updater.close()
    except RuntimeError as e:
        print(f"Error: {e}")
        exit(1)
