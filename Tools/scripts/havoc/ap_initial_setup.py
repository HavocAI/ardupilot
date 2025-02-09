"""
ap_initial_setup.py

This script uses pymavlink to send commands to do the following:
1. Update the bootloader (assuming a new bootloader has been uploaded to the vehicle)
2. Simple accelerometer calibration, assuming the vehicle is on a level surface
3. Fixed magnetometer calibration, using the supplied heading and coordinates
4. Set parameters from a .param file, by first reading all parameters on the 
    vehicle and only updating those that are different

Usage:
    python3 ap_initial_setup.py --connect <connection_string> --lat <latitude> 
        --lon <longitude> --heading <heading> --param-file <param_file>

Arguments:
    --connect      Connection string (e.g., /dev/ttyACM0, tcp:127.0.0.1:5760)
    --lat          Latitude for mag calibration (default: 0.0 to use current GPS)
    --lon          Longitude for mag calibration (default: 0.0 to use current GPS)
    --heading      True heading (yaw) in degrees
    --param-file   Path to the .param file

Author:
    Andrew Gregg

Date:
    2015-02-08
"""

from pymavlink import mavutil
import argparse
import re
import time

class MavCommandHandler:
    """Base class for handling MAVLink commands and responses."""

    COMMAND_NAMES = {
        42650: "Flash Bootloader",
        241: "Simple Accel Calibration",
        42006: "Fixed Mag Calibration"
    }

    def __init__(self, connection_string):
        print(f"Connecting to vehicle on {connection_string}...")
        self.master = mavutil.mavlink_connection(connection_string, baud=115200)
        self.master.wait_heartbeat()
        print("Heartbeat received. Connected successfully.")

    def send_command(self, command_id, params):
        """Send a MAVLink COMMAND_LONG message."""
        command_name = self.COMMAND_NAMES.get(command_id, f"Command {command_id}")
        print(f"Sending {command_name} command...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            command_id,
            0,  # Confirmation
            *params
        )

    def wait_for_command_ack(self, command_id, timeout=3):
        """Wait for a COMMAND_ACK response."""
        command_name = self.COMMAND_NAMES.get(command_id, f"Command {command_id}")
        print(f"Waiting for COMMAND_ACK response for {command_name}...")

        while True:
            message = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=timeout)
            if message is None:
                raise RuntimeError("Timeout: No COMMAND_ACK received.")
            
            if message.command == command_id:
                if message.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    self.print_success(f"{command_name} executed successfully.")
                elif message.result == mavutil.mavlink.MAV_RESULT_DENIED:
                    self.print_error(f"{command_name} denied. Ensure the vehicle is disarmed.")
                else:
                    self.print_error(f"{command_name} failed. Result: {message.result}")
                break

    def close(self):
        """Close the MAVLink connection."""
        print("Closing connection...")
        self.master.close()
        print("Connection closed.")

    @staticmethod
    def print_success(message):
        """Print a success message in bold green text."""
        print(f"\033[1;32m{message}\033[0m")

    @staticmethod
    def print_error(message):
        """Print an error message in bold red text."""
        print(f"\033[1;31m{message}\033[0m")


class BootloaderUpdater(MavCommandHandler):
    """Handles bootloader update commands."""
    MAV_CMD_FLASH_BOOTLOADER = 42650
    MAGIC_NUMBER = 290876.0

    def update_bootloader(self):
        params = [0.0, 0.0, 0.0, 0.0, self.MAGIC_NUMBER, 0.0, 0.0]
        self.send_command(self.MAV_CMD_FLASH_BOOTLOADER, params)
        self.wait_for_command_ack(self.MAV_CMD_FLASH_BOOTLOADER)


class SimpleAccelCalibration(MavCommandHandler):
    """Handles simple accelerometer calibration."""
    MAV_CMD_PREFLIGHT_CALIBRATION = 241

    def calibrate_accel(self):
        params = [0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0]
        self.send_command(self.MAV_CMD_PREFLIGHT_CALIBRATION, params)
        self.wait_for_command_ack(self.MAV_CMD_PREFLIGHT_CALIBRATION)


class FixedMagCal(MavCommandHandler):
    """Handles fixed magnetometer calibration."""
    MAV_CMD_FIXED_MAG_CAL_YAW = 42006

    def check_gps_fix(self):
        """Check if the vehicle has a valid GPS fix."""
        print("Checking for GPS fix...")
        gps_message = self.master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
        if gps_message is None or gps_message.fix_type < 2:
            self.print_error("No lat/lon specified and invalid GPS fix. Mag cal failed.")
            return 0.0, 0.0
        print(f"GPS fix acquired: fix_type={gps_message.fix_type}, lat={gps_message.lat / 1e7}, lon={gps_message.lon / 1e7}")
        return gps_message.lat / 1e7, gps_message.lon / 1e7

    def calibrate_fixed_mag(self, yaw_deg, lat_deg, lon_deg):
        """Perform fixed magnetometer calibration with the given yaw and coordinates."""
        if lat_deg == 0.0 and lon_deg == 0.0:
            if self.check_gps_fix() == (0.0, 0.0):
                return  # Exit if no valid GPS fix
            else:
                lat_deg, lon_deg = self.check_gps_fix()

        params = [yaw_deg, 0.0, lat_deg, lon_deg, 0.0, 0.0, 0.0]
        self.send_command(self.MAV_CMD_FIXED_MAG_CAL_YAW, params)
        self.wait_for_command_ack(self.MAV_CMD_FIXED_MAG_CAL_YAW)

class ParamSetter(MavCommandHandler):
    """Read a .param file and set parameters on the vehicle."""

    def read_param_file(self, file_path):
        """Read and parse the .param file."""
        parameters = {}
        print(f"Reading parameter file: {file_path}")

        with open(file_path, 'r') as file:
            for line in file:
                # Skip comments and empty lines
                if line.startswith("#") or not line.strip():
                    continue

                # Match parameter name and value (e.g., "SYSID_THISMAV,1.0")
                match = re.match(r"^(\w+),(-?\d+(\.\d+)?)$", line.strip())
                if match:
                    param_name, param_value = match.group(1), float(match.group(2))
                    parameters[param_name] = param_value
                else:
                    self.print_error(f"Skipping invalid line: {line.strip()}")

        print(f"Read {len(parameters)} parameters.")
        return parameters

    def get_all_parameters(self):
        """Retrieve all parameters from the vehicle."""
        print("Fetching all parameters from the vehicle...")
        parameters = {}

        # Request all parameters
        self.master.mav.param_request_list_send(
            self.master.target_system,
            self.master.target_component
        )

        while True:
            message = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
            if message is None:
                break  # End of parameter list

            param_name = message.param_id.rstrip('\x00')
            param_value = message.param_value
            parameters[param_name] = param_value

        print(f"Fetched {len(parameters)} parameters from the vehicle.")
        return parameters

    def refresh_parameters(self):
        """Refresh the current parameters from the vehicle."""
        self.current_params = self.get_all_parameters()
        print("Parameters refreshed.")


    def get_current_parameter_type(self, param_name):
        """Retrieve the current type of a parameter from the vehicle."""

        # Mavlink requires null termination unless the name is exactly 16 bytes
        encoded_param_name = param_name.encode('utf-8')
        if len(encoded_param_name) < 16:
            encoded_param_name += b'\x00'

        self.master.mav.param_request_read_send(
            self.master.target_system,
            self.master.target_component,
            encoded_param_name,
            -1
        )

        response = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
        if response and response.param_id.rstrip('\x00') == param_name:
            return response.param_type
        return None

    def set_parameter(self, param_name, param_value):
        """Send a single parameter to the vehicle with retries and delay."""
        param_type = self.get_current_parameter_type(param_name)
        if param_type is None:
            self.print_error(f"Failed to retrieve parameter type for {param_name}. Skipping...")
            return

        print(f"Setting parameter: {param_name} = {param_value}")

        # Mavlink requires null termination unless the name is exactly 16 bytes
        encoded_param_name = param_name.encode('utf-8')
        if len(encoded_param_name) < 16:
            encoded_param_name += b'\x00'

        # Send the parameter using MAVLink's param_set message
        self.master.mav.param_set_send(
            self.master.target_system,
            self.master.target_component,
            encoded_param_name,
            param_value,
            param_type
        )

        # Wait for acknowledgment
        ack = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if ack:
            ack_param_name = ack.param_id.rstrip('\x00')
            print(f"Received ack: param_id={ack_param_name}, param_value={ack.param_value}")
            if ack_param_name == param_name:
                self.print_success(f"Parameter {param_name} set successfully.")
                return
            else:
                self.print_error(f"Received ack for wrong parameter: {ack_param_name}")
        else:
            self.print_error(f"No acknowledgment received for parameter {param_name}. Retrying...")

        self.print_error(f"Failed to set parameter {param_name}")

    def set_all_parameters(self, parameters):
        """Set all parameters, only updating those that are different."""
        self.refresh_parameters()

        # Filter out parameters that are already set to the desired values
        params_to_update = {
            k: v for k, v in parameters.items() 
            if k not in self.current_params or abs(self.current_params[k] - v) > 1e-6
        }

        print(f"Updating {len(params_to_update)} parameters...")

        # Handle enable parameters first
        enable_params = {k: v for k, v in params_to_update.items() if 'ENABLE' in k or 'ENABLED' in k}
        other_params = {k: v for k, v in params_to_update.items() if k not in enable_params}

        # Set enable parameters first
        if enable_params:
            print("Setting enable parameters...")
            for param_name, param_value in enable_params.items():
                self.set_parameter(param_name, param_value)
            time.sleep(1.0)

        # Set remaining parameters
        print("Setting remaining parameters...")
        for param_name, param_value in other_params.items():
            self.set_parameter(param_name, param_value)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Initial setup script for Autopilot.")
    parser.add_argument('--connect', required=True, help="Connection string (e.g., /dev/ttyACM0, tcp:127.0.0.1:5760)")
    parser.add_argument('--lat', type=float, default=0.0, help="Latitude for calibration (default: 0.0 to use current GPS)")
    parser.add_argument('--lon', type=float, default=0.0, help="Longitude for calibration (default: 0.0 to use current GPS)")
    parser.add_argument('--heading', type=float, required=True, help="True heading (yaw) in degrees")
    parser.add_argument('--param-file', required=True, help="Path to the .param file")
    args = parser.parse_args()

    # Bootloader update
    updater = BootloaderUpdater(args.connect)
    try:
        updater.update_bootloader()
    finally:
        updater.close()

    # Accelerometer calibration
    calibrator = SimpleAccelCalibration(args.connect)
    try:
        calibrator.calibrate_accel()
    finally:
        calibrator.close()

    # Fixed mag calibration
    fixed_mag_cal = FixedMagCal(args.connect)
    try:
        fixed_mag_cal.calibrate_fixed_mag(args.heading, args.lat, args.lon)
    finally:
        fixed_mag_cal.close()

    # Parameter setting
    setter = ParamSetter(args.connect)
    try:
        parameters = setter.read_param_file(args.param_file)
        setter.set_all_parameters(parameters)
    finally:
        setter.close()
