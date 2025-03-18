/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
/*
    AP_BattMonitor_SSM.cpp

    Author: Andrew Gregg

    Description:

    Configuring CAN Parameters in ArduRover:
    (Example shows CAN_P2 because this connects to the port marked "CAN0" on the Airbot carrier board)
    CAN_D1_PROTOCOL = 16 -- Sets the driver 1 protocol to SSM Battery
    CAN_P2_DRIVER = 1 -- Sets the 2nd CAN port to use driver 1
    CAN_P2_BITRATE = 250000 -- Sets the 2nd CAN port bitrate

    Settable Parameters - Description, Default value:

    Telemetry Outputs:

*/

#include "AP_BattMonitor_SSM.h"

#if AP_BATTERY_SSM_ENABLED
#include <stdio.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL &hal;

#define AP_SSMBATTERY_DEBUG 0
#define AP_BATT_MONITOR_SSM_TIMEOUT_US 5000000

// Called by frontend to update the state. Called at 10Hz
void AP_BattMonitor_SSM::read()
{
    WITH_SEMAPHORE(sem);

    // Check for timeout, to prevent a faulty script from appearing healthy
    if (last_update_us == 0 || AP_HAL::micros() - last_update_us > AP_BATT_MONITOR_SSM_TIMEOUT_US) {
        _state.healthy = false;
        return;
    }

    if (_state.last_time_micros == last_update_us) {
        // No new data
        return;
    }

    for (uint8_t i = 0; i < AP_BATT_MONITOR_CELLS_MAX; i++) {
        _state.cell_voltages.cells[i] = _internal_state.cell_voltages.cells[i];
    }
    _state.voltage = _internal_state.voltage;
    if (!isnan(_internal_state.current_amps)) {
        _state.current_amps = _internal_state.current_amps;
    }
    if (!isnan(_internal_state.consumed_mah)) {
        _state.consumed_mah = _internal_state.consumed_mah;
    }
    // Overide integrated consumed energy if it has been set
    if (!isnan(_internal_state.consumed_wh)) {
        _state.consumed_wh = _internal_state.consumed_wh;
    }
    if (!isnan(_internal_state.temperature)) {
        _state.temperature = _internal_state.temperature;
    }

    _state.healthy = _internal_state.healthy;

    // Update the timestamp (has to be done after the consumed_mah calculation)
    _state.last_time_micros = last_update_us;
}


void AP_BattMonitor_SSM::init()
{

    if (_driver != nullptr)
    {
        // only allow one instance
        return;
    }

    for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++)
    {
        if (CANSensor::get_driver_type(i) == AP_CAN::Protocol::SSMBattery)
        {
            _driver = NEW_NOTHROW AP_BattMonitor_SSM();
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SSM Battery: Initialized using CAN%d", i);
            return;
        }
    }

    register_driver(AP_CAN::Protocol::SSMBattery);

    // start thread for receiving and sending CAN frames.
    hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_BattMonitor_SSM::loop, void), "ssm_battery", 2048, AP_HAL::Scheduler::PRIORITY_CAN, 0);

}

// parse inbound frames
void AP_BattMonitor_SSM::handle_frame(AP_HAL::CANFrame &frame)
{
    if (!frame.isExtended()) {
        return;
    }
    const uint32_t ext_id = frame.id & AP_HAL::CANFrame::MaskExtID;

    switch (ext_id)
    {
    case SSMBATTERY_QUERY_FRAME_FRAME_ID:
    {
        struct ssmbattery_query_frame_t msg;
        ssmbattery_query_frame_unpack(&msg, frame.data, frame.dlc);
        handle_query_frame(msg);
        break;
    }
    case SSMBATTERY_CELL_VOLTAGE_INFORMATION_FRAME_ID:
    {
        struct ssmbattery_cell_voltage_information_t msg;
        ssmbattery_cell_voltage_information_unpack(&msg, frame.data, frame.dlc);
        handle_cell_voltage_information(msg);
        break;
    }
    case SSMBATTERY_CELL_TEMPERATURE_INFORMATION_FRAME_ID:
    {
        struct ssmbattery_cell_temperature_information_t msg;
        ssmbattery_cell_temperature_information_unpack(&msg, frame.data, frame.dlc);
        handle_cell_temperature_information(msg);
        break;
    }
    case SSMBATTERY_TOTAL_INFORMATION_0_FRAME_ID:
    {
        struct ssmbattery_total_information_0_t msg;
        ssmbattery_total_information_0_unpack(&msg, frame.data, frame.dlc);
        handle_total_information_0(msg);
        break;
    }
    case SSMBATTERY_TOTAL_INFORMATION_1_FRAME_ID:
    {
        struct ssmbattery_total_information_1_t msg;
        ssmbattery_total_information_1_unpack(&msg, frame.data, frame.dlc);
        handle_total_information_1(msg);
        break;
    }
    case SSMBATTERY_CELL_VOLTAGE_STATISTICAL_INFORMATION_FRAME_ID:
    {
        struct ssmbattery_cell_voltage_statistical_information_t msg;
        ssmbattery_cell_voltage_statistical_information_unpack(&msg, frame.data, frame.dlc);
        handle_cell_voltage_statistical_information(msg);
        break;
    }
    case SSMBATTERY_UNIT_TEMPERATURE_STATISTICAL_INFORMATION_FRAME_ID:
    {
        struct ssmbattery_unit_temperature_statistical_information_t msg;
        ssmbattery_unit_temperature_statistical_information_unpack(&msg, frame.data, frame.dlc);
        handle_unit_temperature_statistical_information(msg);
        break;
    }
    case SSMBATTERY_STATUS_INFORMATION_0_FRAME_ID:
    {
        struct ssmbattery_status_information_0_t msg;
        ssmbattery_status_information_0_unpack(&msg, frame.data, frame.dlc);
        handle_status_information_0(msg);
        break;
    }
    case SSMBATTERY_STATUS_INFORMATION_1_FRAME_ID:
    {
        struct ssmbattery_status_information_1_t msg;
        ssmbattery_status_information_1_unpack(&msg, frame.data, frame.dlc);
        handle_status_information_1(msg);
        break;
    }
    case SSMBATTERY_STATUS_INFORMATION_2_FRAME_ID:
    {
        struct ssmbattery_status_information_2_t msg;
        ssmbattery_status_information_2_unpack(&msg, frame.data, frame.dlc);
        handle_status_information_2(msg);
        break;
    }
    case SSMBATTERY_HARDWARE_AND_BATTERY_FAILURE_INFORMATION_FRAME_ID:
    {
        struct ssmbattery_hardware_and_battery_failure_information_t msg;
        ssmbattery_hardware_and_battery_failure_information_unpack(&msg, frame.data, frame.dlc);
        handle_hardware_and_battery_failure_information(msg);
        break;
    }
    case SSMBATTERY_CHARGING_INFORMATION_FRAME_ID:
    {
        struct ssmbattery_charging_information_t msg;
        ssmbattery_charging_information_unpack(&msg, frame.data, frame.dlc);
        handle_charging_information(msg);
        break;
    }
    case SSMBATTERY_LIMITING_FRAME_ID:
    {
        struct ssmbattery_limiting_t msg;
        ssmbattery_limiting_unpack(&msg, frame.data, frame.dlc);
        handle_limiting(msg);
        break;
    }
    case SSMBATTERY_FAULT_FRAME_ID:
    {
        struct ssmbattery_fault_t msg;
        ssmbattery_fault_unpack(&msg, frame.data, frame.dlc);
        handle_fault(msg);
        break;
    }
    default:
        // Ignore other frames
        break;
    }
}

void AP_BattMonitor_SSM::loop()
{
    int16_t motor_rpm_cmd = 0;
    uint8_t trim_cmd = 0;
    uint32_t last_query_time = 0;

    while (true)
    {
        hal.scheduler->delay_microseconds(20000); // 50Hz

        const uint32_t now_ms = AP_HAL::millis();

        // Send query frame every 2 seconds
        if (now_ms - last_query_time >= 2000) {
            send_query_frame();
            last_query_time = now_ms;
        }

    } // while true
}

void AP_BattMonitor_SSM::send_query_frame()
{
    struct ssmbattery_query_frame_t query_msg;
    ssmbattery_query_frame_init(&query_msg);
    uint8_t data[SSMBATTERY_QUERY_FRAME_LENGTH];
    ssmbattery_query_frame_pack(data, &query_msg, sizeof(data));
    send_packet(SSMBATTERY_QUERY_FRAME_FRAME_ID, 1000, data, sizeof(data));
}

bool AP_BattMonitor_SSM::send_packet(const uint32_t id, const uint32_t timeout_us, 
    const uint8_t *data, const uint8_t data_len)
{
    AP_HAL::CANFrame frame;
    frame.id = id | AP_HAL::CANFrame::FlagEFF;
    frame.dlc = data_len;
    frame.canfd = false;
    memcpy(frame.data, data, data_len);

    return write_frame(frame, timeout_us);
}

void AP_BattMonitor_SSM::handle_query_frame(const struct ssmbattery_query_frame_t &msg)
{
    // Handle the query frame message
}

void AP_BattMonitor_SSM::handle_cell_voltage_information(const struct ssmbattery_cell_voltage_information_t &msg)
{
    // Handle the cell voltage information message
    WITH_SEMAPHORE(sem);
}

void AP_BattMonitor_SSM::handle_cell_temperature_information(const struct ssmbattery_cell_temperature_information_t &msg)
{
    // Handle the cell temperature information message
    WITH_SEMAPHORE(sem);
}

void AP_BattMonitor_SSM::handle_total_information_0(const struct ssmbattery_total_information_0_t &msg)
{
    // Handle the total information 0 message
    WITH_SEMAPHORE(sem);
}

void AP_BattMonitor_SSM::handle_total_information_1(const struct ssmbattery_total_information_1_t &msg)
{
    // Handle the total information 1 message
    WITH_SEMAPHORE(sem);
}

void AP_BattMonitor_SSM::handle_cell_voltage_statistical_information(const struct ssmbattery_cell_voltage_statistical_information_t &msg)
{
    // Handle the cell voltage statistical information message
    WITH_SEMAPHORE(sem);
}

void AP_BattMonitor_SSM::handle_unit_temperature_statistical_information(const struct ssmbattery_unit_temperature_statistical_information_t &msg)
{
    // Handle the unit temperature statistical information message
    WITH_SEMAPHORE(sem);
}

void AP_BattMonitor_SSM::handle_status_information_0(const struct ssmbattery_status_information_0_t &msg)
{
    // Handle the status information 0 message
    WITH_SEMAPHORE(sem);
}

void AP_BattMonitor_SSM::handle_status_information_1(const struct ssmbattery_status_information_1_t &msg)
{
    // Handle the status information 1 message
    WITH_SEMAPHORE(sem);
}

void AP_BattMonitor_SSM::handle_status_information_2(const struct ssmbattery_status_information_2_t &msg)
{
    // Handle the status information 2 message
    WITH_SEMAPHORE(sem);
}

void AP_BattMonitor_SSM::handle_hardware_and_battery_failure_information(const struct ssmbattery_hardware_and_battery_failure_information_t &msg)
{
    // Handle the hardware and battery failure information message
    WITH_SEMAPHORE(sem);
}

void AP_BattMonitor_SSM::handle_charging_information(const struct ssmbattery_charging_information_t &msg)
{
    // Handle the charging information message
    WITH_SEMAPHORE(sem);
}

void AP_BattMonitor_SSM::handle_limiting(const struct ssmbattery_limiting_t &msg)
{
    // Handle the limiting message
    WITH_SEMAPHORE(sem);
}

void AP_BattMonitor_SSM::handle_fault(const struct ssmbattery_fault_t &msg)
{
    // Handle the fault message
    WITH_SEMAPHORE(sem);
}

#endif // AP_BATTERY_SSM_ENABLED
