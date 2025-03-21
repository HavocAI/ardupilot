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
    This class is a backend for the AP_BattMonitor class that interfaces with the Solid State Marine
    (SSM) Battery over CAN. The driver sends a request frame to the battery every 2 seconds and
    listens for responses. The battery sends back a variety of information including cell voltages,
    total voltage, current, consumed energy, temperature, and faults. 

    Note: Individual cell voltages are not reported because Battery Manager will use these to 
    override the total voltage value and SSM battery strangely reports voltages for the wrong number 
    of cells, so the sum would be incorrect.

    Configuring Parameters in ArduRover: configure the CAN port to use the J1939 CAN backend
    (See AP_J1939_CAN/AP_J1939_CAN.h for more information)

    Telemetry Outputs:
    - Battery Voltage
    - Battery Current
    - Battery Remaining Capacity
    - Battery Temperature
    - Battery Faults

*/

#include "AP_BattMonitor_SSM.h"

#if AP_BATTERY_SSM_ENABLED
#include <stdio.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

#define AP_SSMBATTERY_DEBUG 0
#define AP_BATT_MONITOR_SSM_TIMEOUT_US 5000000

const AP_Param::GroupInfo AP_BattMonitor_SSM::var_info[] = {
    
    // @Param: CAN_PORT
    // @DisplayName: CAN Port
    // @Description: CAN Port, by default this is set to 0
    // @Values: 0:1
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("CAN_PORT", 6, AP_BattMonitor_SSM, _can_port, 0),

    AP_GROUPEND};

// Constructor
AP_BattMonitor_SSM::AP_BattMonitor_SSM(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &state, AP_BattMonitor_Params &params)
    : CANSensor("SSMBattery"), AP_BattMonitor_Backend(mon, state, params)
{
    _state.healthy = false;

    // start thread for receiving and sending CAN frames.
    hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_BattMonitor_SSM::loop, void), "ssm_battery", 2048, AP_HAL::Scheduler::PRIORITY_CAN, 0);
}

bool AP_BattMonitor_SSM::capacity_remaining_pct(uint8_t &percentage) const
{
    if (_capacity_remaining_pct != UINT8_MAX) {
        percentage = _capacity_remaining_pct;
        return true;
    }
    // Fall back to default implementation
    return AP_BattMonitor_Backend::capacity_remaining_pct(percentage);
}

// Called by frontend to update the state. Called at 10Hz
void AP_BattMonitor_SSM::read()
{
    WITH_SEMAPHORE(_sem);

    // Check for timeout
    if (_last_update_us == 0 || AP_HAL::micros() - _last_update_us > AP_BATT_MONITOR_SSM_TIMEOUT_US) {
        _state.healthy = false;
        return;
    }
    else {
        _state.healthy = true;
    }
    if (_state.last_time_micros == _last_update_us) {
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

    // Update the timestamp (has to be done after the consumed_mah calculation)
    _state.last_time_micros = _last_update_us;
}

// parse inbound frames
void AP_BattMonitor_SSM::handle_frame(AP_HAL::CANFrame &frame)
{
    if (!frame.isExtended()) {
        return;
    }
    const uint32_t ext_id = frame.id & AP_HAL::CANFrame::MaskExtID;

    uint32_t base_id, board_number;
    split_id(ext_id, base_id, board_number);

#if AP_SSMBATTERY_DEBUG
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SSM Battery: Received frame from board: %ld: %ld", board_number, base_id);
#endif

    // Switch on the base_id to determine the message type
    switch (base_id)
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
    uint32_t last_query_time = 0;

    AP_J1939_CAN* j1939 = AP_J1939_CAN::get_instance(_can_port);

    if (!j1939->register_driver(SSMBATTERY_QUERY_FRAME_FRAME_ID, this) ||
        !j1939->register_driver(SSMBATTERY_CELL_VOLTAGE_INFORMATION_FRAME_ID, this) ||
        !j1939->register_driver(SSMBATTERY_CELL_TEMPERATURE_INFORMATION_FRAME_ID, this) ||
        !j1939->register_driver(SSMBATTERY_TOTAL_INFORMATION_0_FRAME_ID, this) ||
        !j1939->register_driver(SSMBATTERY_TOTAL_INFORMATION_1_FRAME_ID, this) ||
        !j1939->register_driver(SSMBATTERY_CELL_VOLTAGE_STATISTICAL_INFORMATION_FRAME_ID, this) ||
        !j1939->register_driver(SSMBATTERY_UNIT_TEMPERATURE_STATISTICAL_INFORMATION_FRAME_ID, this) ||
        !j1939->register_driver(SSMBATTERY_STATUS_INFORMATION_0_FRAME_ID, this) ||
        !j1939->register_driver(SSMBATTERY_STATUS_INFORMATION_1_FRAME_ID, this) ||
        !j1939->register_driver(SSMBATTERY_STATUS_INFORMATION_2_FRAME_ID, this) ||
        !j1939->register_driver(SSMBATTERY_HARDWARE_AND_BATTERY_FAILURE_INFORMATION_FRAME_ID, this) ||
        !j1939->register_driver(SSMBATTERY_CHARGING_INFORMATION_FRAME_ID, this) ||
        !j1939->register_driver(SSMBATTERY_LIMITING_FRAME_ID, this) ||
        !j1939->register_driver(SSMBATTERY_FAULT_FRAME_ID, this))
    {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SSM Battery: Failed to register with J1939");
        return;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SSM Battery: Registered with J1939 on CAN%d", static_cast<int>(_can_port));

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

#if AP_SSMBATTERY_DEBUG
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SSM Battery: Sent query frame");
#endif
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
    WITH_SEMAPHORE(_sem);

    if (msg.module == 1)
    {
        _internal_state.cell_voltages.cells[0] = msg.volt1;
        _internal_state.cell_voltages.cells[1] = msg.volt2;
        _internal_state.cell_voltages.cells[2] = msg.volt3;
    }
    else if (msg.module == 2)
    {
        _internal_state.cell_voltages.cells[3] = msg.volt1;
        _internal_state.cell_voltages.cells[4] = msg.volt2;
        _internal_state.cell_voltages.cells[5] = msg.volt3;
    }
    else if (msg.module == 3)
    {
        _internal_state.cell_voltages.cells[6] = msg.volt1;
        _internal_state.cell_voltages.cells[7] = msg.volt2;
        _internal_state.cell_voltages.cells[8] = msg.volt3;
    }
    else if (msg.module == 4)
    {
        _internal_state.cell_voltages.cells[9] = msg.volt1;
        _internal_state.cell_voltages.cells[10] = msg.volt2;
        _internal_state.cell_voltages.cells[11] = msg.volt3;
    }
    else if (msg.module == 5)
    {
        _internal_state.cell_voltages.cells[12] = msg.volt1;
        _internal_state.cell_voltages.cells[13] = msg.volt2;
    }

    _last_update_us = AP_HAL::micros();

}

void AP_BattMonitor_SSM::handle_cell_temperature_information(const struct ssmbattery_cell_temperature_information_t &msg)
{
    // Handle the cell temperature information message
}

void AP_BattMonitor_SSM::handle_total_information_0(const struct ssmbattery_total_information_0_t &msg)
{
    // Handle the total information 0 message
#if AP_SSMBATTERY_DEBUG
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SSM Battery: Voltage: %f, Current: %f, SOC: %d", float(msg.sum_v) / 10.0f, float(msg.curr) / 10.0f, msg.soc / 10);
#endif

    WITH_SEMAPHORE(_sem);
    _internal_state.voltage = float(msg.sum_v) / 10.0f;
    _internal_state.current_amps = -1 * (float(msg.curr) / 10.0f - 3000.0f); //Flip sign to match BatteryMonitor convention
    _internal_state.last_time_micros = AP_HAL::micros();
    _capacity_remaining_pct = uint8_t(msg.soc / 10);
    _last_update_us = AP_HAL::micros();
}

void AP_BattMonitor_SSM::handle_total_information_1(const struct ssmbattery_total_information_1_t &msg)
{
    // Handle the total information 1 message
}

void AP_BattMonitor_SSM::handle_cell_voltage_statistical_information(const struct ssmbattery_cell_voltage_statistical_information_t &msg)
{
    // Handle the cell voltage statistical information message
}

void AP_BattMonitor_SSM::handle_unit_temperature_statistical_information(const struct ssmbattery_unit_temperature_statistical_information_t &msg)
{
    // Handle the unit temperature statistical information message
    WITH_SEMAPHORE(_sem);
    _internal_state.temperature = msg.max_t - 40;
    _internal_state.temperature_time = AP_HAL::micros();

    _last_update_us = AP_HAL::micros();
}

void AP_BattMonitor_SSM::handle_status_information_0(const struct ssmbattery_status_information_0_t &msg)
{
    // Handle the status information 0 message
}

void AP_BattMonitor_SSM::handle_status_information_1(const struct ssmbattery_status_information_1_t &msg)
{
    // Handle the status information 1 message
}

void AP_BattMonitor_SSM::handle_status_information_2(const struct ssmbattery_status_information_2_t &msg)
{
    // Handle the status information 2 message
}

void AP_BattMonitor_SSM::handle_hardware_and_battery_failure_information(const struct ssmbattery_hardware_and_battery_failure_information_t &msg)
{
    // Handle the hardware and battery failure information message
}

void AP_BattMonitor_SSM::handle_charging_information(const struct ssmbattery_charging_information_t &msg)
{
    // Handle the charging information message
}

void AP_BattMonitor_SSM::handle_limiting(const struct ssmbattery_limiting_t &msg)
{
    // Handle the limiting message
}

void AP_BattMonitor_SSM::handle_fault(const struct ssmbattery_fault_t &msg)
{
    // Handle the fault message
    if (msg.fault_bits)
    {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "SSM Battery: Page %d Fault %d", msg.page_no,msg.fault_bits);
    }
}

void AP_BattMonitor_SSM::split_id(uint32_t can_id, uint32_t& base_id, uint32_t& board_number) {
    base_id = can_id & 0xFFFFFF00;
    board_number = can_id & 0x000000FF;
}

#endif // AP_BATTERY_SSM_ENABLED
