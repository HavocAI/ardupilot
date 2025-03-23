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
    (See AP_J1939_CAN/AP_J1939_CAN.cpp for more information)
    Then configure the SSM Battery parameters:
    - SSM_CANPRT: CAN Port that the SSM battery is connected to (0-indexed), -1 to disable
    - SSM_BRDNUM: Battery board number to use for this monitor, by default this is set to 0
        Different batteries may have different board numbers
    - SSM_CELLS: Number of cells in the battery, by default this is set to 14
        This is used to calculate the total voltage of the battery. SSM batteries all use the same BMS,
        so this value needs to be manually set.

    Telemetry Outputs:
    - Battery Cell Voltages
    - Battery Current
    - Battery Remaining Capacity
    - Battery Temperature
    - Battery Faults

*/
#include "AP_BattMonitor_config.h"

#if AP_BATTERY_SSM_ENABLED

#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/utility/sparse-endian.h>

#include "AP_BattMonitor_SSM.h"

extern const AP_HAL::HAL &hal;

#define AP_BATT_MONITOR_SSM_DEBUG 0
#define AP_BATT_MONITOR_SSM_QUERY_INTERVAL_MS 2000
#define AP_BATT_MONITOR_SSM_TIMEOUT_US 5000000
// Priority and source address for querying the battery
#define AP_BATT_MONITOR_SSM_PRIORITY 1
#define AP_BATT_MONITOR_SSM_SOURCE_ADDRESS 0x80

const AP_Param::GroupInfo AP_BattMonitor_SSM::var_info[] = {

    // Using index 70-75 for the SSM Battery parameters to deconflict with the AP_BattMonitor parameters

    // @Param: CANPRT
    // @DisplayName: CAN Port
    // @Description: CAN Port, by default this is set to 0
    // @Values: 0:1
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("SSM_CANPRT", 70, AP_BattMonitor_SSM, _can_port, -1),

    // @Param: BRDNUM
    // @DisplayName: Board Number
    // @Description: Battery board number to use for this monitor, by default this is set to 0
    // @Values: 0:255
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("SSM_BRDNUM", 71, AP_BattMonitor_SSM, _board_number, 0),

    // @Param: CELLS
    // @DisplayName: Number of Cells
    // @Description: Number of cells in the battery, by default this is set to 14
    // @Values: 1:255
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("SSM_CELLS", 72, AP_BattMonitor_SSM, _num_cells, 14),

    AP_GROUPEND};

// Constructor
AP_BattMonitor_SSM::AP_BattMonitor_SSM(AP_BattMonitor &mon, 
                                       AP_BattMonitor::BattMonitor_State &mon_state, 
                                       AP_BattMonitor_Params &params)
    : AP_BattMonitor_Backend(mon, mon_state, params), CANSensor("SSMBattery")
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
}

void AP_BattMonitor_SSM::init(void)
{
    _state.healthy = false;

    if (_can_port.get() < 0)
    {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SSM Battery: Disabled (CAN port = -1)");
        return;
    }

    AP_J1939_CAN* j1939 = AP_J1939_CAN::get_instance(_can_port.get());

    if (j1939 == nullptr)
    {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SSM Battery: Failed to get J1939 instance");
        return;
    }

    if (!j1939->register_frame_id(SSMBATTERY_QUERY_FRAME_FRAME_ID, this) ||
        !j1939->register_frame_id(SSMBATTERY_CELL_VOLTAGE_INFORMATION_FRAME_ID, this) ||
        !j1939->register_frame_id(SSMBATTERY_CELL_TEMPERATURE_INFORMATION_FRAME_ID, this) ||
        !j1939->register_frame_id(SSMBATTERY_TOTAL_INFORMATION_0_FRAME_ID, this) ||
        !j1939->register_frame_id(SSMBATTERY_TOTAL_INFORMATION_1_FRAME_ID, this) ||
        !j1939->register_frame_id(SSMBATTERY_CELL_VOLTAGE_STATISTICAL_INFORMATION_FRAME_ID, this) ||
        !j1939->register_frame_id(SSMBATTERY_UNIT_TEMPERATURE_STATISTICAL_INFORMATION_FRAME_ID, this) ||
        !j1939->register_frame_id(SSMBATTERY_STATUS_INFORMATION_0_FRAME_ID, this) ||
        !j1939->register_frame_id(SSMBATTERY_STATUS_INFORMATION_1_FRAME_ID, this) ||
        !j1939->register_frame_id(SSMBATTERY_STATUS_INFORMATION_2_FRAME_ID, this) ||
        !j1939->register_frame_id(SSMBATTERY_HARDWARE_AND_BATTERY_FAILURE_INFORMATION_FRAME_ID, this) ||
        !j1939->register_frame_id(SSMBATTERY_CHARGING_INFORMATION_FRAME_ID, this) ||
        !j1939->register_frame_id(SSMBATTERY_LIMITING_FRAME_ID, this) ||
        !j1939->register_frame_id(SSMBATTERY_FAULT_FRAME_ID, this))
    {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SSM Battery: Failed to register with J1939");
        return;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SSM Battery: Registered with J1939 on CAN%d", _can_port.get());

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

    for (uint8_t i = 0; i < MIN(AP_BATT_MONITOR_CELLS_MAX,_num_cells.get()); i++) {
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
    // Only handle extended frames for the "board number"
    // aka. source address that this monitor is configured for
    J1939::J1939Frame j1939_frame = J1939::unpack_j1939_frame(frame);
    if (j1939_frame.source_address != _board_number) {
        return;
    }

    // Use only the J1939 PGN of the frame id to determine the message type
    switch (j1939_frame.pgn)
    {
    case J1939::extract_j1939_pgn(SSMBATTERY_QUERY_FRAME_FRAME_ID):
    {
        struct ssmbattery_query_frame_t msg;
        ssmbattery_query_frame_unpack(&msg, frame.data, frame.dlc);
        handle_query_frame(msg);
        break;
    }
    case J1939::extract_j1939_pgn(SSMBATTERY_CELL_VOLTAGE_INFORMATION_FRAME_ID):
    {
        struct ssmbattery_cell_voltage_information_t msg;
        ssmbattery_cell_voltage_information_unpack(&msg, frame.data, frame.dlc);
        handle_cell_voltage_information(msg);
        break;
    }
    case J1939::extract_j1939_pgn(SSMBATTERY_CELL_TEMPERATURE_INFORMATION_FRAME_ID):
    {
        struct ssmbattery_cell_temperature_information_t msg;
        ssmbattery_cell_temperature_information_unpack(&msg, frame.data, frame.dlc);
        handle_cell_temperature_information(msg);
        break;
    }
    case J1939::extract_j1939_pgn(SSMBATTERY_TOTAL_INFORMATION_0_FRAME_ID):
    {
        struct ssmbattery_total_information_0_t msg;
        ssmbattery_total_information_0_unpack(&msg, frame.data, frame.dlc);
        handle_total_information_0(msg);
        break;
    }
    case J1939::extract_j1939_pgn(SSMBATTERY_TOTAL_INFORMATION_1_FRAME_ID):
    {
        struct ssmbattery_total_information_1_t msg;
        ssmbattery_total_information_1_unpack(&msg, frame.data, frame.dlc);
        handle_total_information_1(msg);
        break;
    }
    case J1939::extract_j1939_pgn(SSMBATTERY_CELL_VOLTAGE_STATISTICAL_INFORMATION_FRAME_ID):
    {
        struct ssmbattery_cell_voltage_statistical_information_t msg;
        ssmbattery_cell_voltage_statistical_information_unpack(&msg, frame.data, frame.dlc);
        handle_cell_voltage_statistical_information(msg);
        break;
    }
    case J1939::extract_j1939_pgn(SSMBATTERY_UNIT_TEMPERATURE_STATISTICAL_INFORMATION_FRAME_ID):
    {
        struct ssmbattery_unit_temperature_statistical_information_t msg;
        ssmbattery_unit_temperature_statistical_information_unpack(&msg, frame.data, frame.dlc);
        handle_unit_temperature_statistical_information(msg);
        break;
    }
    case J1939::extract_j1939_pgn(SSMBATTERY_STATUS_INFORMATION_0_FRAME_ID):
    {
        struct ssmbattery_status_information_0_t msg;
        ssmbattery_status_information_0_unpack(&msg, frame.data, frame.dlc);
        handle_status_information_0(msg);
        break;
    }
    case J1939::extract_j1939_pgn(SSMBATTERY_STATUS_INFORMATION_1_FRAME_ID):
    {
        struct ssmbattery_status_information_1_t msg;
        ssmbattery_status_information_1_unpack(&msg, frame.data, frame.dlc);
        handle_status_information_1(msg);
        break;
    }
    case J1939::extract_j1939_pgn(SSMBATTERY_STATUS_INFORMATION_2_FRAME_ID):
    {
        struct ssmbattery_status_information_2_t msg;
        ssmbattery_status_information_2_unpack(&msg, frame.data, frame.dlc);
        handle_status_information_2(msg);
        break;
    }
    case J1939::extract_j1939_pgn(SSMBATTERY_HARDWARE_AND_BATTERY_FAILURE_INFORMATION_FRAME_ID):
    {
        struct ssmbattery_hardware_and_battery_failure_information_t msg;
        ssmbattery_hardware_and_battery_failure_information_unpack(&msg, frame.data, frame.dlc);
        handle_hardware_and_battery_failure_information(msg);
        break;
    }
    case J1939::extract_j1939_pgn(SSMBATTERY_CHARGING_INFORMATION_FRAME_ID):
    {
        struct ssmbattery_charging_information_t msg;
        ssmbattery_charging_information_unpack(&msg, frame.data, frame.dlc);
        handle_charging_information(msg);
        break;
    }
    case J1939::extract_j1939_pgn(SSMBATTERY_LIMITING_FRAME_ID):
    {
        struct ssmbattery_limiting_t msg;
        ssmbattery_limiting_unpack(&msg, frame.data, frame.dlc);
        handle_limiting(msg);
        break;
    }
    case J1939::extract_j1939_pgn(SSMBATTERY_FAULT_FRAME_ID):
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
    while (true)
    {
        hal.scheduler->delay_microseconds(20000); // 50Hz

        const uint32_t now_ms = AP_HAL::millis();

        // Send query frame every 2 seconds
        if (now_ms - last_query_time >= AP_BATT_MONITOR_SSM_QUERY_INTERVAL_MS) {
            send_query_frame();
            last_query_time = now_ms;
        }

    } // while true
}

void AP_BattMonitor_SSM::send_query_frame()
{
    // Prepare the data for the query frame (all zeros)
    struct ssmbattery_query_frame_t query_msg;
    ssmbattery_query_frame_init(&query_msg);
    uint8_t data[SSMBATTERY_QUERY_FRAME_LENGTH];
    ssmbattery_query_frame_pack(data, &query_msg, sizeof(data));

    // Even though the frame ID contains the priority and source address,
    // we set them explicitly here for clarity

    J1939::J1939Frame frame;
    frame.priority = AP_BATT_MONITOR_SSM_PRIORITY;
    frame.pgn = J1939::extract_j1939_pgn(SSMBATTERY_QUERY_FRAME_FRAME_ID);
    frame.source_address = AP_BATT_MONITOR_SSM_SOURCE_ADDRESS;
    memcpy(frame.data, data, sizeof(data));

    AP_J1939_CAN::get_instance(_can_port.get())->
        send_message(frame);

#if AP_BATT_MONITOR_SSM_DEBUG
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SSM Battery: Sent query frame");
#endif
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
#if AP_BATT_MONITOR_SSM_DEBUG
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
