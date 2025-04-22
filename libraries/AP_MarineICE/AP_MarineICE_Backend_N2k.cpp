#include "AP_MarineICE_Backend_N2k.h"

#if HAL_MARINEICE_ENABLED

#include <AP_Math/AP_Math.h>
#include <algorithm>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

using namespace MarineICE::Types;

#define MARINEICE_BACKEND_N2K_DEBUG 0

#define MARINEICE_BACKEND_N2K_SOURCE_ADDRESS 0xEE // TODO: This should not be hard coded, but should be set by the NMEA2000 stack
#define MARINEICE_BACKEND_N2K_HEALTHY_TIMEOUT_MS 2000

void AP_MarineICE_Backend_N2k::init()
{
    if(_params.can_port.get() < 0) {
        return;
    }
    j1939 = AP_J1939_CAN::get_instance(_params.can_port.get());

    if (j1939 == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "MarineICE: Failed to get J1939 instance");
        return;
    }
    // Register the driver with the J1939 CAN backend for the NMEA2000 PGNs

    if (!j1939->register_frame_id(N2K_PGN_65385_ACTUATOR_FEEDBACK_FRAME_ID, this) ||
        !j1939->register_frame_id(N2K_PGN_127501_BINARY_SWITCH_BANK_STATUS_FRAME_ID, this) ||
        !j1939->register_frame_id(N2K_PGN_127488_ENGINE_PARAMETERS_RAPID_UPDATE_FRAME_ID, this) ||
        !j1939->register_frame_id(N2K_PGN_127489_ENGINE_PARAMETERS_DYNAMIC_FRAME_ID, this) ||
        !j1939->register_frame_id(N2K_PGN_127493_TRANSMISSION_PARAMETERS_DYNAMIC_FRAME_ID, this) ||
        !j1939->register_frame_id(N2K_PGN_127497_TRIP_PARAMETERS_ENGINE_FRAME_ID, this) ||
        !j1939->register_frame_id(N2K_PGN_128267_WATER_DEPTH_FRAME_ID, this))
        {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "MarineICE: Failed to register with J1939");
        return;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MarineICE: Registered with J1939 on CAN%d", _params.can_port.get());

}

void AP_MarineICE_Backend_N2k::monitor_faults()
{
    // Check for engine overspeed
    if (_state.engine_data.rpm > _params.rpm_max.get())
    {
        set_fault(ENGINE_OVERSPEED, true);
    }

    // Check for engine overtemp
    if (_state.engine_data.temp_degc > _params.temp_max.get())
    {
        set_fault(ENGINE_OVERTEMP, true);
    }

    // TODO: Check for throttle actuator failure

    // TODO: Check for gear actuator failure

    // TODO: Check for alternator voltage low

    // Check for engine start attempts exceeded
    if (get_num_start_attempts() > _params.start_retries.get())
    {
        set_fault(ENGINE_START_ATTEMPTS_EXCEEDED, true);
    }

    // Check if we have received all messages in the healthy timeout
    const uint32_t now_ms = AP_HAL::millis();
    if (_last_new_actuator_fb_msg_ms == 0 || 
        now_ms - _last_new_actuator_fb_msg_ms > MARINEICE_BACKEND_N2K_HEALTHY_TIMEOUT_MS)
    {
        set_fault(NO_COMM_SHIFT_THROTTLE_ACTUATOR, true);
    }
    
    if (_last_new_switch_bank_status_msg_ms == 0 || 
        now_ms - _last_new_switch_bank_status_msg_ms > MARINEICE_BACKEND_N2K_HEALTHY_TIMEOUT_MS)
    {
        set_fault(NO_COMM_LOAD_CONTROLLER, true);
    }
    if (_last_new_engine_fb_msg_ms == 0 || 
        now_ms - _last_new_engine_fb_msg_ms > MARINEICE_BACKEND_N2K_HEALTHY_TIMEOUT_MS)
    {
        set_fault(NO_COMM_MOTOR, true);
    }

    if (_last_new_water_depth_msg_ms == 0 || 
        now_ms - _last_new_water_depth_msg_ms > MARINEICE_BACKEND_N2K_HEALTHY_TIMEOUT_MS)
    {
        set_fault(NO_COMM_DEPTH_SENSOR, true);
    }

}

void AP_MarineICE_Backend_N2k::set_cmd_shift_throttle(GearPosition gear, float throttle_pct)
{
    n2k_pgn_65380_actuator_command_port_engine_t msg;
    msg.manufacturer_id = 0X73A; // Dometic
    msg.reserved_1 = 3; // Magic number
    msg.industry_group = 4; // Marine
    msg.source_instance = 0; // Default
    msg.reserved_2 = 0;
    msg.reserved_3 = 0;
    msg.gear_command = static_cast<uint8_t>(gear);
    msg.reserved_4 = 0xF; // Magic number
    msg.throttle_command = static_cast<uint16_t>(abs(throttle_pct) * 10);
    msg.reserved_5 = 0;
    msg.reserved_6 = 0xFF; // Magic number
    send_pgn_65380_actuator_command_port_engine(msg);

    n2k_pgn_65390_control_head_feedback_t msg2;
    msg2.manufacturer_id = 0X73A; // Dometic
    msg2.reserved_field1 = 3; // Magic number
    msg2.industry_group = 4; // Marine
    msg2.source_instance = 0; // Default
    msg2.reserved_field2 = 0xF; // Magic number
    msg2.port_lever_gear_position = static_cast<uint8_t>(gear);
    msg2.stbd_lever_gear_position = 0xF; // Not available
    msg2.port_lever_throttle = static_cast<uint8_t>(abs(throttle_pct));
    msg2.stbd_lever_throttle = 0xFF; // Not available
    msg2.reserved_field3 = 0xFF; // Magic number
    msg2.danger_fault = 0;
    msg2.warning_fault = 0;
    msg2.ch_controlling = 1;
    msg2.port_ntw_active = 0;
    msg2.stbd_ntw_active = 0;
    msg2.sync_mode_active = 0;
    msg2.reserved_field4 = 0;
    send_pgn_65390_control_head_feedback(msg2);
}

void AP_MarineICE_Backend_N2k::set_cmd_trim(TrimCommand trim)
{
    // Set the header and load center bank instance
    n2k_pgn_126208_command_group_function_two_pair_bytes_t msg;
    msg.function_code = 1;
    msg.pgn_number = 127501L;
    msg.reserved = 0xF;
    msg.priority_setting = 8;
    msg.number_of_pairs = 2;
    msg.field_number_1 = 1;
    msg.field_value_1 = _params.lc_bank_instance;

    // Set the Trim Up Channel
    msg.field_number_2 = _params.lc_trim_up + 1;
    msg.field_value_2 = trim == TrimCommand::TRIM_UP ? 1 : 0;
    send_pgn_126208_command_group_function_two_pair_bytes(msg);

    // Set the Trim Down Channel
    msg.field_number_2 = _params.lc_trim_down + 1;
    msg.field_value_2 = trim == TrimCommand::TRIM_DOWN ? 1 : 0;
    send_pgn_126208_command_group_function_two_pair_bytes(msg);

}

void AP_MarineICE_Backend_N2k::set_cmd_ignition(bool enable)
{
    n2k_pgn_126208_command_group_function_two_pair_bytes_t msg;
    msg.function_code = 1;
    msg.pgn_number = 127501L;
    msg.reserved = 0xF;
    msg.priority_setting = 8;
    msg.number_of_pairs = 2;
    msg.field_number_1 = 1;
    msg.field_value_1 = _params.lc_bank_instance;
    msg.field_number_2 = _params.lc_ignition + 1;
    msg.field_value_2 = enable ? 1 : 0;
    send_pgn_126208_command_group_function_two_pair_bytes(msg);
}

void AP_MarineICE_Backend_N2k::set_cmd_starter(bool enable)
{
    n2k_pgn_126208_command_group_function_two_pair_bytes_t msg;
    msg.function_code = 1;
    msg.pgn_number = 127501L;
    msg.reserved = 0xF;
    msg.priority_setting = 8;
    msg.number_of_pairs = 2;
    msg.field_number_1 = 1;
    msg.field_value_1 = _params.lc_bank_instance;
    msg.field_number_2 = _params.lc_starter + 1;
    msg.field_value_2 = enable ? 1 : 0;
    send_pgn_126208_command_group_function_two_pair_bytes(msg);
}


bool AP_MarineICE_Backend_N2k::send_pgn_65380_actuator_command_port_engine(const struct n2k_pgn_65380_actuator_command_port_engine_t &msg)
{
    // Prepare the data
    uint8_t data[N2K_PGN_65380_ACTUATOR_COMMAND_PORT_ENGINE_LENGTH];
    n2k_pgn_65380_actuator_command_port_engine_pack(data, &msg, sizeof(data));

    // Even though the frame ID contains the priority and source address,
    // we set them explicitly here for clarity
    J1939::J1939Frame frame;
    frame.priority = 2;
    frame.pgn = J1939::extract_j1939_pgn(N2K_PGN_65380_ACTUATOR_COMMAND_PORT_ENGINE_FRAME_ID);
    frame.source_address = MARINEICE_BACKEND_N2K_SOURCE_ADDRESS;
    memcpy(frame.data, data, sizeof(data));

    return j1939->send_message(frame);
}

bool AP_MarineICE_Backend_N2k::send_pgn_65390_control_head_feedback(const struct n2k_pgn_65390_control_head_feedback_t &msg)
{
    // Prepare the data
    uint8_t data[N2K_PGN_65390_CONTROL_HEAD_FEEDBACK_LENGTH];
    n2k_pgn_65390_control_head_feedback_pack(data, &msg, sizeof(data));

    // Even though the frame ID contains the priority and source address,
    // we set them explicitly here for clarity
    J1939::J1939Frame frame;
    frame.priority = 5;
    frame.pgn = J1939::extract_j1939_pgn(N2K_PGN_65390_CONTROL_HEAD_FEEDBACK_FRAME_ID);
    frame.source_address = MARINEICE_BACKEND_N2K_SOURCE_ADDRESS;
    memcpy(frame.data, data, sizeof(data));

    return j1939->send_message(frame);
}

bool AP_MarineICE_Backend_N2k::send_pgn_126208_command_group_function_two_pair_bytes(const struct n2k_pgn_126208_command_group_function_two_pair_bytes_t &msg)
{
    // Prepare the data
    uint8_t data[N2K_PGN_126208_COMMAND_GROUP_FUNCTION_TWO_PAIR_BYTES_LENGTH];
    n2k_pgn_126208_command_group_function_two_pair_bytes_pack(data, &msg, sizeof(data));

    uint32_t pgn = N2K_PGN_126208_COMMAND_GROUP_FUNCTION_TWO_PAIR_BYTES_FRAME_ID;
    uint8_t priority = 3;
    // Send the message
    return send_nmea2000_fastpacket_message(pgn, priority, data, sizeof(data));

}

bool AP_MarineICE_Backend_N2k::send_nmea2000_fastpacket_message(uint32_t pgn, uint8_t priority, const uint8_t *data, size_t length)
{
    if (length <= 8) {
        // Single-frame message
        J1939::J1939Frame frame;
        frame.priority = priority;
        frame.pgn = J1939::extract_j1939_pgn(pgn);
        frame.source_address = MARINEICE_BACKEND_N2K_SOURCE_ADDRESS;
        memset(frame.data, 0xFF, sizeof(frame.data)); // Clear the frame data
        memcpy(frame.data, data, length);

        return j1939->send_message(frame);
    } else {
        // Fast-packet message
        size_t bytes_sent = 0;
        uint8_t sequence_id = 0;

        while (bytes_sent < length) {
            J1939::J1939Frame frame;
            frame.priority = priority;
            frame.pgn = J1939::extract_j1939_pgn(pgn);
            frame.source_address = MARINEICE_BACKEND_N2K_SOURCE_ADDRESS;
            memset(frame.data, 0xFF, sizeof(frame.data)); // Clear the frame data

            if (bytes_sent == 0) {
                // First frame (control frame)
                frame.data[0] = sequence_id++; // Sequence ID
                frame.data[1] = length;       // Total length of the message
                size_t chunk_size = std::min(static_cast<size_t>(6), length - bytes_sent);
                memcpy(frame.data + 2, data + bytes_sent, chunk_size);
                bytes_sent += chunk_size;
            } else {
                // Consecutive frames
                frame.data[0] = sequence_id++; // Sequence ID
                size_t chunk_size = std::min(static_cast<size_t>(7), length - bytes_sent);
                memcpy(frame.data + 1, data + bytes_sent, chunk_size);
                bytes_sent += chunk_size;
            }

            if (!j1939->send_message(frame)) {
                return false;
            }
        }
    }

    return true;
}

// bool AP_MarineICE_Backend_N2k::send_maretron_load_center(const struct maretron_load_center_t &msg)
// {}


// handler for incoming CAN frames
void AP_MarineICE_Backend_N2k::handle_frame(AP_HAL::CANFrame &frame)
{
    J1939::J1939Frame j1939_frame = J1939::unpack_j1939_frame(frame);

    switch (j1939_frame.pgn)
    {
    case J1939::extract_j1939_pgn(N2K_PGN_65385_ACTUATOR_FEEDBACK_FRAME_ID):
    {
        // Should be sent every 100ms
        struct n2k_pgn_65385_actuator_feedback_t msg;
        n2k_pgn_65385_actuator_feedback_unpack(&msg, frame.data, frame.dlc);
        handle_pgn_65385_actuator_feedback(msg);
        _last_new_actuator_fb_msg_ms = AP_HAL::millis();
#if MARINEICE_BACKEND_N2K_DEBUG
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "MarineICE: Received actuator feedback: gear=%d, throttle=%.1f", msg.actual_gear_value, msg.actual_throttle_value * 0.1f);
#endif
        break;
    }
    case J1939::extract_j1939_pgn(N2K_PGN_127501_BINARY_SWITCH_BANK_STATUS_FRAME_ID):
    {
        struct n2k_pgn_127501_binary_switch_bank_status_t msg;
        n2k_pgn_127501_binary_switch_bank_status_unpack(&msg, frame.data, frame.dlc);
        handle_pgn_127501_binary_switch_bank_status(msg);
        _last_new_switch_bank_status_msg_ms = AP_HAL::millis();
#if MARINEICE_BACKEND_N2K_DEBUG
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "MarineICE: Received switch bank status: starter=%d, ignition=%d, trim_up=%d, trim_down=%d", msg.indicator2, msg.indicator3, msg.indicator12, msg.indicator11);
#endif
        break;
    }
    case J1939::extract_j1939_pgn(N2K_PGN_127488_ENGINE_PARAMETERS_RAPID_UPDATE_FRAME_ID):
    {
        struct n2k_pgn_127488_engine_parameters_rapid_update_t msg;
        n2k_pgn_127488_engine_parameters_rapid_update_unpack(&msg, frame.data, frame.dlc);
        handle_pgn_127488_engine_parameters_rapid_update(msg);
        _last_new_engine_fb_msg_ms = AP_HAL::millis();
#if MARINEICE_BACKEND_N2K_DEBUG
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "MarineICE: Received engine parameters rapid update: rpm=%.1f, coolant_temp=%.1f", msg.rpm * 0.125f, msg.coolant_temp * 0.01f);
#endif
        break;
    }
    case J1939::extract_j1939_pgn(N2K_PGN_127489_ENGINE_PARAMETERS_DYNAMIC_FRAME_ID):
    {
        struct n2k_pgn_127489_engine_parameters_dynamic_t msg;
        n2k_pgn_127489_engine_parameters_dynamic_unpack(&msg, frame.data, frame.dlc);
        handle_pgn_127489_engine_parameters_dynamic(msg);
        _last_new_engine_fb_msg_ms = AP_HAL::millis();
#if MARINEICE_BACKEND_N2K_DEBUG
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "MarineICE: Received engine parameters dynamic: oil_pressure=%.1f, fuel_rate=%.1f", msg.oil_pressure * 0.01f, msg.fuel_rate * 0.01f);
#endif
        break;
    }
    case J1939::extract_j1939_pgn(N2K_PGN_127493_TRANSMISSION_PARAMETERS_DYNAMIC_FRAME_ID):
    {
        struct n2k_pgn_127493_transmission_parameters_dynamic_t msg;
        n2k_pgn_127493_transmission_parameters_dynamic_unpack(&msg, frame.data, frame.dlc);
        handle_pgn_127493_transmission_parameters_dynamic(msg);
        _last_new_engine_fb_msg_ms = AP_HAL::millis();
#if MARINEICE_BACKEND_N2K_DEBUG
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "MarineICE: Received transmission parameters dynamic: oil_temp=%.1f, oil_pressure=%.1f", msg.oil_temp * 0.01f, msg.oil_pressure * 0.01f);
#endif
        break;
    }
    case J1939::extract_j1939_pgn(N2K_PGN_127497_TRIP_PARAMETERS_ENGINE_FRAME_ID):
    {
        struct n2k_pgn_127497_trip_parameters_engine_t msg;
        n2k_pgn_127497_trip_parameters_engine_unpack(&msg, frame.data, frame.dlc);
        handle_pgn_127497_trip_parameters_engine(msg);
        _last_new_engine_fb_msg_ms = AP_HAL::millis();
#if MARINEICE_BACKEND_N2K_DEBUG
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "MarineICE: Received trip parameters engine: trip_time=%.1f, trip_distance=%.1f", msg.trip_time * 0.01f, msg.trip_distance * 0.01f);
#endif
        break;
    }
    case J1939::extract_j1939_pgn(N2K_PGN_128267_WATER_DEPTH_FRAME_ID):
    {
        struct n2k_pgn_128267_water_depth_t msg;
        n2k_pgn_128267_water_depth_unpack(&msg, frame.data, frame.dlc);
        handle_pgn_128267_water_depth(msg);
        _last_new_water_depth_msg_ms = AP_HAL::millis();
#if MARINEICE_BACKEND_N2K_DEBUG
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "MarineICE: Received water depth: %.2f m", msg.depth * 0.01f);
#endif
        break;
    }
    default:
        // Ignore other frames
        break;
    }
}

void AP_MarineICE_Backend_N2k::handle_pgn_65385_actuator_feedback(const struct n2k_pgn_65385_actuator_feedback_t &msg)
{
    _state.actuator_gear_position = static_cast<GearPosition>(msg.actual_gear_value);
    _state.actuator_throttle_pct = static_cast<float>(msg.actual_throttle_value) * 0.1f;
}

void AP_MarineICE_Backend_N2k::handle_pgn_127501_binary_switch_bank_status(const struct n2k_pgn_127501_binary_switch_bank_status_t &msg)
{
    _state.starter_on = msg.indicator2;
    _state.ignition_on = msg.indicator3;

    if (msg.indicator11)
    {
        // Trim down
        _state.trim_command = TrimCommand::TRIM_DOWN;
    }
    else if (msg.indicator12)
    {
        // Trim up
        _state.trim_command = TrimCommand::TRIM_DOWN;
    }
    else
    {
        // No trim command
        _state.trim_command = TrimCommand::TRIM_STOP;
    }
}

void AP_MarineICE_Backend_N2k::handle_pgn_127488_engine_parameters_rapid_update(const struct n2k_pgn_127488_engine_parameters_rapid_update_t &msg)
{
    _state.engine_data.rpm = static_cast<float>(msg.speed) * 0.25f;
    _state.engine_data.trim_pct = static_cast<float>(msg.tilt_trim);
}

void AP_MarineICE_Backend_N2k::handle_pgn_127489_engine_parameters_dynamic(const struct n2k_pgn_127489_engine_parameters_dynamic_t &msg)
{
    // TODO: This is NMEA2000 fast packet data
    // Get temperature, alternator voltage, total engine hours, and fuel rate
}

void AP_MarineICE_Backend_N2k::handle_pgn_127493_transmission_parameters_dynamic(const struct n2k_pgn_127493_transmission_parameters_dynamic_t &msg)
{
    _state.engine_data.transmission_gear = static_cast<GearPosition>(msg.transmission_gear);
}

void AP_MarineICE_Backend_N2k::handle_pgn_127497_trip_parameters_engine(const struct n2k_pgn_127497_trip_parameters_engine_t &msg)
{
    // TODO: This is NMEA2000 fast packet data
    // Get trip fuel used
}

void AP_MarineICE_Backend_N2k::handle_pgn_128267_water_depth(const struct n2k_pgn_128267_water_depth_t &msg)
{
    _state.water_depth_m = static_cast<float>(msg.depth) * 0.01f;
}

#endif // HAL_MARINEICE_ENABLED
