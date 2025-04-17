#include "AP_MarineICE_Backend_N2k.h"

#if HAL_MARINEICE_ENABLED

#include <AP_Math/AP_Math.h>
#include <algorithm>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

using namespace MarineICE::Types;

#define MARINEICE_DEBUG 0
#define MARINEICE_SIMULATE_MOTOR 1

#define MARINEICE_BACKEND_N2K_SOURCE_ADDRESS 0xEE // TODO: This should not be hard coded, but should be set by the NMEA2000 stack
#define MARINEICE_BACKEND_N2K_HEALTHY_TIMEOUT_MS 1000

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
        !j1939->register_frame_id(N2K_PGN_128267_WATER_DEPTH_FRAME_ID, this)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "MarineICE: Failed to register with J1939");
        return;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MarineICE: Registered with J1939 on CAN%d", _params.can_port.get());

#if MARINEICE_SIMULATE_MOTOR
    _state.engine_data.rpm = 1000.0; // Set a default RPM for w/o engine present
#endif
}

bool AP_MarineICE_Backend_N2k::healthy()
{
    // Check if we have received all messages in the healthy timeout
    const uint32_t now_ms = AP_HAL::millis();
    if ((_last_new_actuator_fb_msg_ms == 0 || now_ms - _last_new_actuator_fb_msg_ms > MARINEICE_BACKEND_N2K_HEALTHY_TIMEOUT_MS)
        || (_last_new_switch_bank_status_msg_ms == 0 || now_ms - _last_new_switch_bank_status_msg_ms > MARINEICE_BACKEND_N2K_HEALTHY_TIMEOUT_MS)
        || (_last_new_water_depth_msg_ms == 0 || now_ms - _last_new_water_depth_msg_ms > MARINEICE_BACKEND_N2K_HEALTHY_TIMEOUT_MS)) {
        // One or more messages have not been received in the healthy timeout
        return false;
    }
    return true;
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
    // TODO
}

void AP_MarineICE_Backend_N2k::set_cmd_ignition(bool enable)
{
    // TODO
}

void AP_MarineICE_Backend_N2k::set_cmd_starter(bool enable)
{
    // TODO
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
#if MARINEICE_DEBUG
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
#if MARINEICE_DEBUG
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "MarineICE: Received switch bank status: starter=%d, ignition=%d, trim_up=%d, trim_down=%d", msg.indicator2, msg.indicator3, msg.indicator12, msg.indicator11);
#endif
        break;
    }
    case J1939::extract_j1939_pgn(N2K_PGN_128267_WATER_DEPTH_FRAME_ID):
    {
        struct n2k_pgn_128267_water_depth_t msg;
        n2k_pgn_128267_water_depth_unpack(&msg, frame.data, frame.dlc);
        handle_pgn_128267_water_depth(msg);
        _last_new_water_depth_msg_ms = AP_HAL::millis();
#if MARINEICE_DEBUG
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
    _state.gear_position = static_cast<GearPosition>(msg.actual_gear_value);
    _state.throttle_pct = static_cast<float>(msg.actual_throttle_value) * 0.1f;
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

void AP_MarineICE_Backend_N2k::handle_pgn_128267_water_depth(const struct n2k_pgn_128267_water_depth_t &msg)
{
    _state.water_depth_m = static_cast<float>(msg.depth) * 0.01f;
}

#endif // HAL_MARINEICE_ENABLED
