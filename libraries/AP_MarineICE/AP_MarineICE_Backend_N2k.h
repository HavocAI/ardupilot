#pragma once

#include "AP_MarineICE_config.h"

#if HAL_MARINEICE_ENABLED
#include "AP_MarineICE_Backend.h"
#include <AP_CANManager/AP_CANSensor.h>
#include <AP_J1939_CAN/AP_J1939_CAN.h>
#include "n2k.h"

class AP_MarineICE_Backend_N2k : public AP_MarineICE_Backend, public CANSensor
{
public:
    // constructor
    AP_MarineICE_Backend_N2k(AP_MarineICE_Params &params) :
        AP_MarineICE_Backend(params),
        CANSensor("MarineICE_Backend_N2k")
    {};

    // do not allow copies
    CLASS_NO_COPY(AP_MarineICE_Backend_N2k);

    // initialise driver
    void init() override;

    // returns true if communicating with all required interfaces
    bool healthy() override;

    void set_cmd_shift_throttle(GearPosition gear, float throttle_pct) override;
    void set_cmd_trim(TrimCommand trim) override;
    void set_cmd_ignition(bool enable) override;
    void set_cmd_starter(bool enable) override;

    // J1939 CAN backend
    AP_J1939_CAN* j1939;

private:

    bool send_pgn_65380_actuator_command_port_engine(const struct n2k_pgn_65380_actuator_command_port_engine_t &msg);
    bool send_pgn_65390_control_head_feedback(const struct n2k_pgn_65390_control_head_feedback_t &msg);
    
    // TODO: Create PGN 126208
    // bool send_maretron_load_center(const struct maretron_load_center_t &msg);

    // handler for incoming CAN frames
    void handle_frame(AP_HAL::CANFrame &frame) override;

    void handle_pgn_65385_actuator_feedback(const struct n2k_pgn_65385_actuator_feedback_t &msg);
    void handle_pgn_127501_binary_switch_bank_status(const struct n2k_pgn_127501_binary_switch_bank_status_t &msg);
    // TODO: Additional handlers for Suzuki NMEA2000 engine data    
    void handle_pgn_128267_water_depth(const struct n2k_pgn_128267_water_depth_t &msg);

    // Time last subscribed PGN was received
    uint32_t _last_new_actuator_fb_msg_ms;
    uint32_t _last_new_switch_bank_status_msg_ms;
    uint32_t _last_new_water_depth_msg_ms;

};

#endif // HAL_MARINEICE_ENABLED
