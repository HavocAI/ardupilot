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

    // check for fault conditions and set _status.faults
    void monitor_faults() override;

    void set_cmd_shift_throttle(GearPosition gear, float throttle_pct) override;
    void set_cmd_trim(TrimCommand trim) override;
    void set_cmd_ignition(bool enable) override;
    void set_cmd_starter(bool enable) override;

    // J1939 CAN backend
    AP_J1939_CAN* j1939;

private:

    // senders for NMEA2000 PGNs
    bool send_pgn_65380_actuator_command_port_engine(const struct n2k_pgn_65380_actuator_command_port_engine_t &msg);
    bool send_pgn_65390_control_head_feedback(const struct n2k_pgn_65390_control_head_feedback_t &msg);
    bool send_pgn_126208_command_group_function_two_pair_bytes(const struct n2k_pgn_126208_command_group_function_two_pair_bytes_t &msg);

    // send NMEA2000 fast packet message
    // PGN 126208 is a fast packet message, so we need to send it in chunks
    // of 8 bytes at a time. 
    // TODO: This should be part of the NMEA2000 stack
    bool send_nmea2000_fastpacket_message(uint32_t pgn, uint8_t priority, const uint8_t *data, size_t length);

    // handler for incoming CAN frames
    void handle_frame(AP_HAL::CANFrame &frame) override;

    // handle Dometic actuator feedback
    void handle_pgn_65385_actuator_feedback(const struct n2k_pgn_65385_actuator_feedback_t &msg);

    // handle Maretron load center feedback
    void handle_pgn_127501_binary_switch_bank_status(const struct n2k_pgn_127501_binary_switch_bank_status_t &msg);

    // handle Suzuki engine feedback
    void handle_pgn_127488_engine_parameters_rapid_update(const struct n2k_pgn_127488_engine_parameters_rapid_update_t &msg);
    void handle_pgn_127489_engine_parameters_dynamic(const struct n2k_pgn_127489_engine_parameters_dynamic_t &msg);
    void handle_pgn_127493_transmission_parameters_dynamic(const struct n2k_pgn_127493_transmission_parameters_dynamic_t &msg);
    void handle_pgn_127497_trip_parameters_engine(const struct n2k_pgn_127497_trip_parameters_engine_t &msg);

    // handle depth sensor feedback
    void handle_pgn_128267_water_depth(const struct n2k_pgn_128267_water_depth_t &msg);

    // Time last subscribed PGN was received
    uint32_t _last_new_actuator_fb_msg_ms;
    uint32_t _last_new_switch_bank_status_msg_ms;
    uint32_t _last_new_engine_fb_msg_ms;
    uint32_t _last_new_water_depth_msg_ms;

};

#endif // HAL_MARINEICE_ENABLED
