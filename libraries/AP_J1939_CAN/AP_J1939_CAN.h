#pragma once

#include "AP_J1939_CAN_config.h"

#if HAL_J1939_CAN_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANSensor.h>
#include <map>
#include <vector>

// Diagnostic Message 1 PGN
#define J1939_PGN_DM1 0xFECA

namespace J1939 {
    struct J1939Frame {
        uint8_t priority;
        uint32_t pgn;
        uint8_t source_address;
        uint8_t data[8];
    };

    // Pack a J1939 frame into a CAN frame
    AP_HAL::CANFrame pack_j1939_frame(const J1939Frame &frame);

    // Unpack a J1939 frame from a CAN frame
    J1939Frame unpack_j1939_frame(const AP_HAL::CANFrame &frame);

    // Get the PGN from an extended CAN ID
    inline constexpr uint32_t extract_j1939_pgn(uint32_t ext_can_id)
    {
        return (ext_can_id >> 8) & 0xFFFF;
    }

    class DM1Frame {
        public:

        static uint32_t suspect_parameter_number(const uint8_t* pdu);
        static uint8_t failure_mode_identifier(const uint8_t* pdu);
    };
}

class AP_J1939_CAN : public CANSensor
{
public:
    static AP_J1939_CAN* get_instance(uint8_t can_port);

    // Register a frame ID to a driver (actually registers the PGN)
    bool register_frame_id(uint32_t can_id, CANSensor* driver);

    // Register a PGN to a driver
    bool register_pgn(uint32_t pgn, CANSensor* driver);

    // Send a J1939 frame
    bool send_message(J1939::J1939Frame &frame);

protected:
    // Handler for incoming frames
    void handle_frame(AP_HAL::CANFrame &frame) override;

private:
    explicit AP_J1939_CAN(uint8_t can_port);
    
    static std::map<uint8_t, AP_J1939_CAN*> _instances;
    std::map<uint32_t, std::vector<CANSensor*>> _msg_handlers;

    uint8_t _can_port;
};

#endif // HAL_J1939_CAN_ENABLED
