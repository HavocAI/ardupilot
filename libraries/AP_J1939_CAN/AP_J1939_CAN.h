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

    /// There are two different PDU formats. PDU1 format is used for sending messages
    /// with a specific destination address, while PDU2 format is used for broadcast messages.
    /// The PDU format byte in the identifier determines the message format. If the PDU format
    /// byte is less than 240, it indicates PDU1 format, and if it is greater than 239, it is PDU2.
    enum class PDUFormat {
        PDU1 = 0, // PDU format 1
        PDU2 = 1, // PDU format 2
    };

    enum class PGNType {
        TransportProtocolConnectionManagement,
        TransportProtocolDataTransfer,
        DiagnosticMessage1,
        Other,
    };

    class PGN {
        public:
            PGN() : pgn(0) {}
            PGN(uint32_t v) : pgn(v) {}

            PGNType type() const;
            uint32_t raw() const { return pgn; }

        private:
            uint32_t pgn;
    };

    class Id {
        public:
            Id(uint32_t v) : id(v) {}

            uint8_t priority() const;

            uint8_t data_page() const;

            uint32_t pgn_raw() const;
            PGN pgn() const;

            PDUFormat pdu_format() const;

            uint8_t source_address() const;

            uint8_t pdu_specific() const;

        private:
            uint32_t id;
    };

    #define J1939_BROADCAST_MAX_LEN 70

    class BroadcastTransport {
        public:

        bool from_frame(const AP_HAL::CANFrame &frame);
        uint8_t packet_count() const;
        uint16_t data_len() const { return data_length; }
        const uint8_t* data_ptr() const { return data; }
        PGN get_pgn() const { return pgn; }

        private:
            uint8_t source_address;
            PGN pgn;
            uint16_t data_length;
            uint8_t data[J1939_BROADCAST_MAX_LEN];

            enum class State {
                ConnectionManagement,
                DataTransfer,
            } state;
            
    };


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

        static bool red_stop_lamp(const uint8_t* pdu);
        static bool amber_warning_lamp(const uint8_t* pdu);
        static bool protect_lamp(const uint8_t* pdu);

        static uint32_t suspect_parameter_number(const uint8_t* pdu);
        static uint8_t failure_mode_identifier(const uint8_t* pdu);
    };

    class DiagnosticMessage1 {
        public:

        class LampStatus {
            public:
            static LampStatus from_data(const uint8_t*);

            bool red_stop_lamp() const { return data[0] & 0x01; }
            bool amber_warning_lamp() const { return data[0] & 0x02; }
            bool protect_lamp() const { return data[0] & 0x04; }

            private:
                uint8_t data[2];
        };

        class DTC {
            public:
                static DTC from_data(const uint8_t* data);

                uint32_t spn() const;
                uint8_t fmi() const;
                uint8_t oc() const;
                bool cm() const;

            private:
                uint8_t data[4];
        };

        DiagnosticMessage1(const AP_HAL::CANFrame &frame);
        DiagnosticMessage1(const DTC&, const LampStatus&);

        const LampStatus& get_lamp_status() const { return lamp_status; }
        const DTC& get_dtc() const { return dtc; }

        private:
            LampStatus lamp_status;
            DTC dtc;

    }; 
}

class AP_J1939_CAN : public CANSensor
{
public:

    // typedef std::function<void(const J1939::BroadcastTransport&)> TransportCallback;
    // typedef std::function<void(const J1939::DiagnosticMessage1&)> DiagnosticMessage1Callback;

    FUNCTOR_TYPEDEF(TransportCallback, void, const J1939::BroadcastTransport&);
    FUNCTOR_TYPEDEF(DiagnosticMessage1Callback, void, const J1939::DiagnosticMessage1&);

    static AP_J1939_CAN* get_instance(uint8_t can_port);

    // Register a frame ID to a driver (actually registers the PGN)
    bool register_frame_id(uint32_t can_id, CANSensor* driver);

    // Register a PGN to a driver
    bool register_pgn(uint32_t pgn, CANSensor* driver);

    // Send a J1939 frame
    bool send_message(J1939::J1939Frame &frame);

    TransportCallback on_transport_callback;
    DiagnosticMessage1Callback on_diagnostic_message1_callback;

protected:
    // Handler for incoming frames
    void handle_frame(AP_HAL::CANFrame &frame) override;

private:
    explicit AP_J1939_CAN(uint8_t can_port);
    
    static std::map<uint8_t, AP_J1939_CAN*> _instances;
    std::map<uint32_t, std::vector<CANSensor*>> _msg_handlers;

    uint8_t _can_port;
    J1939::BroadcastTransport _broadcast_transport;

    void handle_dm1_transport();
};

#endif // HAL_J1939_CAN_ENABLED
