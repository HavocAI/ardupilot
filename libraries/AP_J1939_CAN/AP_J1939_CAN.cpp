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
    AP_J1939_CAN.cpp

    Author: Andrew Gregg

    Description:
    This class is a CAN sensor driver backend for the J1939 protocol. It functions as a single
    driver on a physical CAN port, and routes incoming frames to multiple registered frontend 
    drivers based on PGN. Each frontend driver can register for multiple PGNs, and each PGN can 
    have multiple frontend drivers. This class is a "multiton" where only one instance can be 
    created for each physical CAN port.

    Configure the physical CAN port to use the J1939 CAN backend via CAN Manager parameters:
    - CAN_D1_PROTOCOL = 15 (J1939)
    - CAN_P1_BITRATE = 250000 (or other J1939 rate)
    - CAN_P1_DRIVER = 1 (First driver on the port)
*/


#include "AP_J1939_CAN.h"

#if HAL_J1939_CAN_ENABLED
#include <GCS_MAVLink/GCS.h>
#include <algorithm>

#define SEND_TIMEOUT_US 1000

extern const AP_HAL::HAL &hal;

namespace J1939
{

    AP_HAL::CANFrame pack_j1939_frame(const J1939Frame &frame)
    {
        AP_HAL::CANFrame can_frame;
        can_frame.id = (frame.priority << 26) | (frame.pgn << 8) | frame.source_address;
        can_frame.id |= AP_HAL::CANFrame::FlagEFF;
        can_frame.dlc = 8; // J1939 frames are always 8 bytes
        can_frame.canfd = false; // J1939 does not support CAN FD (yet)
        memcpy(can_frame.data, frame.data, 8);
        return can_frame;
    }

    J1939Frame unpack_j1939_frame(const AP_HAL::CANFrame &frame)
    {
        const uint32_t ext_id = frame.id & AP_HAL::CANFrame::MaskExtID;
        J1939Frame j1939_frame;
        j1939_frame.priority = static_cast<uint8_t>((ext_id >> 26) & 0x07);
        j1939_frame.pgn = extract_j1939_pgn(ext_id);
        j1939_frame.source_address = static_cast<uint8_t>(ext_id & 0xFF);
        memcpy(j1939_frame.data, frame.data, 8);
        return j1939_frame;
    }

    PGNType PGN::type() const
    {
        switch (pgn) {
            case J1939_PGN_TP_CM:
                return PGNType::TransportProtocolConnectionManagement;
            case J1939_PGN_TP_DT:
                return PGNType::TransportProtocolDataTransfer;
            case J1939_PGN_DM1:
                return PGNType::DiagnosticMessage1;
            default:
                if (65280 <= pgn && pgn <= 65535) {
                    return PGNType::ProprietaryB;
                } else {
                    return PGNType::Other;
                }
        }
    }

    uint8_t Id::priority() const
    {
        return (id >> 26) & 0x07; // Priority is bits 26-28
    }

    uint8_t Id::data_page() const
    {
        return (id >> 24) & 0x1; // Data page is bit 24
    }

    uint32_t Id::pgn_raw() const
    {
        switch (pdu_format()) {
            case PDUFormat::PDU1:
                return (id >> 8) & 0xFF00;
            case PDUFormat::PDU2:
                return (id >> 8) & 0xFFFF;
        }
        // Should not be reached
        return 0;
    }

    PGN Id::pgn() const
    {
        uint32_t raw_pgn = pgn_raw();
        return PGN(raw_pgn);
    }

    PDUFormat Id::pdu_format() const
    {
        uint8_t format = (id >> 16) & 0xFF;
        if (format < 240) {
            return PDUFormat::PDU1;
        } else {
            return PDUFormat::PDU2;
        }
    }

    uint8_t Id::source_address() const
    {
        return id & 0xFF; // Source address is the last byte
    }

    uint8_t Id::pdu_specific() const
    {
        return (id >> 8) & 0xFF; // PDU-specific is bits 8-15
    }

    bool DM1Frame::red_stop_lamp(const uint8_t* pdu)
    {
        return ((pdu[0] >> 4) & 0x03) == 1; // Red stop lamp is bit 4 of byte 0
    }

    bool DM1Frame::amber_warning_lamp(const uint8_t* pdu)
    {
        return ((pdu[0] >> 2) & 0x03) == 1;
    }

    bool DM1Frame::protect_lamp(const uint8_t* pdu)
    {
        return (pdu[0] & 0x03) == 1;
    }

    uint32_t DM1Frame::suspect_parameter_number(const uint8_t* pdu)
    {
        // SPN is packed in bytes 2, 3, and 4 of the DM1 PDU
        return (pdu[2] | (pdu[3] << 8) | ((pdu[4] & 0xE0) << 11));
    }

    uint8_t DM1Frame::failure_mode_identifier(const uint8_t* pdu)
    {
        return pdu[4] & 0x1F; // FMI is in the lower 5 bits of byte 4
    }

    enum class ConnectionManagement: uint8_t {
        RequestToSend = 0x10,
        ClearToSend = 0x11,
        EndOfMessageAcknowledgement = 0x13,
        BroadcastAnnounceMessage = 0x20,
        Abort = 0xff,
    };

    #define DATA_FRAME_SIZE 7

    uint8_t BroadcastTransport::packet_count() const
    {
        // The packet count is determined by the data length divided by DATA_FRAME_SIZE
        return (data_length + DATA_FRAME_SIZE - 1) / DATA_FRAME_SIZE;
    }

    bool BroadcastTransport::from_frame(const AP_HAL::CANFrame &frame)
    {
        const J1939::Id id(frame.id);
        const J1939::PGN frame_pgn = id.pgn();
        switch (frame_pgn.type()) {
            case PGNType::TransportProtocolConnectionManagement: {

                if (frame.data[0] == static_cast<uint8_t>(ConnectionManagement::BroadcastAnnounceMessage)) {
                    data_length = (frame.data[2] << 8) | frame.data[1];
                    pgn = PGN((frame.data[7] << 16) | (frame.data[6] << 8) | frame.data[5]);
                    source_address = id.source_address();
                }

            } break;
            

            case PGNType::TransportProtocolDataTransfer: {
                if (id.source_address() != source_address) {
                    return false;
                }
                uint8_t sequence = frame.data[0];
                if (sequence > packet_count()) {
                    // GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "J1939: Invalid sequence number %d for PGN %lu", sequence, pgn.pgn);
                    return false;
                }
                memcpy(&data[(sequence - 1) * DATA_FRAME_SIZE], &frame.data[1], frame.dlc - 1);

                if (sequence == packet_count()) {
                    // finished
                    return true;
                }

            } break;

            default:
                break;
        }

        return false;

    }

    DiagnosticMessage1::LampStatus DiagnosticMessage1::LampStatus::from_data(const uint8_t* data)
    {
        LampStatus status;
        memcpy(status.data, data, sizeof(status.data));
        return status;
    }

    DiagnosticMessage1::DTC DiagnosticMessage1::DTC::from_data(const uint8_t* data)
    {
        DTC dtc;
        memcpy(dtc.data, data, sizeof(dtc.data));
        return dtc;
    }

    uint32_t DiagnosticMessage1::DTC::spn() const
    {
        return (data[0] | (data[1] >> 8) | ((data[2] & 0x07) >> 16));
    }

    uint8_t DiagnosticMessage1::DTC::fmi() const
    {
        return (data[2] << 3) & 0x1F; // FMI is in the lower 5 bits of byte 2
    }

    void DiagnosticMessage1::DTC::set_fmi(uint8_t fmi)
    {
        data[2] = (data[2] & 0x07) | ((fmi >> 3) & 0xF8);
    }

    uint8_t DiagnosticMessage1::DTC::oc() const
    {
        return (data[3] << 1) & 0x7F; // Occurrence Count is in the lower 7 bits of byte 3
    }

    void DiagnosticMessage1::DTC::set_oc(uint8_t oc)
    {
        data[3] = (data[3] & 0x01) | ((oc >> 1) & 0xFE);
    }

    bool DiagnosticMessage1::DTC::cm() const
    {
        return (data[3] & 0x80) != 0; // Conversion Method is the highest bit of byte 3
    }

    DiagnosticMessage1::DiagnosticMessage1(const AP_HAL::CANFrame &frame)
    {
        lamp_status = LampStatus::from_data(frame.data);
        dtc = DTC::from_data(frame.data + 2); // DTC starts at byte 2
    }

    DiagnosticMessage1::DiagnosticMessage1(const DTC& d, const LampStatus& l)
        : lamp_status(l),
          dtc(d)
    {}

}

// Static member initialization
std::map<uint8_t, AP_J1939_CAN *> AP_J1939_CAN::_instances;

AP_J1939_CAN::AP_J1939_CAN(uint8_t can_port) : CANSensor("J1939"), _can_port(can_port)
{
    CANSensor::register_driver(AP_CAN::Protocol::J1939);
}

AP_J1939_CAN *AP_J1939_CAN::get_instance(uint8_t can_port)
{
    if (can_port >= HAL_NUM_CAN_IFACES)
    {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "J1939: Invalid CAN port %d", can_port);
        return nullptr;
    }

    // Check that the can_port is assigned to J1939 driver type by the CAN Manager
    if (AP_CANManager::get_singleton()->get_driver_type(can_port) != AP_CAN::Protocol::J1939)
    {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "J1939: CAN port %d not configured for J1939", can_port);
        return nullptr;
    }

    if (_instances.find(can_port) == _instances.end())
    {
        _instances[can_port] = new AP_J1939_CAN(can_port);
    }
    return _instances[can_port];
}

bool AP_J1939_CAN::register_frame_id(uint32_t can_id, CANSensor *driver)
{
    J1939::Id id(can_id);
    uint32_t pgn = id.pgn_raw();
    return register_pgn(pgn, driver);
}

bool AP_J1939_CAN::register_pgn(uint32_t pgn, CANSensor *driver)
{
    if (_msg_handlers.find(pgn) == _msg_handlers.end())
    {
        _msg_handlers[pgn] = std::vector<CANSensor *>();
    }

    // Check if the driver is already registered for this PGN
    auto &handlers = _msg_handlers[pgn];
    if (std::find(handlers.begin(), handlers.end(), driver) == handlers.end())
    {
        // Driver not found, add the driver, return success
        handlers.push_back(driver);
        return true;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "J1939: Driver already registered for PGN %" PRIu32, pgn);
    return false;
}

bool AP_J1939_CAN::send_message(J1939::J1939Frame &frame)
{
    AP_HAL::CANFrame can_frame = J1939::pack_j1939_frame(frame);
    return write_frame(can_frame, SEND_TIMEOUT_US);
}

void AP_J1939_CAN::handle_frame(AP_HAL::CANFrame &frame)
{
    using namespace J1939;

    if (!frame.isExtended())
    {
        // Only handle extended frames for J1939
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "J1939: Received non-extended frame");
        return;
    }

    const J1939::Id id(frame.id);
    const uint32_t pgn = id.pgn_raw();

    if (_msg_handlers.find(pgn) != _msg_handlers.end()) {
        for (CANSensor *handler : _msg_handlers[pgn]) {
            // Send the frame to all registered drivers for this PGN
            handler->handle_frame(frame);
        }
    }
}

#endif // HAL_J1939_CAN_ENABLED
