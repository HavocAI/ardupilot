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
    driver on a CAN port, and routes incoming frames to registered frontend drivers based on PGN.
    Each frontend driver can register for multiple PGNs, and each PGN can have multiple frontend
    drivers. This class is a "multiton" where only one instance can be created for each physical
    CAN port.

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
    // Only register the PGN, not the full 29-bit ID
    uint32_t pgn = J1939::extract_j1939_pgn(can_id);
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

    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "J1939: Driver already registered for PGN %lu", pgn);
    return false;
}

bool AP_J1939_CAN::send_message(J1939::J1939Frame &frame)
{
    AP_HAL::CANFrame can_frame = J1939::pack_j1939_frame(frame);
    return write_frame(can_frame, SEND_TIMEOUT_US);
}

void AP_J1939_CAN::handle_frame(AP_HAL::CANFrame &frame)
{
    if (!frame.isExtended())
    {
        // Only handle extended frames for J1939
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "J1939: Received non-extended frame");
        return;
    }

    uint32_t pgn = J1939::extract_j1939_pgn(frame.id);
    if (_msg_handlers.find(pgn) != _msg_handlers.end())
    {
        for (CANSensor *handler : _msg_handlers[pgn])
        {
            // Send the frame to all registered drivers for this PGN
            handler->handle_frame(frame);
        }
    }
    // else
    // {
    //     GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "J1939: Unhandled PGN %lu", pgn);
    // }
}

#endif // HAL_J1939_CAN_ENABLED
