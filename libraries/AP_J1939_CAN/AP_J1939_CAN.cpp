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
    This class is a backend for the AP_J1939 class that interfaces with the J1939 CAN protocol.
    The driver listens for incoming frames and forwards them to the appropriate handler based on the
    message ID. It also provides a mechanism for sending J1939 messages.

    Configuring Parameters in ArduRover:
    CAN_D1_PROTOCOL = 15 -- Sets the driver 1 protocol to J1939
    CAN_P2_DRIVER = 1 -- Sets the 2nd CAN port to use driver 1
    CAN_P2_BITRATE = 250000 -- Sets the 2nd CAN port bitrate
*/

#include "AP_J1939_CAN.h"

#if HAL_J1939_CAN_ENABLED
#include <GCS_MAVLink/GCS.h>
#include <algorithm>

extern const AP_HAL::HAL &hal;

// Static member initialization
std::map<uint8_t, AP_J1939_CAN*> AP_J1939_CAN::_instances;

AP_J1939_CAN::AP_J1939_CAN(uint8_t can_port) : CANSensor("J1939"), _can_port(can_port)
{
    CANSensor::register_driver(AP_CAN::Protocol::J1939);
}

AP_J1939_CAN* AP_J1939_CAN::get_instance(uint8_t can_port)
{
    if (_instances.find(can_port) == _instances.end())
    {
        _instances[can_port] = new AP_J1939_CAN(can_port);
    }
    return _instances[can_port];
}

bool AP_J1939_CAN::register_driver(uint32_t msg_id, CANSensor* driver)
{
    if (_msg_handlers.find(msg_id) == _msg_handlers.end())
    {
        _msg_handlers[msg_id] = std::vector<CANSensor*>();
    }

    // Check if the driver is already registered for this message ID
    auto& handlers = _msg_handlers[msg_id];
    if (std::find(handlers.begin(), handlers.end(), driver) == handlers.end())
    {
        handlers.push_back(driver);
        return true;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "J1939: Driver already registered for CAN ID %lu", msg_id);
    return false;
}

bool AP_J1939_CAN::send_message(uint32_t msg_id, const uint8_t* data, uint8_t len)
{
    uint32_t now = AP_HAL::millis();

    // Enforce rate limiting: 100ms minimum interval per message ID
    if (_last_sent_time.find(msg_id) != _last_sent_time.end() && (now - _last_sent_time[msg_id]) < 100)
    {
        return false;
    }

    _last_sent_time[msg_id] = now;

    AP_HAL::CANFrame frame;
    frame.id = msg_id | AP_HAL::CANFrame::FlagEFF;
    frame.dlc = len;
    frame.canfd = false;
    memcpy(frame.data, data, len);
    return write_frame(frame, 1000);
}

void AP_J1939_CAN::handle_frame(AP_HAL::CANFrame &frame)
{
    uint32_t msg_id = frame.id & AP_HAL::CANFrame::MaskExtID;
    if (_msg_handlers.find(msg_id) != _msg_handlers.end())
    {
        for (CANSensor* handler : _msg_handlers[msg_id])
        {
            handler->handle_frame(frame);
        }
    }
    else
    {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "J1939: Unhandled CAN ID %lu", msg_id);
    }
}

bool AP_J1939_CAN::enqueue_message(uint32_t msg_id, const uint8_t* data, uint8_t len)
{
    _queue_semaphore->take();
    if (_message_queue.size() < _max_queue_size)
    {
        _message_queue.push({msg_id, {}, len});
        memcpy(_message_queue.back().data, data, len);
        _queue_semaphore->give();
        return true;
    }
    else
    {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "J1939: Message queue full CAN ID %lu", msg_id);
        _queue_semaphore->give();
        return false;
    }
}

void AP_J1939_CAN::process_queue()
{
    while (true)
    {
        if (!_message_queue.empty())
        {
            _queue_semaphore->take();
            CANMessage msg = _message_queue.front();
            _message_queue.pop();
            _queue_semaphore->give();

            send_message(msg.msg_id, msg.data, msg.len);
        }
        hal.scheduler->delay_microseconds(5000); // Adjust for performance
    }
}

#endif // HAL_J1939_CAN_ENABLED
