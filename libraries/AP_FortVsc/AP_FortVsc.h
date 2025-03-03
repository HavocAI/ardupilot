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
   This driver supports communicating with the FORT Robotics Vehicle
   Safety Controller via RS232. CAN communication is not yet supported.
   For further documentation, see: <https://fortrobotics.com/>

   Code by Andrew Gregg, 2025
 */

#pragma once

#include "AP_FortVsc_config.h"

#if HAL_FORTVSC_ENABLED

#include <AP_Param/AP_Param.h>

namespace fort {

    // maximum packet size
    static constexpr uint8_t MAX_PACKET_SIZE = 32;

    // Packet header
    static constexpr uint16_t PACKET_HEADER = 0x1001;

    // Message types
    enum class MessageType : uint8_t {
        JOYSTICK = 0x10,
        SRC_GPS = 0x12,
        HEARTBEAT = 0x20,
        REMOTE_STATUS = 0x22,
        HEARTBEAT_TO_VSC = 0x21,
        MESSAGE_CONTROL = 0x23,
        USER_FEEDBACK_SET = 0x30,
        USER_FEEDBACK_NAME = 0x31,
        USER_FEEDBACK_GET = 0x32
    };

    // Structure for a generic VSC packet
    struct VscPacket {
        uint16_t header;
        uint8_t messageType;
        uint8_t messageLength;
        uint8_t data[MAX_PACKET_SIZE];
        uint16_t checksum;
    };

}

class AP_FortVsc {
public:
    AP_FortVsc();

    CLASS_NO_COPY(AP_FortVsc);

    static AP_FortVsc* get_singleton();

    // initialize driver
    void init();

    // consume incoming messages from VSC, and send responses
    void thread_main();

    // returns true if communicating with the VSC
    bool healthy();

    static const struct AP_Param::GroupInfo var_info[];

private:
   
    // initialize serial port (run from background thread)
    // returns true on success
    bool init_internals();

    // send a message to the VSC
    void sendPacket(fort::MessageType type, const uint8_t *data, uint8_t length);

    // receive a message from the VSC
    bool receivePacket(fort::VscPacket &packet);

    // send a heartbeat message to the VSC
    void sendHeartbeatToVSC(uint8_t estopStatus);

    // send a message control message to the VSC
    void sendMessageControl(uint8_t messageType, uint8_t enabled, uint16_t interval);

    // send a user feedback set message to the VSC
    void sendUserFeedbackSet(uint8_t key, int32_t value);

    // send a user feedback get message to the VSC
    void sendUserFeedbackGet(uint8_t key);

    // get data from a joystick message and set RC overrides
    bool getJoystickData(const fort::VscPacket &packet) const;

    // map joystick value to RC value
    int16_t mapJoystickValue(uint16_t joystickData) const;

    // map joystick button bytes to RC values
    void mapJoystickButtons(uint8_t buttonData, int16_t pwmValues[4]) const;

    // get data from a heartbeat message
    bool getHeartbeat(const fort::VscPacket &packet, uint8_t &vscMode, uint8_t &autonomyMode) const;

    // get data from a remote status message
    bool getRemoteStatus(const fort::VscPacket &packet, uint8_t &batteryLevel, 
        uint8_t &batteryCharging, uint8_t &connectionStrength) const;

    // decode one character, return true if we have successfully completed a msg, false otherwise
    bool _decode(char c);

    // Process complete incoming messages
    void processIncomingMessages();

    // calculate the checksum of a packet
    uint16_t calculateChecksum(const uint8_t *data, uint8_t length) const;

    // parameters

    // members
    AP_HAL::UARTDriver *_uart;         // serial port to communicate with VSC
    bool _initialized;                 // true once driver has been initialized

    // parsing members
    uint8_t _recv_buffer[fort::MAX_PACKET_SIZE + 6];   // buffer for incoming data
    uint8_t _recv_buffer_index;                         // index into buffer
    fort::VscPacket _current_packet;            // current packet being received

    static AP_FortVsc *_singleton;  // singleton instance

};

namespace AP {
    AP_FortVsc *fortvsc();
}

#endif // HAL_FORTVSC_ENABLED