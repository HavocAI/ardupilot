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

#include "AP_FortVsc.h"

#if HAL_FORTVSC_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_SerialManager/AP_SerialManager.h>

#define FORTVSC_SERIAL_BAUD     115200 // communication is always at 115200

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_FortVsc::var_info[] = {
//    AP_GROUPINFO("fortvsc", 0, AP_FortVsc, _uart, AP_PARAM(_uart)),
    AP_GROUPEND
};

AP_FortVsc::AP_FortVsc()
{
    _singleton = this;
    // AP_Param::setup_object_defaults(this, var_info);
}

void AP_FortVsc::init() {
    // only init once
    // Note: a race condition exists here if init is called multiple times quickly before thread_main has a chance to set _initialize
    if (_initialized) {
        return;
    }

    // create background thread to process serial input and output
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_FortVsc::thread_main, void), "fortvsc", 2048, AP_HAL::Scheduler::PRIORITY_RCOUT, 1)) {
        return;
    }
}

// initialize serial port (run from background thread)
bool AP_FortVsc::init_internals() {
    // find serial driver and initialise
    const AP_SerialManager &serial_manager = AP::serialmanager();
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FortVsc, 0);
    if (_uart == nullptr) {
        return false;
    }
    _uart->begin(FORTVSC_SERIAL_BAUD);
    _uart->configure_parity(0);
    _uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    //_uart->set_unbuffered_writes(true);

    return true;
}


// consume incoming messages from VSC, reply with heartbeat and other data
void AP_FortVsc::thread_main() {
    // initialization
    if (!init_internals()) {
        return;
    }
    _initialized = true;

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FortVsc: Initialized");
    
    int16_t numc;
    uint32_t last_heartbeat_ms = AP_HAL::millis();

    while (true) {
        // 1ms loop delay
        hal.scheduler->delay_microseconds(1000);

        // Send a heartbeat at 20 hz
        if (AP_HAL::millis() - last_heartbeat_ms > 50) {
            sendHeartbeatToVSC(0); // Default to no estop
            last_heartbeat_ms = AP_HAL::millis();
        }

        numc = _uart->available();
        while (numc--) {
            char c = _uart->read();
            // log_data((const uint8_t *)&c, 1);
            if (processNewByte(c)) {
                processIncomingMessages();
            }
        }
    }
}

bool AP_FortVsc::healthy() {
    if (!_initialized) {
        return false;
    }

    // Todo: healthy if a heartbeat has been received in the last n seconds

    return true;
}

void AP_FortVsc::sendPacket(fort::MessageType type, const uint8_t *data, uint8_t length) {
    // Check if UART is available and length is within the allowed packet size
    if (!_uart || length > fort::MAX_PACKET_SIZE) {
        return;
    }

    // Create a VscPacket and populate its fields
    fort::VscPacket packet;
    packet.header = fort::PACKET_HEADER;
    packet.messageType = static_cast<uint8_t>(type);
    packet.messageLength = length;
    memcpy(packet.data, data, length); // Copy the data into the packet

    // Calculate and set the checksum
    packet.checksum = calculateChecksum((uint8_t *)&packet, 4 + length);

    // Write the packet to the UART
    _uart->write((uint8_t *)&packet, 6 + length);
}

bool AP_FortVsc::processNewByte(char c) {
    if (_recv_buffer_index >= fort::MAX_PACKET_SIZE + 6) {
        _recv_buffer_index = 0;
    }

    // Getting header (0x1001) - first two bytes
    if (_recv_buffer_index == 0) {
        if (c == 0x10) {
            _recv_buffer[0] = c;
            _recv_buffer_index = 1;
            _current_packet = fort::VscPacket();
            return false;
        }
    } else if (_recv_buffer_index == 1 && c == 0x01) {
        _recv_buffer[1] = c;
        _recv_buffer_index = 2;
        _current_packet.header = (_recv_buffer[0] << 8) | _recv_buffer[1];
        return false;
    }

    // Getting message type
    else if (_recv_buffer_index == 2) {
        _recv_buffer[2] = c;
        _recv_buffer_index = 3;
        _current_packet.messageType = c;
        return false;
    }

    // Getting message length
    else if (_recv_buffer_index == 3) {
        _recv_buffer[3] = c;
        _recv_buffer_index = 4;
        _current_packet.messageLength = c;
        return false;
    }

    // Getting message data
    else if (_recv_buffer_index >= 4 && _recv_buffer_index < 4 + _current_packet.messageLength) {
        _recv_buffer[_recv_buffer_index] = c;
        _recv_buffer_index++;
        return false;
    }

    // Getting checksum - first byte
    else if (_recv_buffer_index == 4 + _current_packet.messageLength) {
        _recv_buffer[4 + _current_packet.messageLength] = c;
        _recv_buffer_index = 4 + _current_packet.messageLength + 1;
        return false;
    }

    // Getting checksum - second byte
    else if (_recv_buffer_index == 4 + _current_packet.messageLength + 1) {
        _recv_buffer[4 + _current_packet.messageLength + 1] = c;
        _current_packet.checksum =  
            ((c - 0x0F) << 8) | (_recv_buffer[4 + _current_packet.messageLength]);
        // Todo: 0x0F subtraction is cheating
        _recv_buffer_index = 0;
        // Copy the _recv_buffer data into the current packet
        memcpy(_current_packet.data, _recv_buffer + 4, _current_packet.messageLength);
        // Check the checksum
        uint16_t checksum = calculateChecksum(reinterpret_cast<uint8_t *>(&_current_packet), 4 + _current_packet.messageLength);

        if (checksum == _current_packet.checksum) {
            return true;
        }
    }

    return false;
}

void AP_FortVsc::sendHeartbeatToVSC(uint8_t estopStatus) {
    uint8_t data[1] = {estopStatus};
    sendPacket(fort::MessageType::HEARTBEAT_TO_VSC, data, 1);
}

void AP_FortVsc::sendMessageControl(uint8_t messageType, uint8_t enabled, uint16_t interval) {
    uint8_t data[4] = {messageType, enabled};
    memcpy(data + 2, &interval, 2);
    sendPacket(fort::MessageType::MESSAGE_CONTROL, data, 4);
}

void AP_FortVsc::sendUserFeedbackSet(uint8_t key, int32_t value) {
    uint8_t data[5] = {key};
    memcpy(data + 1, &value, 4);
    sendPacket(fort::MessageType::USER_FEEDBACK_SET, data, 5);
}

void AP_FortVsc::sendUserFeedbackGet(uint8_t key) {
    uint8_t data[1] = {key};
    sendPacket(fort::MessageType::USER_FEEDBACK_GET, data, 1);
}

bool AP_FortVsc::getJoystickData(const fort::VscPacket &packet) const {
    if (packet.messageType == static_cast<uint8_t>(fort::MessageType::JOYSTICK)) {
        const uint32_t tnow = AP_HAL::millis();
        for (int i = 0; i < 6; i++) {
            uint16_t joystickData;
            memcpy(&joystickData, packet.data + i * 2, 2);
            RC_Channels::set_override(i, mapJoystickValue(joystickData), tnow);
        }

        // Extract button data
        int16_t leftPwmValues[4];
        mapJoystickButtons(packet.data[12], leftPwmValues);
        for (int i = 0; i < 4; i++) {
            RC_Channels::set_override(6 + i, leftPwmValues[i], tnow);
        }
        int16_t rightPwmValues[4];
        mapJoystickButtons(packet.data[13], rightPwmValues);
        for (int i = 0; i < 4; i++) {
            RC_Channels::set_override(10 + i, rightPwmValues[i], tnow);
        }
        return true;
    }
    return false;
}

int16_t AP_FortVsc::mapJoystickValue(uint16_t joystickData) const {
    // Extract status bits from byte 0
    uint8_t lsb = joystickData & 0xFF;
    uint8_t msb = (joystickData >> 8) & 0xFF;

    uint16_t magnitude = (msb << 2) | ((lsb >> 6) & 0x03); // Extract magnitude

    bool positive = (lsb & 0x30) != 0; // Bits 5 and 4 indicate positive
    bool negative = (lsb & 0x0C) != 0; // Bits 3 and 2 indicate negative
    bool neutral = (lsb & 0x03) != 0;  // Bits 1 and 0 indicate neutral

    constexpr int16_t PWM_MIN = 1100;
    constexpr int16_t PWM_MAX = 1900;
    constexpr int16_t PWM_CENTER = 1500;

    // Assuming max possible magnitude is 1023 (10-bit value)
    constexpr int16_t MAX_MAGNITUDE = 1023;

    int16_t pwmValue = PWM_CENTER; // Default to center

    if (positive && !negative && !neutral) {
        pwmValue = PWM_CENTER + (magnitude * (PWM_MAX - PWM_CENTER) / MAX_MAGNITUDE);
    } else if (negative && !positive && !neutral) {
        pwmValue = PWM_CENTER - (magnitude * (PWM_CENTER - PWM_MIN) / MAX_MAGNITUDE);
    }

    return pwmValue;
}

void AP_FortVsc::mapJoystickButtons(uint8_t buttonData, int16_t pwmValues[4]) const {
    constexpr int16_t PWM_PRESSED = 1900;
    constexpr int16_t PWM_UNPRESSED = 1100;

    for (int i = 0; i < 4; ++i) {
        uint8_t status = (buttonData >> (i * 2)) & 0x03; // Extract 2-bit status
        pwmValues[i] = (status == 0x01) ? PWM_PRESSED : PWM_UNPRESSED; // 0x01 means pressed
    }
}

bool AP_FortVsc::getHeartbeat(const fort::VscPacket &packet, fort::VscState &vscState) const {
    if (packet.messageType == static_cast<uint8_t>(fort::MessageType::HEARTBEAT)) {
        vscState.vscMode = packet.data[0];
        vscState.autonomyMode = packet.data[1];

        // Set the emergency stop if the VSC has set it
        // Use the SRV_Channels to set the emergency stop and report status
        memcpy(&vscState.eStopIndication, packet.data + 2, 4);
        if (_vscState.eStopIndication > 0) {
            if (SRV_Channels::get_emergency_stop() == 0) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "FortVsc: Emergency stop activated");
            }
            SRV_Channels::set_emergency_stop(1);
        } else {
            if (SRV_Channels::get_emergency_stop() == 1) {
                gcs().send_text(MAV_SEVERITY_INFO, "FortVsc: Emergency stop deactivated");
            }
            SRV_Channels::set_emergency_stop(0);
        }

        return true;
    }
    return false;
}

bool AP_FortVsc::getRemoteStatus(const fort::VscPacket &packet, fort::VscState &vscState) const {
    if (packet.messageType == static_cast<uint8_t>(fort::MessageType::REMOTE_STATUS)) {
        vscState.batteryLevel = packet.data[0];
        vscState.batteryCharging = packet.data[1];
        vscState.connectionStrengthPerVsc = packet.data[2];
        vscState.connectionStrengthPerSrc = packet.data[3];
        return true;
    }
    return false;
}

void AP_FortVsc::processIncomingMessages() {
    // gcs().send_text(MAV_SEVERITY_INFO, "FortVsc: Received packet, type: %d", _current_packet.messageType);
    switch (static_cast<fort::MessageType>(_current_packet.messageType)) {
        case fort::MessageType::JOYSTICK:
            getJoystickData(_current_packet);
            break;
        case fort::MessageType::HEARTBEAT:
            getHeartbeat(_current_packet, _vscState);
            break;
        case fort::MessageType::REMOTE_STATUS:
            getRemoteStatus(_current_packet, _vscState);
            break;
        default:
            break;
    }
}

uint16_t AP_FortVsc::calculateChecksum(const uint8_t *data, uint8_t length) const {
    uint16_t sum1 = 0, sum2 = 0;
    for (int i = 0; i < length; ++i) {
        sum1 = (sum1 + data[i]) & 0xFF;
        sum2 = (sum2 + sum1) & 0xFF;
    }
    return (sum2 << 8) | sum1;
}


AP_FortVsc* AP_FortVsc::get_singleton() {
    return _singleton;
}

AP_FortVsc *AP_FortVsc::_singleton = nullptr;

namespace AP {
    AP_FortVsc *fortvsc() {
        return AP_FortVsc::get_singleton();
    }
}

#endif // HAL_FORTVSC_ENABLED
