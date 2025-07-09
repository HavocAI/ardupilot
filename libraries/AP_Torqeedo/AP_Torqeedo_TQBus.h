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
   This driver supports communicating with Torqeedo motors that implement the "TQ Bus" protocol
   which includes the Ultralight, Cruise 2.0 R, Cruise 4.0 R, Travel 503, Travel 1003 and Cruise 10kW

   The autopilot should be connected either to the battery's tiller connector or directly to the motor
   as described on the ArduPilot wiki. https://ardupilot.org/rover/docs/common-torqeedo.html
   TQ Bus is a serial protocol over RS-485 meaning that a serial to RS-485 converter is required.

       Tiller connection: Autopilot <-> Battery (master) <-> Motor
       Motor connection:  Autopilot (master) <-> Motor

    Communication between the components is always initiated by the master with replies sent within 25ms

    Example "Remote (0x01)" reply message to allow tiller to control motor speed
    Byte        Field Definition    Example Value   Comments
    ---------------------------------------------------------------------------------
    byte 0      Header              0xAC
    byte 1      TargetAddress       0x00            see MsgAddress enum
    byte 2      Message ID          0x00            only master populates this. replies have this set to zero
    byte 3      Flags               0x05            bit0=pin present, bit2=motor speed valid
    byte 4      Status              0x00            0x20 if byte3=4, 0x0 is byte3=5
    byte 5      Motor Speed MSB     ----            Motor Speed MSB (-1000 to +1000)
    byte 6      Motor Speed LSB     ----            Motor Speed LSB (-1000 to +1000)
    byte 7      CRC-Maxim           ----            CRC-Maxim value
    byte 8      Footer              0xAD

   More details of the TQ Bus protocol are available from Torqeedo after signing an NDA.
 */

#pragma once

#include "AP_Torqeedo_config.h"

#if HAL_TORQEEDO_ENABLED

#include "AP_Torqeedo_Backend.h"
#include <GCS_MAVLink/GCS.h>

#define TORQEEDO_MESSAGE_LEN_MAX    35  // messages are no more than 35 bytes

class AP_Torqeedo_TQBus : public AP_Torqeedo_Backend {
public:

    // constructor
    using AP_Torqeedo_Backend::AP_Torqeedo_Backend;

    AP_Torqeedo_TQBus(AP_Torqeedo_Params &params, uint8_t instance);

    CLASS_NO_COPY(AP_Torqeedo_TQBus);

    // initialise driver
    void init() override;

    // returns true if communicating with the motor
    bool healthy() override;

    // get latest battery status info.  returns true on success and populates arguments
    bool get_batt_info(float &voltage, float &current_amps, float &temp_C, uint8_t &pct_remaining) const override;
    bool get_batt_capacity_Ah(uint16_t &amp_hours) const override;

    void send_mavlink_status(mavlink_channel_t ch) override;

private:

    AP_HAL::UARTDriver* _uart;
    int16_t _motor_speed_desired = 0; // desired motor speed in range -1000 to +1000
    float _filtered_desired_speed = 0.0f; // filtered desired motor speed in range -1000 to +1000
    int16_t _motor_rpm = 0;
    uint32_t _last_rx_ms = 0; // last time a message was received from the motor
    uint32_t _last_set_rpm_ms = 0; // last time we set the motor speed
    uint32_t _last_state_change_ms = 0;
    uint32_t _last_master_error_code_set_ms = 0;

    mavlink_torqeedo_telemetry_t _torqeedo_telemetry = {}; // telemetry data to send to GCS

    enum class ExpectedReply {
        None,
        MotorStatus,
        MotorParam,
    } _expectedReply = ExpectedReply::None;
    
    enum class DriverState {
        Init,
        Stop,
        Ready,
        Forward,
        Reverse,
        PowerOn,
        PowerOff,
        Error,
    } _state;

    enum class ComsState {
        Healthy,
        Unhealthy,
    } _comsState;

    

    // consume incoming messages from motor, reply with latest motor speed
    // runs in background thread
    void thread_main();
    
    void filter_desired_speed();

    void set_master_error_code(uint8_t error_code);

    void process_rx_frame(const uint8_t* frame, uint8_t len);
    void handle_bus_master_msg(const uint8_t* frame, uint8_t len);
    void handle_remote_msg(const uint8_t* frame, uint8_t len);
    void handle_display_msg(const uint8_t* frame, uint8_t len);
    void handle_motor_msg(const uint8_t* frame, uint8_t len);

    
};

#endif // HAL_TORQEEDO_ENABLED
