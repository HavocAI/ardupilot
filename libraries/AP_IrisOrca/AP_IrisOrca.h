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
   This driver supports communicating with the Iris Orca linear actuator via RS485
   ModBus protocol.
   For further documentation, see: <https://irisdynamics.com/downloads>
 */

#pragma once

#include "AP_IrisOrca_config.h"

#if HAL_IRISORCA_ENABLED

#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AC_PID/AC_PID_Basic.h>
#include <AP_Common/async.h>
#include "AP_IrisOrcaModbus.h"

#define IRISORCA_MESSAGE_LEN_MAX    35  // messages are no more than 35 bytes

namespace orca {

enum Result {
    OK = 0,
    ERROR = -1,
    TIMEOUT = -2,
};

struct get_firmware_state {
    get_firmware_state() {
        async_init(this);
        result = Result::ERROR;
    }
    async_state;
    Result result;
    uint16_t major_version;
    uint16_t release_state;
    uint16_t revision_number;
    uint16_t serial_number_low;
    uint16_t serial_number_high;
    uint16_t commit_hash_low;
    uint16_t commit_hash_high;
};

struct check_modbus_state {
    check_modbus_state() {
        async_init(this);
        result = Result::ERROR;
    }
    async_state;
    Result result;
    uint16_t reg_value;
};

} // namespace orca

class AP_IrisOrca {
public:
    AP_IrisOrca();
    CLASS_NO_COPY(AP_IrisOrca);

    static AP_IrisOrca* get_singleton();

    // initialise driver
    void init();

    // returns true if communicating with the actuator
    bool healthy();

    void send_mavlink_status(mavlink_channel_t ch);

    static const struct AP_Param::GroupInfo var_info[];

private:
    // parameters
    AP_Int8 _pin_de;                    // Pin number connected to RS485 to Serial converter's DE pin. -1 to disable sending commands to actuator
    AP_Int16 _max_travel_mm;            // maximum travel of actuator in millimeters
    AP_Int32 _f_max;                    // position control max force in milliNewtons
    AP_Int16 _gain_p;                   // position control proportional gain
    AP_Int16 _gain_i;                   // position control integral gain
    AP_Int16 _gain_dv;                  // position control derivative gain
    AP_Int16 _gain_de;                  // position control derivative error gain
    AP_Int16 _auto_zero_f_max;          // maximum force for auto zero in Newtons
    AP_Int8 _temp_derating_threshold;   // temperature threshold in degrees Celsius for derating
    AP_Float _throttle_activate;        // min throttle to activate steering

    AP_HAL::UARTDriver *_uart;
    bool _initialised;
    bool _healthy;

    struct run_state {
        
        async_state;
        uint32_t last_send_ms;
        
        
        orca::get_firmware_state get_firmware;
        orca::check_modbus_state check_modbus_rs485_mode;
    
    } _run_state;

    static AP_IrisOrca *_singleton;

    void run_io();
    async run();
    async read_firmware(orca::get_firmware_state *state);
    async check_modbus_rs485_mode(orca::check_modbus_state *state);
    uint32_t get_desired_shaft_pos();
    void disable_throttle();

    uint32_t _counter;
    uint8_t _num_timeouts;
    uint16_t _board_temp;


    orca::OperatingMode _operating_mode;
public:
    orca::ActuatorState _actuator_state;
private:

    ReadRegisterTransaction read_register_tx;
    WriteRegisterTransaction write_register_tx;
    WriteMotorCmdStreamTransaction write_motor_cmd_stream_tx;
    ReadMotorStreamTransaction read_motor_stream_tx;

    bool _disable_throttle;
    uint32_t _temp_derating_max_force;
    AC_PID_Basic _pid_temp;

};

namespace AP {
    AP_IrisOrca *irisorca();
};

#endif // HAL_IRISORCA_ENABLED