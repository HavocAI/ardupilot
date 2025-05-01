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

#include "AP_IrisOrca.h"

#if HAL_IRISORCA_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include "AP_IrisOrcaModbus.h"

#define HIGHWORD(x) ((uint16_t)((x) >> 16))
#define LOWWORD(x) ((uint16_t)(x))
#define TIME_PASSED(start, delay_ms) (AP_HAL::millis() - (start) > delay_ms)


extern const AP_HAL::HAL& hal;

// parameters
const AP_Param::GroupInfo AP_IrisOrca::var_info[] = {

    // @Param: DE_PIN
    // @DisplayName: Iris Orca DE pin
    // @Description: Pin number connected to RS485 to Serial converter's DE pin. -1 to use serial port's CTS pin if available
    // @Values: -1:Disabled,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("DE_PIN", 1, AP_IrisOrca, _pin_de, -1),

    // @Param: MAX_TRAVEL
    // @DisplayName: Shaft max physical travel distance
    // @Description: The max physical travel distance as measured from the zero position, which will be at one end of the actuator after zeroing.
    // @Units: mm
    // @Range: 0 300
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MAX_TRAVEL", 2, AP_IrisOrca, _max_travel_mm, 205),

    // @Param: REVERSE_DIR
    // @DisplayName: Reverse direction
    // @Description: Reverse the direction of the actuator
    // @Values: 0:Normal,1:Reverse
    // @User: Standard
    AP_GROUPINFO("REVERSE_DIR", 3, AP_IrisOrca, _reverse_direction, 0),

    // @Param: PAD_TRAVEL
    // @DisplayName: Pad travel distance
    // @Description: Amount to pad the physical travel distance by to ensure the actuator does not reach the physical end stops during normal motion.
    // @Units: mm
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("PAD_TRAVEL", 4, AP_IrisOrca, _pad_travel_mm, 10),

    // @Param: F_MAX
    // @DisplayName: Maximum force
    // @Description: Maximum force for position control
    // @Units: mN
    // @Range: 0 1061000
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("F_MAX", 5, AP_IrisOrca, _f_max, 638000),

    // @Param: GAIN_P
    // @DisplayName: Position control P gain
    // @Description: Proportional gain for position control
    // @Units: 64*N/mm
    // @Range: 0 65535
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("GAIN_P", 6, AP_IrisOrca, _gain_p, 200),

    // @Param: GAIN_I
    // @DisplayName: Position control I gain
    // @Description: Integral gain for position control
    // @Units: 64*N*s/mm
    // @Range: 0 65535
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("GAIN_I", 7, AP_IrisOrca, _gain_i, 1000),

    // @Param: GAIN_DV
    // @DisplayName: Position control Dv gain
    // @Description: Derivative gain for position control
    // @Units: 2*N*s/m
    // @Range: 0 65535
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("GAIN_DV", 8, AP_IrisOrca, _gain_dv, 800),

    // @Param: GAIN_DE
    // @DisplayName: Position control De gain
    // @Description: Derivative error gain for position control
    // @Units: 2*N*s/m err
    // @Range: 0 65535
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("GAIN_DE", 9, AP_IrisOrca, _gain_de, 0),

    // @Param: AZ_F_MAX
    // @DisplayName: Auto-zero max force
    // @Description: Force threshold for auto-zero mode
    // @Units: N
    // @Range: 0 1061
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("AZ_F_MAX", 10, AP_IrisOrca, _auto_zero_f_max, 300),

    AP_GROUPEND
};

AP_IrisOrca::AP_IrisOrca()
 : _uart(nullptr), 
   _initialised(false),
   _healthy(false),
   _disable_throttle(false)
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);

    async_init(&_run_state);
}

void AP_IrisOrca::init()
{
    if (!_initialised) {
        _initialised = true;
        // hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_IrisOrca::run_io, void));
        hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_IrisOrca::run_io, void), "irisorca", 2048, AP_HAL::Scheduler::PRIORITY_TIMER, 0);
    }
}


struct state { async_state; };


#define READ_REGISTER(reg, store) \
    read_register_tx = ReadRegisterTransaction(_uart, static_cast<uint16_t>(reg)); \
    await( read_register_tx.run() ); \
    if (read_register_tx.is_timeout()) { \
        state->result = orca::Result::TIMEOUT; \
        async_exit; \
    } else { \
        store = read_register_tx.reg_value(); \
    }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
async AP_IrisOrca::read_firmware(orca::get_firmware_state *state)
{
    async_begin(state)

    READ_REGISTER(orca::Register::MAJOR_VERSION, state->major_version);
    READ_REGISTER(orca::Register::RELEASE_STATE, state->release_state);
    READ_REGISTER(orca::Register::REVISION_NUMBER, state->revision_number);
    READ_REGISTER(orca::Register::SERIAL_NUMBER_LOW, state->serial_number_low);
    READ_REGISTER(orca::Register::SERIAL_NUMBER_HIGH, state->serial_number_high);
    READ_REGISTER(orca::Register::COMMIT_ID_LOW, state->commit_hash_low);
    READ_REGISTER(orca::Register::COMMIT_ID_HIGH, state->commit_hash_high);
    
    state->result = orca::Result::OK;

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Fw %u.%u.%u",
        state->major_version,
        state->release_state,
        state->revision_number);
    
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Serial %" PRIu32 "",
        static_cast<uint32_t>(state->serial_number_low |
        static_cast<uint32_t>(state->serial_number_high) << 16));

    async_end
}
#pragma GCC diagnostic pop


#define WRITE_REGISTER(reg, value, err_msg) \
    write_register_tx = WriteRegisterTransaction(_uart, static_cast<uint16_t>(reg), static_cast<uint16_t>(value)); \
    await( write_register_tx.run() ); \
    if (write_register_tx.is_timeout()) { \
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, err_msg); \
        async_init(&_run_state); \
        return ASYNC_CONT; \
    }

#define BLOCKING_SLEEP 1
#ifdef BLOCKING_SLEEP

#define SLEEP(ms) AP_HAL::get_HAL().scheduler->delay(ms)

#else

#define SLEEP(ms) \
    await(TIME_PASSED(_run_state.last_send_ms, ms))

#endif

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
async AP_IrisOrca::run()
{
    async_begin(&_run_state)

    _healthy = false;
    _disable_throttle = true;

    if (_uart == nullptr) {
        _uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_IrisOrca, 0);
        init_uart_for_modbus(_uart);
    }

    if (_uart == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IrisOrca: Failed to find serial port");
        async_init(&_run_state);
        return ASYNC_CONT;
    }

    _run_state.last_send_ms = AP_HAL::millis();
    SLEEP(5000);

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Initialising");

    // WRITE_REGISTER(orca::Register::CTRL_REG_0, 1, "IrisOrca: Failed to restart motor");
    // SLEEP(7000);

    // set motor to sleep
    WRITE_REGISTER(orca::Register::CTRL_REG_3, orca::OperatingMode::SLEEP, "IrisOrca: not responding");

    // WRITE_REGISTER(orca::Register::CTRL_REG_4, 7, "IrisOrca: reset defaults"); 

    // read firmware version
    _run_state.get_firmware = orca::get_firmware_state();
    await( async_call(read_firmware, &_run_state.get_firmware) );
    if (_run_state.get_firmware.result != orca::Result::OK) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IrisOrca: Failed to read firmware version");
        async_init(&_run_state);
        return ASYNC_CONT;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: P/I/D: %d/%d/%d, De: %d",
        _gain_p.get(),
        _gain_i.get(),
        _gain_dv.get(),
        _gain_de.get());

    // These PID values are used durring AUTO-ZERO. 
    // We found its important to have ~1000 P gain to get the motor to home correctly.
    WRITE_REGISTER(orca::Register::PC_PGAIN, 2000, "IrisOrca: Failed to set P gain");
    WRITE_REGISTER(orca::Register::PC_IGAIN, 1000, "IrisOrca: Failed to set I gain");
    WRITE_REGISTER(orca::Register::PC_DVGAIN, 100, "IrisOrca: Failed to set Dv gain");
    WRITE_REGISTER(orca::Register::PC_DEGAIN, _gain_de, "IrisOrca: Failed to set De gain");
    WRITE_REGISTER(orca::Register::MB_POS_FILTER, 9950, "IrisOrca: Failed to set position filter");

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: max force: %" PRIu32, _f_max.get());
    WRITE_REGISTER(orca::Register::PC_FSATU, LOWWORD(_f_max), "IrisOrca: Failed to set max force");
    WRITE_REGISTER(orca::Register::PC_FSATU_H, HIGHWORD(_f_max), "IrisOrca: Failed to set max force");

    // WRITE_REGISTER(orca::Register::USER_MAX_FORCE, LOWWORD(0), "IrisOrca: Failed to set max force");
    // WRITE_REGISTER(orca::Register::USER_MAX_FORCE_H, HIGHWORD(0), "IrisOrca: Failed to set max force");

    // set comms timeout to 300ms
    WRITE_REGISTER(orca::Register::USER_COMMS_TIMEOUT, 300, "IrisOrca: Failed to set comms timeout");

    // set auto zero max force
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Auto zero max force %" PRIi16, _auto_zero_f_max.get());
    WRITE_REGISTER(orca::Register::AUTO_ZERO_FORCE_N, _auto_zero_f_max.get(), "IrisOrca: Failed to set auto zero max force");

    // set auto zero exit mode
    WRITE_REGISTER(orca::Register::AUTO_ZERO_EXIT_MODE, orca::OperatingMode::POSITION, "IrisOrca: Failed to set auto zero exit mode");

    // set auto-zero enabled
    WRITE_REGISTER(orca::Register::ZERO_MODE, 2, "IrisOrca: Failed to set auto zero mode");

    // set motor to auto zero
    WRITE_REGISTER(orca::Register::CTRL_REG_3, orca::OperatingMode::AUTO_ZERO, "IrisOrca: Failed to set auto zero mode");

    // read the motor stream and wait to see the operating mode goto position
    while (true) {
        _run_state.last_send_ms = AP_HAL::millis();

        read_motor_stream_tx = ReadMotorStreamTransaction(_uart, orca::Register::CTRL_REG_3, 1);
        await( read_motor_stream_tx.run() );
        if (read_motor_stream_tx.is_timeout()) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IrisOrca: Failed to read motor stream");
            async_init(&_run_state);
            return ASYNC_CONT;
        }

        
        _actuator_state = read_motor_stream_tx.actuator_state();
        _operating_mode = read_motor_stream_tx.operating_mode();

        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Shaft position %" PRIi32, _actuator_state.shaft_position);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Force realized %" PRIi32, _actuator_state.force_realized);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Power consumed %" PRIu16, _actuator_state.power_consumed);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Temperature %" PRIu8, _actuator_state.temperature);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Voltage %" PRIu16, _actuator_state.voltage);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Operating mode %" PRIu8, (uint8_t)_operating_mode);

        if (_actuator_state.errors) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IrisOrca: Motor error 0x%04X", _actuator_state.errors);
            async_init(&_run_state);
            return ASYNC_CONT;
        }

        if (_operating_mode == orca::OperatingMode::AUTO_ZERO) {
        } else if (_operating_mode == orca::OperatingMode::POSITION) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Position control");
            break;
        } else if (_operating_mode == orca::OperatingMode::SLEEP) {
            WRITE_REGISTER(orca::Register::CTRL_REG_3, orca::OperatingMode::AUTO_ZERO, "IrisOrca: Failed to set auto zero mode");
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IrisOrca: Unknown operating mode %u", (uint8_t)_operating_mode);
            async_init(&_run_state);
            return ASYNC_CONT;
        }

        SLEEP(100);
    }

    // set the pids for position control
    WRITE_REGISTER(orca::Register::PC_PGAIN, _gain_p, "IrisOrca: Failed to set P gain");
    WRITE_REGISTER(orca::Register::PC_IGAIN, _gain_i, "IrisOrca: Failed to set I gain");
    WRITE_REGISTER(orca::Register::PC_DVGAIN, _gain_dv, "IrisOrca: Failed to set Dv gain");
    WRITE_REGISTER(orca::Register::PC_DEGAIN, _gain_de, "IrisOrca: Failed to set De gain");
    // then write the bit to have it take effect immediately
    WRITE_REGISTER(orca::Register::CTRL_REG_1, (1 << 10), "IrisOrca: Failed to set position control mode");

    _counter = 0;
    _num_timeouts = 0;

	while (true) {
        _run_state.last_send_ms = AP_HAL::millis();

        write_motor_cmd_stream_tx = WriteMotorCmdStreamTransaction(_uart, orca::MotorCommandStreamSubCode::POSITION_CONTROL_STREAM, get_desired_shaft_pos());
        await( write_motor_cmd_stream_tx.run() );
        if (write_motor_cmd_stream_tx.is_timeout()) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IrisOrca: timeout writing motor command stream");
            _num_timeouts++;
            if (_num_timeouts > 5) {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IrisOrca: Too many timeouts");
                async_init(&_run_state);
                return ASYNC_CONT;
            }
        } else {
            _num_timeouts = 0;
            _healthy = true;
            _disable_throttle = false;
            _actuator_state = write_motor_cmd_stream_tx.actuator_state();
            if (_counter++ % 100 == 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Shaft position %" PRIi32, _actuator_state.shaft_position);
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Force realized %" PRIi32, _actuator_state.force_realized);
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Power consumed %" PRIu16, _actuator_state.power_consumed);
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Temperature %" PRIu8, _actuator_state.temperature);
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Voltage %" PRIu16, _actuator_state.voltage);
            }

            if (_actuator_state.errors) {
                _disable_throttle = true;
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IrisOrca: Motor error 0x%04X", _actuator_state.errors);
                async_init(&_run_state);
                return ASYNC_CONT;
            }
        }

	    // send at 10Hz
        SLEEP(100);
    }
	
	async_end
}
#pragma GCC diagnostic pop

void AP_IrisOrca::run_io()
{
    while (true) {
        run();
        disable_throttle();
    }
}

void AP_IrisOrca::disable_throttle()
{
    if (_disable_throttle) {
        uint8_t chan;
        if (SRV_Channels::find_channel(SRV_Channel::k_throttle, chan)) {
            SRV_Channel* ch = SRV_Channels::srv_channel(chan);
            uint16_t pwm = ch->pwm_from_scaled_value(0.0);
            SRV_Channels::set_output_pwm_chan_timeout(chan, pwm, 500);
        }
    }
}

// returns true if communicating with the actuator
bool AP_IrisOrca::healthy()
{
    return _healthy;    
}

uint32_t AP_IrisOrca::get_desired_shaft_pos()
{
    const float yaw = constrain_float(SRV_Channels::get_output_norm(SRV_Channel::Aux_servo_function_t::k_steering), -1.0, 1.0);

    const float m = (_reverse_direction ? -1.0 : 1.0) * 0.5 * 1000 * (_max_travel_mm - (2.0 * _pad_travel_mm));
    const float b = 0.5 * 1000 * _max_travel_mm;
    
    const float shaft_position_um = yaw * m + b;

    return shaft_position_um;
}


// get the AP_IrisOrca singleton
AP_IrisOrca *AP_IrisOrca::get_singleton()
{
    return _singleton;
}

AP_IrisOrca *AP_IrisOrca::_singleton = nullptr;

namespace AP {
AP_IrisOrca *irisorca()
{
    return AP_IrisOrca::get_singleton();
}
};

#endif // HAL_IRISORCA_ENABLED