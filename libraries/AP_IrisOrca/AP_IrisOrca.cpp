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

// #if HAL_IRISORCA_ENABLED

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
    AP_GROUPINFO("PAD_TRAVEL", 4, AP_IrisOrca, _pad_travel_mm, 30),

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
    AP_GROUPINFO("AZ_F_MAX", 10, AP_IrisOrca, _auto_zero_f_max, 30),

    AP_GROUPEND
};

AP_IrisOrca::AP_IrisOrca()
 : _uart(nullptr), 
   _initialised(false),
   _healthy(false)
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);

    async_init(&_run_state);
}

void AP_IrisOrca::init()
{
    if (!_initialised) {
        hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_IrisOrca::run_io, void));
        _initialised = true;
    }
}


struct state { async_state; };


#define READ_REGISTER(reg, store) \
    _run_state.read_register_tx = ReadRegisterTransaction(_uart, static_cast<uint16_t>(reg)); \
    await( _run_state.read_register_tx.run() ); \
    if (_run_state.read_register_tx.is_timeout()) { \
        state->result = orca::Result::TIMEOUT; \
        async_exit; \
    } else { \
        store = _run_state.read_register_tx.reg_value(); \
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
    
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Serial %lu",
        static_cast<uint32_t>(state->serial_number_low |
        static_cast<uint32_t>(state->serial_number_high) << 16));

    async_end
}
#pragma GCC diagnostic pop


#define WRITE_REGISTER(reg, value, err_msg) \
    _run_state.write_register_tx = WriteRegisterTransaction(_uart, static_cast<uint16_t>(reg), static_cast<uint16_t>(value)); \
    await( _run_state.write_register_tx.run() ); \
    if (_run_state.write_register_tx.is_timeout()) { \
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, err_msg); \
        async_init(&_run_state); \
        return ASYNC_CONT; \
    }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
async AP_IrisOrca::run()
{
    async_begin(&_run_state)

    _run_state.last_send_ms = AP_HAL::millis();
    await(TIME_PASSED(_run_state.last_send_ms, 1000));

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Initialising");
    
    if (_uart == nullptr) {
        _uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_IrisOrca, 0);
        init_uart_for_modbus(_uart);
    }

    // set motor to sleep
    WRITE_REGISTER(orca::Register::CTRL_REG_3, orca::OperatingMode::SLEEP, "IrisOrca: Failed to set sleep mode");

    // read firmware version
    _run_state.get_firmware = orca::get_firmware_state();
    await( async_call(read_firmware, &_run_state.get_firmware) );
    if (_run_state.get_firmware.result != orca::Result::OK) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IrisOrca: Failed to read firmware version");
        async_init(&_run_state);
        return ASYNC_CONT;
    }

    

	while (true) {

        _run_state.last_send_ms = AP_HAL::millis();

	    // send at 10Hz
	    await(TIME_PASSED(_run_state.last_send_ms, 100));
    }
	
	async_end
}
#pragma GCC diagnostic pop

void AP_IrisOrca::run_io()
{
    run();
}

// returns true if communicating with the actuator
bool AP_IrisOrca::healthy()
{
    return _healthy;    
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

// #endif // HAL_IRISORCA_ENABLED