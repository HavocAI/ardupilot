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

#include "AP_MarineICE.h"

#if HAL_MARINEICE_ENABLED

#include "AP_MarineICE_Backend.h"
#include "AP_MarineICE_Simulator.h"

#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Arming/AP_Arming.h>
#include <SRV_Channel/SRV_Channel.h>

using namespace MarineICE::Types;
using namespace MarineICE::States;

const AP_Param::GroupInfo AP_MarineICE::var_info[] = {

    AP_SUBGROUPINFO(_params, "", 1, AP_MarineICE, AP_MarineICE_Params),

    AP_GROUPEND
};

AP_MarineICE::AP_MarineICE()
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);

    // setup the state machine
    setup_states();
}

void AP_MarineICE::init()
{

    // check init has not been called before
    if (_backend != nullptr)
    {
        return;
    }

    // Set up the backend
    switch ((BackendType)_params.type.get())
    {
    case BackendType::BACKEND_TYPE_DISABLED:
        return;
    case BackendType::BACKEND_TYPE_SIMULATED:
        _backend = NEW_NOTHROW AP_MarineICE_Simulator(_params);
        break;
    case BackendType::BACKEND_TYPE_NMEA2000:
        // TODO: implement NMEA2000 backend
        return;
    default:
        return;
    }

    _backend->init();

    // initialize the state machines
    _fsm_engine.change_state(EngineState::ENGINE_INIT, *this);
    _fsm_trim.change_state(TrimState::TRIM_MANUAL, *this);
}

// returns true if a backend has been configured (e.g. TYPE param has been set)
bool AP_MarineICE::enabled() const
{
    switch ((BackendType)_params.type.get())
    {
    case BackendType::BACKEND_TYPE_DISABLED:
        return false;
    case BackendType::BACKEND_TYPE_SIMULATED:
        return true;
    case BackendType::BACKEND_TYPE_NMEA2000:
        return true;
    default:
        return false;
    }
    return false;
}

bool AP_MarineICE::healthy()
{
    if (_backend == nullptr)
    {
        return false;
    }
    return _backend->healthy();
}

bool AP_MarineICE::pre_arm_checks(char *failure_msg, uint8_t failure_msg_len)
{
    // exit immediately if not enabled
    if (!enabled())
    {
        return true;
    }

    // check if the backend is healthy
    if (!healthy())
    {
        strncpy(failure_msg, "not healthy", failure_msg_len);
        return false;
    }
    return true;
}

void AP_MarineICE::update()
{
    // Check for faults
    monitor_faults();

    // Check if any fault is set
    bool any_fault = false;
    for (bool fault : _faults) {
        if (fault) {
            any_fault = true;
            break;
        }
    }

    if (any_fault &&
        _fsm_engine.current_state_id() != EngineState::ENGINE_FAULT) {
        // If any fault is set, change to the fault state
        _fsm_engine.change_state(EngineState::ENGINE_FAULT, *this);
    } else if (get_active_engine_stop() && 
            _fsm_engine.current_state_id() != EngineState::ENGINE_INIT) {
        // If there is an engine stop condition, change to the init state
        _fsm_engine.change_state(EngineState::ENGINE_INIT, *this);
    }

    _fsm_engine.update(*this);
    _fsm_trim.update(*this);

    if (healthy())
    {
        _backend->update_esc_telemetry();
    }
}

bool AP_MarineICE::get_cmd_neutral_lock() const
{
    static bool neutral_lock = constrain_int16((SRV_Channels::get_output_norm(
        SRV_Channel::k_engine_run_enable) + 1) * 0.5, 0, 1) != 0;
    return neutral_lock;
}

int16_t AP_MarineICE::get_cmd_throttle() const { 
    return constrain_int16(SRV_Channels::get_output_norm(
        SRV_Channel::k_throttle) * 100, -100, 100); 
}

uint16_t AP_MarineICE::get_cmd_trim() const { 
    return constrain_int16(SRV_Channels::get_output_norm(
        SRV_Channel::k_motor_tilt) * 100, 0, 100); 
}

bool AP_MarineICE::get_cmd_manual_engine_start() const
{
    static bool neutral_lock = constrain_int16((SRV_Channels::get_output_norm(
        SRV_Channel::k_starter) + 1) * 0.5, 0, 1) != 0;
    return neutral_lock;
}

bool AP_MarineICE::get_cmd_e_stop() const
{
    return SRV_Channels::get_emergency_stop();
}

uint8_t AP_MarineICE::get_current_mode() const { 
    return AP::vehicle()->get_mode(); 
}

bool AP_MarineICE::get_armed() const { 
    return hal.util->get_soft_armed(); 
}

bool AP_MarineICE::get_active_engine_stop() const
{
    return get_cmd_e_stop() || 
        !get_armed() || 
        (get_current_mode() == 4); // TODO: Use enum lookup. MODE 4 = HOLD for Rover
}

void AP_MarineICE::setup_states()
{
    _fsm_engine.register_state(EngineState::ENGINE_INIT, new State_Init());
    _fsm_engine.register_state(EngineState::ENGINE_RUN_NEUTRAL, new State_Run_Neutral());
    _fsm_engine.register_state(EngineState::ENGINE_RUN_FORWARD, new State_Run_Forward());
    _fsm_engine.register_state(EngineState::ENGINE_RUN_REVERSE, new State_Run_Reverse());
    _fsm_engine.register_state(EngineState::ENGINE_START, new State_Start());
    _fsm_engine.register_state(EngineState::ENGINE_START_WAIT, new State_Start_Wait());
    _fsm_engine.register_state(EngineState::ENGINE_FAULT, new State_Fault());

    _fsm_trim.register_state(TrimState::TRIM_MANUAL, new State_Trim_Manual());
    _fsm_trim.register_state(TrimState::TRIM_AUTO_STOP, new State_Trim_Auto_Stop());
    _fsm_trim.register_state(TrimState::TRIM_AUTO_UP, new State_Trim_Auto_Up());
    _fsm_trim.register_state(TrimState::TRIM_AUTO_DOWN, new State_Trim_Auto_Down());
}

void AP_MarineICE::monitor_faults()
{
    // Check for engine overspeed
    if (_backend->get_engine_data().rpm > _params.rpm_max.get())
    {
        set_fault(ENGINE_OVERSPEED, true);
    }

    // Check for engine overtemp
    if (_backend->get_engine_data().temp_degc > _params.temp_max.get())
    {
        set_fault(ENGINE_OVERTEMP, true);
    }

    // Check for throttle actuator failure - backend throttle percent is not within tolerance of commanded
    // taking into account the slew rate
    // float throttle_tolerance = _params.thr_slewrate.get() * MARINEICE_UPDATE_INTERVAL_MS / 1000.0f;
    float throttle_tolerance = 25.0;
    if (abs(_backend->get_throttle_pct() - get_cmd_throttle()) > throttle_tolerance)
    {
        set_fault(THROTTLE_ACTUATOR_FAILURE, true);
    }

    // TODO: Check for gear actuator failure

    // TODO: Check for alternator voltage low

    // Check for engine start attempts exceeded
    if (get_num_start_attempts() > _params.start_retries.get())
    {
        set_fault(ENGINE_START_ATTEMPTS_EXCEEDED, true);
    }
}

// Get the AP_MarineICE singleton
AP_MarineICE* AP_MarineICE::get_singleton()
{
    return _singleton;
}
AP_MarineICE* AP_MarineICE::_singleton = nullptr;

namespace AP {
AP_MarineICE* marineice()
{
    return AP_MarineICE::get_singleton();
}
};

#endif // HAL_MARINEICE_ENABLED
