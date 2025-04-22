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
#include "AP_MarineICE_Backend_N2k.h"
#include "AP_MarineICE_Backend_Sim.h"

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
        _backend = NEW_NOTHROW AP_MarineICE_Backend_Sim(_params);
        break;
    case BackendType::BACKEND_TYPE_NMEA2000:
        _backend = NEW_NOTHROW AP_MarineICE_Backend_N2k(_params);
        break;
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
    if (_backend == nullptr)
    {
        return;
    }
    // Check for faults
    _backend->monitor_faults();

    if (!healthy() &&
        _fsm_engine.current_state_id() != EngineState::ENGINE_FAULT) {
        // If not healthy, change to the fault state
        _fsm_engine.change_state(EngineState::ENGINE_FAULT, *this);
    } else if ((get_active_engine_stop() || (_backend->get_engine_data().trim_pct > _params.safe_trim)) && 
            _fsm_engine.current_state_id() != EngineState::ENGINE_INIT &&
            _fsm_engine.current_state_id() != EngineState::ENGINE_FAULT) {
        // If there is an engine stop condition or unsafe trim, change to the init state
        _fsm_engine.change_state(EngineState::ENGINE_INIT, *this);
    }

    // Update the state machines
    _fsm_engine.update(*this);
    _fsm_trim.update(*this);

    // Update the telemetry (published from backend to GCS)
    _backend->update_esc_telemetry();
}

bool AP_MarineICE::get_cmd_neutral_lock() const
{
    return (rc().find_channel_for_option(RC_Channel::AUX_FUNC::RELAY2)->
        norm_input()) > 0.5f;
}

float AP_MarineICE::get_cmd_throttle() const { 
    return SRV_Channels::get_output_norm(SRV_Channel::k_throttle) * _params.thr_max.get(); 
}

// Get the auto trim command as a percentage value between 0 and 100
uint8_t AP_MarineICE::get_cmd_trim_setpoint() const { 
    return static_cast<uint8_t>((rc().find_channel_for_option(RC_Channel::AUX_FUNC::RELAY3)->
        norm_input() + 1) * 50);
}

// Get the manual trim command as either up or down
TrimCommand AP_MarineICE::get_cmd_manual_trim() const { 
    float trim = rc().find_channel_for_option(RC_Channel::AUX_FUNC::RELAY4)->
        norm_input();
    if (trim < -0.5f) {
        return TrimCommand::TRIM_DOWN;
    } else if (trim > 0.5f) {
        return TrimCommand::TRIM_UP;
    } else {
        return TrimCommand::TRIM_STOP;
    }
}

bool AP_MarineICE::get_cmd_manual_engine_start() const
{
    return (rc().find_channel_for_option(RC_Channel::AUX_FUNC::RELAY)->
        norm_input()) > 0.5f;
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
