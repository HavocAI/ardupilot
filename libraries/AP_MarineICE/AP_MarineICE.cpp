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

AP_MarineICE::AP_MarineICE() {
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
    
    // setup the state machine
    setup_states();
}

void AP_MarineICE::init() {

    // check init has not been called before
    if (_backend != nullptr) {
        return;
    }

    // Set up the backend
    switch ((BackendType)_params.type.get()) {
        case BackendType::TYPE_DISABLED:
            // do nothing
            break;
        case BackendType::TYPE_SIMULATED:
            _backend = NEW_NOTHROW AP_MarineICE_Simulator(_params);
            break;
        case BackendType::TYPE_NMEA2000:
            // TODO: implement NMEA2000 backend
            break;
        default:
            // do nothing
            break;
        }

    _backend->init();

    _fsm.change_state(EngineState::INIT, *this);
}

// returns true if a backend has been configured (e.g. TYPE param has been set)
bool AP_MarineICE::enabled() const {
    switch ((BackendType)_params.type.get()) {
        case BackendType::TYPE_DISABLED:
            return false;
        case BackendType::TYPE_SIMULATED:
            return true;
        case BackendType::TYPE_NMEA2000:
            return true;
        default:
            return false;
        }
    return false;
}

bool AP_MarineICE::healthy() {
    if (_backend == nullptr) {
        return false;
    }
    return _backend->healthy();
}

bool AP_MarineICE::pre_arm_checks(char *failure_msg, uint8_t failure_msg_len) {
    // exit immediately if not enabled
    if (!enabled()) {
        return true;
    }

    // check if the backend is healthy
    if (!healthy()) {
        strncpy(failure_msg, "not healthy", failure_msg_len);
        return false;
    }
    return true;
}

void AP_MarineICE::update() {


    _fsm.update(*this);
}

void AP_MarineICE::setup_states() {
    _fsm.register_state(EngineState::INIT,    new State_Init());
    _fsm.register_state(EngineState::RUN_NEUTRAL, new State_Run_Neutral());
    _fsm.register_state(EngineState::RUN_FORWARD, new State_Run_Forward());
    _fsm.register_state(EngineState::RUN_REVERSE, new State_Run_Reverse());
    _fsm.register_state(EngineState::START, new State_Start());
    _fsm.register_state(EngineState::START_WAIT, new State_Start_Wait());
    _fsm.register_state(EngineState::FAULT,   new State_Fault());
}

StateMachine<EngineState, AP_MarineICE>& AP_MarineICE::fsm() {
    return _fsm;
}

#endif // HAL_MARINEICE_ENABLED
