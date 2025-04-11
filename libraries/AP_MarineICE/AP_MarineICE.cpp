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

AP_MarineICE::AP_MarineICE() {
    setup_states();
}

void AP_MarineICE::init() {
    _fsm.change_state(EngineState::STARTUP, *this);
}

void AP_MarineICE::update() {
    simulate_rpm();
    simulate_temp();

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

void AP_MarineICE::set_starter(bool enable) {
    starter_on = enable;
}

int AP_MarineICE::get_rpm() const {
    return mock_rpm;
}

float AP_MarineICE::get_temp() const {
    return mock_temp;
}

void AP_MarineICE::request_stop() {
    stop_requested = true;
}

// Simulation classes

// Mock simulation: increase RPM when starter is on
void AP_MarineICE::simulate_rpm() {
    if (starter_on) {
        mock_rpm += 100;
        if (mock_rpm > 600) mock_rpm = 600;
    } else {
        mock_rpm -= 50;
        if (mock_rpm < 0) mock_rpm = 0;
    }
}

void AP_MarineICE::simulate_temp() {
    if (_fsm.current_state() == EngineState::RUNNING) {
        mock_temp += 0.2f;
    } else {
        mock_temp -= 0.1f;
    }
    if (mock_temp < 20.0f) mock_temp = 20.0f;
    if (mock_temp > 100.0f) mock_temp = 100.0f;
}

#endif // HAL_MARINEICE_ENABLED
