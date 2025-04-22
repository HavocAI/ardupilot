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

#pragma once

#include "AP_MarineICE_config.h"

#if HAL_MARINEICE_ENABLED

#include <AP_Common/AP_Common.h>
#include "AP_MarineICE_Params.h"

#include "AP_MarineICE_Types.h"
#include "AP_MarineICE_States.h"
#include "StateMachine.h"
#include "State_Init.h"
#include "State_Run_Forward.h"
#include "State_Run_Neutral.h"
#include "State_Run_Reverse.h"
#include "State_Start.h"
#include "State_Start_Wait.h"
#include "State_Fault.h"
#include "State_Trim.h"

using namespace MarineICE::Types;
using namespace MarineICE::States;

// fwd declare backend classes
class AP_MarineICE_Backend;
class AP_MarineICE_Backend_Sim;
class AP_MarineICE_Backend_N2k;

class AP_MarineICE {

    // declare backends as friend
    friend class AP_MarineICE_Backend;
    friend class AP_MarineICE_Backend_Sim;
    friend class AP_MarineICE_Backend_N2k;

public:
    AP_MarineICE();

    CLASS_NO_COPY(AP_MarineICE);

    // Get singleton instance
    static AP_MarineICE* get_singleton();

    // Initialize driver
    void init();

    // Returns true if the backend is enabled
    bool enabled() const;

    // Returns true if the backend is receiving all required inputs recently
    bool healthy();

    // Run pre-arm check. Returns false on failure and fills in failure_msg
    // Any failure_msg returned will not include a prefix
    bool pre_arm_checks(char *failure_msg, uint8_t failure_msg_len);
  
    // Update the state machines and the telemetry
    void update();

    // State machine access
    StateMachine<EngineState, AP_MarineICE>& fsm();

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

    // Getters for FSMs
    StateMachine<EngineState, AP_MarineICE>& get_fsm_engine() { return _fsm_engine; }
    StateMachine<TrimState, AP_MarineICE>& get_fsm_trim() { return _fsm_trim; }

    // Get pointer to backend
    AP_MarineICE_Backend* get_backend() { return _backend; } const

    // Get reference to the frontend parameters
    AP_MarineICE_Params& get_params() { return _params; }

    // Get the neutral lock state from RELAY2
    // Returns true if the neutral lock is set
    bool get_cmd_neutral_lock() const;

    // Get the throttle command from k_throttle
    // Returns a percentage value between -100.0 and 100.0
    float get_cmd_throttle() const;

    // Get the auto trim setpoint command from RELAY3
    // Returns a percentage value between 0 and 100
    uint8_t get_cmd_trim_setpoint() const;

    // Get the manual trim command
    // Returns a TrimCommand enum value
    // TRIM_UP, TRIM_DOWN, or TRIM_STOP
    TrimCommand get_cmd_manual_trim() const;

    // Get the manual engine start command from RELAY
    // Returns true if the manual engine start is set
    bool get_cmd_manual_engine_start() const;

    // Get the state of the e-stop
    bool get_cmd_e_stop() const;

    // Get the current mode
    uint8_t get_current_mode() const;

    // Helper function to get whether the vehicle is armed
    bool get_armed() const;

    // Roll-up: Get whether there is an active engine stop condition
    bool get_active_engine_stop() const;

private:
    static AP_MarineICE* _singleton;

    AP_MarineICE_Backend* _backend;

    // Parameters for backends
    AP_MarineICE_Params _params;

    StateMachine<EngineState, AP_MarineICE> _fsm_engine;
    StateMachine<TrimState, AP_MarineICE> _fsm_trim;

    // Set up a state machine
    void setup_states();

};

namespace AP {
    AP_MarineICE* marineice();
};

#endif // HAL_MARINEICE_ENABLED
