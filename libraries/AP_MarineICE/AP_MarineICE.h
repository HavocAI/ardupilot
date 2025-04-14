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

#include <AP_Param/AP_Param.h>
#include "AP_MarineICE_Params.h"

#include "EngineState.h"
#include "StateMachine.h"

#include "States/State_Init.h"
#include "States/State_Run_Forward.h"
#include "States/State_Run_Neutral.h"
#include "States/State_Run_Reverse.h"
#include "States/State_Start.h"
#include "States/State_Start_Wait.h"
#include "States/State_Fault.h"

#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS.h>

// declare backend classes
class AP_MarineICE_Backend;

class AP_MarineICE {

    // declare backend as friend
    friend class AP_MarineICE_Backend;

public:
    AP_MarineICE();

    CLASS_NO_COPY(AP_MarineICE);

    // Get singleton instance
    static AP_MarineICE* get_singleton();
    
    // TYPE parameter values
    enum class BackendType : uint8_t {
        TYPE_DISABLED = 0,
        TYPE_SIMULATED = 1,
        TYPE_NMEA2000 = 2
    };

    // Initialize driver
    void init();

    // Returns true if the backend is enabled
    bool enabled() const;

    // Returns true if the backend is receiving all required inputs recently
    bool healthy();

    // Run pre-arm check. Returns false on failure and fills in failure_msg
    // Any failure_msg returned will not include a prefix
    bool pre_arm_checks(char *failure_msg, uint8_t failure_msg_len);
  
    // Update the state machines
    void update();

    // State machine access
    StateMachine<EngineState, AP_MarineICE>& fsm();

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

    // Parameters for backends
    AP_MarineICE_Params _params;

private:
    // Get pointer to backend
    AP_MarineICE_Backend* get_backend() const;
    // Pointer to backend
    AP_MarineICE_Backend* _backend;

    static AP_MarineICE* _singleton;

    StateMachine<EngineState, AP_MarineICE> _fsm;
    void setup_states();
};

namespace AP {
    AP_MarineICE* marineice();
};

#endif // HAL_MARINEICE_ENABLED
