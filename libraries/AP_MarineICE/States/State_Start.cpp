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

#include "State_Start.h"
#include <GCS_MAVLink/GCS.h>

void State_Start::enter(AP_MarineICE& ctx) {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[MarineICE] START: Starting engine...");
    
    // Reset local variables
    _start_time = AP_HAL::millis();
}

void State_Start::run(AP_MarineICE& ctx) {

    // Check if the start command has been rescinded (auto or manual)
    // TODO: Add the manual start command check
    if (!ctx.get_params().auto_start.get() ) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[MarineICE] Engine start command rescinded.");
        ctx.get_fsm_engine().change_state(EngineState::ENGINE_RUN_NEUTRAL, ctx);
        return;
    }

    // Check for reaching the minimum RPM threshold
    if (ctx.get_backend()->get_engine_data().rpm >= ctx.get_params().rpm_thres.get()) {
        // Engine started successfully
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[MarineICE] Engine started successfully.");
        ctx.get_fsm_engine().change_state(EngineState::ENGINE_RUN_NEUTRAL, ctx);
        return;
    }

    // Check if start time limit has been reached
    if ((AP_HAL::millis() - _start_time) > (ctx.get_params().start_time.get() * 1000)) {
        // Engine start failed
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "[MarineICE] Engine start failed.");
        ctx.get_fsm_engine().change_state(EngineState::ENGINE_START_WAIT, ctx);
        return;
    }

    // Start the engine
    ctx.get_backend()->set_cmd_throttle(0.0f);
    ctx.get_backend()->set_cmd_gear(GearPosition::GEAR_NEUTRAL);
    ctx.get_backend()->set_cmd_starter(true);

}

void State_Start::exit(AP_MarineICE& ctx) {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[MarineICE] EXIT: START\n");

    // Disable the starter
    ctx.get_backend()->set_cmd_throttle(0.0f);
    ctx.get_backend()->set_cmd_gear(GearPosition::GEAR_NEUTRAL);
    ctx.get_backend()->set_cmd_starter(false);

}
