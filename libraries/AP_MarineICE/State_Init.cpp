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

#include "State_Init.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_MarineICE/AP_MarineICE.h>
#include <AP_MarineICE/AP_MarineICE_Backend.h>

using namespace MarineICE::States;
using namespace MarineICE::Types;

void State_Init::enter(AP_MarineICE& ctx) {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[MarineICE] INIT: Entering...");
}

void State_Init::run(AP_MarineICE& ctx) {

    // Set "safe" commands for the engine
    ctx.get_backend()->set_cmd_shift_throttle(GearPosition::GEAR_NEUTRAL, 0.0f);
    ctx.get_backend()->set_cmd_ignition(false);
    ctx.get_backend()->set_cmd_starter(false);

    // If there is no engine stop condition and the water depth is above the threshold, 
    // change to ENGINE_RUN_NEUTRAL state
    if (!ctx.get_active_engine_stop() && 
        (ctx.get_backend()->get_water_depth_m() >= ctx.get_params().water_depth_thres.get())) 
    {
        ctx.get_fsm_engine().change_state(EngineState::ENGINE_RUN_NEUTRAL, ctx);
    }
}

void State_Init::exit(AP_MarineICE& ctx) {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[MarineICE] INIT: Exiting...");
}
