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

#include "State_Run_Reverse.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_MarineICE/AP_MarineICE.h>
#include <AP_MarineICE/AP_MarineICE_Backend.h>

using namespace MarineICE::States;
using namespace MarineICE::Types;

void State_Run_Reverse::enter(AP_MarineICE& ctx) {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[MarineICE] RUN_REVERSE: Entering...");
}

void State_Run_Reverse::run(AP_MarineICE& ctx) {
    // Check for neutral lock
    if (ctx.get_cmd_neutral_lock()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "[MarineICE] Neutral lock engaged.");
        ctx.get_fsm_engine().change_state(EngineState::ENGINE_RUN_NEUTRAL, ctx);
        return;
    }

    // Check for RPM below threshold
    if (ctx.get_backend()->get_engine_data().rpm < ctx.get_params().rpm_thres.get()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "[MarineICE] RPM fell below idle in RUN_REVERSE.");
        ctx.get_fsm_engine().change_state(EngineState::ENGINE_RUN_NEUTRAL, ctx);
        return;
    }

    // Check for throttle command crossing zero deadband
    if (ctx.get_cmd_throttle() > -1 * ctx.get_params().thr_deadband.get()) {
        ctx.get_fsm_engine().change_state(EngineState::ENGINE_RUN_NEUTRAL, ctx);
        return;
    }

    // Set the throttle and gear commands
    ctx.get_backend()->set_cmd_throttle(abs(ctx.get_cmd_throttle()));
    ctx.get_backend()->set_cmd_gear(GearPosition::GEAR_REVERSE);
    ctx.get_backend()->set_cmd_ignition(true);
    ctx.get_backend()->set_cmd_starter(false);
}

void State_Run_Reverse::exit(AP_MarineICE& ctx) {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[MarineICE] RUN_REVERSE: Exiting...");
}
