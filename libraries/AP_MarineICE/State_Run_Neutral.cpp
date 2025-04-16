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

#include "State_Run_Neutral.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_MarineICE/AP_MarineICE.h>
#include <AP_MarineICE/AP_MarineICE_Backend.h>

using namespace MarineICE::States;
using namespace MarineICE::Types;

void State_Run_Neutral::enter(AP_MarineICE& ctx) {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[MarineICE] RUN_NEUTRAL: Entering...");
}

void State_Run_Neutral::run(AP_MarineICE& ctx) {
    // Check for throttle command exceeding positive deadband
    // and not in neutral lock
    if ((ctx.get_cmd_throttle() > ctx.get_params().thr_deadband.get()) &&
            !ctx.get_cmd_neutral_lock()) {
        ctx.get_fsm_engine().change_state(EngineState::ENGINE_RUN_FORWARD, ctx);
        return;
    }

    // Check for throttle command exceeding negative deadband
    // and not in neutral lock
    if ((ctx.get_cmd_throttle() < -1 * ctx.get_params().thr_deadband.get()) &&
            !ctx.get_cmd_neutral_lock()) {
        ctx.get_fsm_engine().change_state(EngineState::ENGINE_RUN_REVERSE, ctx);
        return;
    }

    // Check for RPM below idle threshold
    if (ctx.get_backend()->get_engine_data().rpm < ctx.get_params().rpm_thres.get()) {
        // GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "[MarineICE] RPM below idle in RUN_NEUTRAL.");
        if ((ctx.get_cmd_manual_engine_start() || (ctx.get_current_mode() != 0 && ctx.get_params().auto_start.get())) &&
            (ctx.get_backend()->get_water_depth_m() >= ctx.get_params().water_depth_thres.get())) {
            // Attempt to start the engine
            ctx.get_fsm_engine().change_state(EngineState::ENGINE_START, ctx);
        }
        return;
    }

    // Set the throttle and gear commands
    ctx.get_backend()->set_cmd_shift_throttle(GearPosition::GEAR_NEUTRAL, 0.0f);
    ctx.get_backend()->set_cmd_ignition(true);
    ctx.get_backend()->set_cmd_starter(false);

}

void State_Run_Neutral::exit(AP_MarineICE& ctx) {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[MarineICE] RUN_NEUTRAL: Exiting...");
}
