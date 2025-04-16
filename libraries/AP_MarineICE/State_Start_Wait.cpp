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

#include "State_Start_Wait.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_MarineICE/AP_MarineICE.h>
#include <AP_MarineICE/AP_MarineICE_Backend.h>

using namespace MarineICE::States;
using namespace MarineICE::Types;

void State_Start_Wait::enter(AP_MarineICE& ctx) {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[MarineICE] START_WAIT: Entering...");

    // Reset local variables
    _begin_starter_wait_time = AP_HAL::millis();

}

void State_Start_Wait::run(AP_MarineICE& ctx) {
    // Check if the start command has been rescinded (auto or manual)
    if (!ctx.get_params().auto_start.get() && !ctx.get_cmd_manual_engine_start()) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[MarineICE] Engine start command rescinded.");
        ctx.get_fsm_engine().change_state(EngineState::ENGINE_RUN_NEUTRAL, ctx);
        return;
    }

    // Check if the wait time limit has been reached
    if ((AP_HAL::millis() - _begin_starter_wait_time) >= (ctx.get_params().start_delay.get() * 1000)) {
        ctx.get_fsm_engine().change_state(EngineState::ENGINE_START, ctx);
        return;
    }

    // Wait to start the engine
    ctx.get_backend()->set_cmd_shift_throttle(GearPosition::GEAR_NEUTRAL, 0.0f);
    ctx.get_backend()->set_cmd_ignition(true);
    ctx.get_backend()->set_cmd_starter(false);
}

void State_Start_Wait::exit(AP_MarineICE& ctx) {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[MarineICE] START_WAIT: Exiting...");
}
