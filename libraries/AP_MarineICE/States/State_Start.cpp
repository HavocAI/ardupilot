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
#include "../AP_MarineICE.h"
#include "../EngineState.h"

void State_Start::enter(AP_MarineICE& ctx) {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[MarineICE] START: Starting engine...");

}

void State_Start::run(AP_MarineICE& ctx) {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[MarineICE] START\n");

    // Start the engine
    ctx.set_starter(true);
    ctx.set_gear(GearPosition::NEUTRAL);
    ctx.set_throttle(0.0f);

    // Transition to the next state
    ctx.get_state_machine().change_state(EngineState::RUN_NEUTRAL, ctx);
}

void State_Start::exit(AP_MarineICE& ctx) {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[MarineICE] EXIT: START\n");

    // Stop starter motor
    ctx.set_starter(false);
}
