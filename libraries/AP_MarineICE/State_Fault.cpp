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

#include "State_Fault.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_MarineICE/AP_MarineICE.h>
#include <AP_MarineICE/AP_MarineICE_Backend.h>

using namespace MarineICE::States;
using namespace MarineICE::Types;

void State_Fault::enter(AP_MarineICE& ctx) {
    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "[MarineICE] FAULT: Entering...");

    // Print the active faults
    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "[MarineICE] Active Faults:");
    for (int i = 0; i < 6; ++i)
    {
        if (ctx.get_fault(static_cast<FaultIndex>(i)))
        {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s", fault_to_string(static_cast<FaultIndex>(i)));
        }
    }
}

void State_Fault::run(AP_MarineICE& ctx) {
    // Set "safe" commands for the engine
    ctx.get_backend()->set_cmd_shift_throttle(GearPosition::GEAR_NEUTRAL, 0.0f);
    ctx.get_backend()->set_cmd_ignition(false);
    ctx.get_backend()->set_cmd_starter(false);

    // TODO: Is disarm the best way to clear faults and return to Init?
    if (!ctx.get_armed()) // Clear faults if not armed
    {
        for (int i = 0; i < 6; ++i)
        {
            ctx.set_fault(static_cast<FaultIndex>(i), false);
        }
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[MarineICE] FAULT: All faults cleared.");
        ctx.get_fsm_engine().change_state(EngineState::ENGINE_INIT, ctx);
    }

}

void State_Fault::exit(AP_MarineICE& ctx) {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[MarineICE] FAULT: Exiting...");
}
