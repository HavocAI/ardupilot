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

void State_Fault::enter(AP_MarineICE &ctx)
{
    if (ctx.get_params().debug.get())
    {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[MarineICE] FAULT: Entering...");
    }
    _entry_time_ms = AP_HAL::millis();
}

void State_Fault::run(AP_MarineICE &ctx)
{
    // Set "safe" commands for the engine
    ctx.get_backend()->set_cmd_shift_throttle(GearPosition::GEAR_NEUTRAL, 0.0f);
    ctx.get_backend()->set_cmd_ignition(false);
    ctx.get_backend()->set_cmd_starter(false);

    if (AP_HAL::millis() - _last_printout_ms > 1000)
    {
        // Print the active faults every second
        for (int i = 0; i < 6; ++i)
        {
            if (ctx.get_backend()->get_fault(static_cast<FaultIndex>(i)))
            {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "[MarineICE] %s", fault_to_string(static_cast<FaultIndex>(i)));
            }
        }
        _last_printout_ms = AP_HAL::millis();
    }

    if (!ctx.get_armed() &&
        (AP_HAL::millis() > (_entry_time_ms + 1000))) // Clear faults if not armed for 1 sec
    {
        // TODO: Is disarm the best way to clear faults and return to Init?
        // Clear faults and reset the number of start attempts
        ctx.get_backend()->clear_faults();
        ctx.get_backend()->set_num_start_attempts(0);

        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[MarineICE] Faults and start attempts cleared");
        ctx.get_fsm_engine().change_state(EngineState::ENGINE_INIT, ctx);
    }
}

void State_Fault::exit(AP_MarineICE &ctx)
{
    if (ctx.get_params().debug.get())
    {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[MarineICE] FAULT: Exiting...");
    }
}
