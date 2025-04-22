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
#include <AP_MarineICE/AP_MarineICE.h>
#include <AP_MarineICE/AP_MarineICE_Backend.h>

using namespace MarineICE::States;
using namespace MarineICE::Types;

void State_Start::enter(AP_MarineICE &ctx)
{
    if (ctx.get_params().debug.get())
    {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[MarineICE] START: Entering...");
    }
    // Reset local variables
    _begin_starter_run_time = AP_HAL::millis();

    // Increment the number of start attempts stored by the backend
    ctx.get_backend()->set_num_start_attempts(ctx.get_backend()->get_num_start_attempts() + 1);
}

void State_Start::run(AP_MarineICE &ctx)
{
    // Check if the start command has been rescinded (auto or manual)
    if (!ctx.get_params().auto_start.get() && !ctx.get_cmd_manual_engine_start())
    {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[MarineICE] Engine start command rescinded.");
        ctx.get_fsm_engine().change_state(EngineState::ENGINE_RUN_NEUTRAL, ctx);
        return;
    }

    // Check for reaching the minimum RPM threshold
    if (ctx.get_backend()->get_engine_data().rpm >= ctx.get_params().rpm_thres.get())
    {
        // Engine started successfully
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[MarineICE] Engine started successfully.");
        ctx.get_fsm_engine().change_state(EngineState::ENGINE_RUN_NEUTRAL, ctx);
        ctx.get_backend()->set_num_start_attempts(0);
        // TODO: Should the engine be required to run for some amount of time before resetting the start attempts?
        return;
    }

    // Check if start time limit has been reached
    if ((AP_HAL::millis() - _begin_starter_run_time) > (ctx.get_params().start_time.get() * 1000))
    {
        // Engine start failed
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "[MarineICE] Engine start failed.");
        ctx.get_fsm_engine().change_state(EngineState::ENGINE_START_WAIT, ctx);
        return;
    }

    // Start the engine
    ctx.get_backend()->set_cmd_shift_throttle(GearPosition::GEAR_NEUTRAL, 0.0f);
    ctx.get_backend()->set_cmd_ignition(true);
    ctx.get_backend()->set_cmd_starter(true);
}

void State_Start::exit(AP_MarineICE &ctx)
{
    if (ctx.get_params().debug.get())
    {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[MarineICE] START: Exiting...");
    }
    // Disable the starter
    ctx.get_backend()->set_cmd_starter(false);
}
