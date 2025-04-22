#include "State_Trim.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_MarineICE/AP_MarineICE.h>
#include <AP_MarineICE/AP_MarineICE_Backend.h>

using namespace MarineICE::States;
using namespace MarineICE::Types;

#define MARINEICE_TRIM_DEADBAND 10

// TRIM MANUAL

void State_Trim_Manual::enter(AP_MarineICE &ctx)
{
    if (ctx.get_params().debug.get())
    {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[MarineICE] TRIM_MANUAL: Entering...");
    }
}

void State_Trim_Manual::run(AP_MarineICE &ctx)
{

    // Check if auto_trim has been enabled and mode is not MANUAL
    if (ctx.get_params().auto_trim.get() && ctx.get_current_mode() != 0)
    {
        ctx.get_fsm_trim().change_state(TrimState::TRIM_AUTO_STOP, ctx);
        return;
    }

    ctx.get_backend()->set_cmd_trim(ctx.get_cmd_manual_trim());
}

void State_Trim_Manual::exit(AP_MarineICE &ctx)
{
    if (ctx.get_params().debug.get())
    {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[MarineICE] TRIM_MANUAL: Exiting...");
    }
}

// AUTO STOP

void State_Trim_Auto_Stop::enter(AP_MarineICE &ctx) {}

void State_Trim_Auto_Stop::run(AP_MarineICE &ctx)
{

    // Check if auto_trim has been disabled or mode is MANUAL
    if (!ctx.get_params().auto_trim.get() || ctx.get_current_mode() == 0)
    {
        ctx.get_fsm_trim().change_state(TrimState::TRIM_MANUAL, ctx);
        return;
    }

    // If not in engine stop condition, check for trim commands exceeding deadband
    if (!ctx.get_active_engine_stop())
    {
        // Check for trim command exceeding deadband
        if (ctx.get_cmd_trim_setpoint() >
            (ctx.get_backend()->get_engine_data().trim_pct + MARINEICE_TRIM_DEADBAND))
        {
            ctx.get_fsm_trim().change_state(TrimState::TRIM_AUTO_UP, ctx);
            return;
        }
        if (ctx.get_cmd_trim_setpoint() <
            (ctx.get_backend()->get_engine_data().trim_pct - MARINEICE_TRIM_DEADBAND))
        {
            ctx.get_fsm_trim().change_state(TrimState::TRIM_AUTO_DOWN, ctx);
            return;
        }
    }

    // Set the trim command
    ctx.get_backend()->set_cmd_trim(TrimCommand::TRIM_STOP);
}

void State_Trim_Auto_Stop::exit(AP_MarineICE &ctx) {}

// AUTO UP

void State_Trim_Auto_Up::enter(AP_MarineICE &ctx) {}

void State_Trim_Auto_Up::run(AP_MarineICE &ctx)
{

    // Check if auto_trim has been disabled or mode is MANUAL
    if (!ctx.get_params().auto_trim.get() || ctx.get_current_mode() == 0)
    {
        ctx.get_fsm_trim().change_state(TrimState::TRIM_MANUAL, ctx);
        return;
    }

    // Check for engine stop condition
    if (ctx.get_active_engine_stop())
    {
        ctx.get_fsm_trim().change_state(TrimState::TRIM_AUTO_STOP, ctx);
        return;
    }

    // Check if the trim position has been reached
    if (ctx.get_backend()->get_engine_data().trim_pct >=
        (ctx.get_cmd_trim_setpoint() - MARINEICE_TRIM_DEADBAND))
    {
        ctx.get_fsm_trim().change_state(TrimState::TRIM_AUTO_STOP, ctx);
        return;
    }

    // Set the trim command
    ctx.get_backend()->set_cmd_trim(TrimCommand::TRIM_UP);
}

void State_Trim_Auto_Up::exit(AP_MarineICE &ctx) {}

// AUTO DOWN

void State_Trim_Auto_Down::enter(AP_MarineICE &ctx) {}

void State_Trim_Auto_Down::run(AP_MarineICE &ctx)
{

    // Check if auto_trim has been disabled or mode is MANUAL
    if (!ctx.get_params().auto_trim.get() || ctx.get_current_mode() == 0)
    {
        ctx.get_fsm_trim().change_state(TrimState::TRIM_MANUAL, ctx);
        return;
    }

    // Check for engine stop condition
    if (ctx.get_active_engine_stop())
    {
        ctx.get_fsm_trim().change_state(TrimState::TRIM_AUTO_STOP, ctx);
        return;
    }

    // Check if the trim position has been reached
    if (ctx.get_backend()->get_engine_data().trim_pct <=
        (ctx.get_cmd_trim_setpoint() + MARINEICE_TRIM_DEADBAND))
    {
        ctx.get_fsm_trim().change_state(TrimState::TRIM_AUTO_STOP, ctx);
        return;
    }

    // Set the trim command
    ctx.get_backend()->set_cmd_trim(TrimCommand::TRIM_DOWN);
}

void State_Trim_Auto_Down::exit(AP_MarineICE &ctx) {}
