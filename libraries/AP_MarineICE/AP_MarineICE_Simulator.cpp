#include "AP_MarineICE_Simulator.h"

#if HAL_MARINEICE_ENABLED

#include <AP_Math/AP_Math.h>
#include <algorithm>
#include <SRV_Channel/SRV_Channel.h>

#define MARINEICE_SIMULATOR_RATE_HZ 10
#define MARINEICE_SIMULATOR_MAX_RPM 6000
#define MARINEICE_SIMULATOR_MAX_TRIM 30.0f
#define MARINEICE_SIMULATOR_MIN_TRIM 0.0f

extern const AP_HAL::HAL& hal;

using namespace MarineICE::Types;

void AP_MarineICE_Simulator::init()
{
    // Create background thread to run the simulation
    // Use the scripting priority
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_MarineICE_Simulator::thread_main, void), "marineice_besim", 2048, AP_HAL::Scheduler::PRIORITY_SCRIPTING, 1)) {
        return;
    }
}

bool AP_MarineICE_Simulator::healthy()
{
    // Simulated backend is always healthy
    return true;
}

// thread main function
void AP_MarineICE_Simulator::thread_main()
{
    // Simulate the engine and other components
    while (true) {
        hal.scheduler->delay(1000 / MARINEICE_SIMULATOR_RATE_HZ);

        update_esc_telemetry();

        // Simulate engine RPM and temperature
        simulate_motor();
        simulate_dometic_shift_throttle();
        simulate_maretron_load_center();
        simulate_water_depth();
    }
}

void AP_MarineICE_Simulator::simulate_motor() {
    if (SRV_Channels::get_emergency_stop() || !_state.ignition_on) {
        // Simulate engine stopping if e-stop is pressed or ignition is off
        // This simulates an E-stop button that is also wired directly to the engine Ign/Kill circuit
        _state.engine_data.rpm = std::max(static_cast<float>((_state.engine_data.rpm - 300) / MARINEICE_SIMULATOR_RATE_HZ), 
            0.0f);
    } else if ((_state.engine_data.rpm < (_params.rpm_thres.get() - 100)) && !_state.starter_on) {
        // Simulate engine stalling if below min RPM and starter is off
        _state.engine_data.rpm = std::max(static_cast<float>((_state.engine_data.rpm - 300) / MARINEICE_SIMULATOR_RATE_HZ), 
            0.0f);
    } else if (_state.starter_on && _state.gear_position == GearPosition::GEAR_NEUTRAL) {
        // Simulate starting and enforce the Neutral interlock
        // Increment the engine RPM until it reaches the threshold
        if (_state.engine_data.rpm < _params.rpm_thres.get()) {
            _state.engine_data.rpm += 300 / MARINEICE_SIMULATOR_RATE_HZ; // Increment RPM
        }
    } else {
        // Simulate engine running based on throttle percentage, with a minimum idle threshold
        _state.engine_data.rpm = std::max((_state.throttle_pct * MARINEICE_SIMULATOR_MAX_RPM / 100.0f), 
            static_cast<float>(_params.rpm_thres.get()));
    }

    // Simulate alternator voltage based on RPM
    if (_state.engine_data.rpm > _params.rpm_thres.get()) {
        _state.engine_data.alternator_voltage_v = 12.0f + (_state.engine_data.rpm / MARINEICE_SIMULATOR_MAX_RPM) * 2.0f;
    } else {
        _state.engine_data.alternator_voltage_v = 0.0f;
    }

    // Simulate engine time_hr
    if (_state.engine_data.rpm > 0) {
        _state.engine_data.engine_time_hr += (1.0f / MARINEICE_SIMULATOR_RATE_HZ) / 3600.0f;
    }
    
    // Simulate engine temp based on RPM
    if (_state.engine_data.rpm > 0) {
        _state.engine_data.temp_degc = 20.0f + (_state.engine_data.rpm / MARINEICE_SIMULATOR_MAX_RPM) * 80.0f;
    } else {
        _state.engine_data.temp_degc = 15.0f; // Default temperature when engine is off
    }

    // Simulate engine trim based on trim command
    if ((_state.trim_command == TrimCommand::TRIM_UP) && (_state.engine_data.trim_deg < MARINEICE_SIMULATOR_MAX_TRIM)) {
        _state.engine_data.trim_deg += 5.0f / MARINEICE_SIMULATOR_RATE_HZ; // Simulate trim up
    } else if ((_state.trim_command == TrimCommand::TRIM_DOWN) && (_state.engine_data.trim_deg > MARINEICE_SIMULATOR_MIN_TRIM)) {
        _state.engine_data.trim_deg -= 5.0f / MARINEICE_SIMULATOR_RATE_HZ; // Simulate trim down
    }

    // Simulate engine fuel rate based on RPM
    if (_state.engine_data.rpm > 0) {
        _state.engine_data.fuel_rate_lpm = 0.1f + (_state.engine_data.rpm / MARINEICE_SIMULATOR_MAX_RPM) * 0.5f;
    } else {
        _state.engine_data.fuel_rate_lpm = 0.0f;
    }

    // Simulate engine fuel used
    if (_state.engine_data.rpm > 0) {
        _state.engine_data.fuel_used_l += _state.engine_data.fuel_rate_lpm / (MARINEICE_SIMULATOR_RATE_HZ * 60.0f);
    }

    // Simulate engine seasonal fuel used
    _state.engine_data.seasonal_fuel_used_l = _state.engine_data.fuel_used_l;

    // Simulate engine trip fuel used
    _state.engine_data.trip_fuel_used_l = _state.engine_data.fuel_used_l;

}

void AP_MarineICE_Simulator::simulate_dometic_shift_throttle() {
    // Simulate shift gear feedback by applying a delay at gear change
    if (_cmd_gear != _state.gear_position) {
        // Simulate delay of 1 sec for gear change (blocking)
        hal.scheduler->delay(1000);
        _state.gear_position = _cmd_gear;
    }

    // Simulate throttle percent feedback by applying the slew rate to the command
    if (_cmd_throttle_pct > 100) {
        _cmd_throttle_pct = 100; // Clamp to 100%
    }
    // Apply slew rate to throttle percentage
    if (_cmd_throttle_pct > _state.throttle_pct) {
        _state.throttle_pct += _params.thr_slewrate.get() / MARINEICE_SIMULATOR_RATE_HZ;
    } else if (_cmd_throttle_pct < _state.throttle_pct) {
        _state.throttle_pct -= _params.thr_slewrate.get() / MARINEICE_SIMULATOR_RATE_HZ;
    }

}

void AP_MarineICE_Simulator::simulate_maretron_load_center() {
    // Simulate load center feedback responding to trim up/down
    if (_cmd_trim == TrimCommand::TRIM_UP) {
        _state.trim_command = TrimCommand::TRIM_UP;
    } else if (_cmd_trim == TrimCommand::TRIM_DOWN) {
        _state.trim_command = TrimCommand::TRIM_DOWN;
    } else {
        _state.trim_command = TrimCommand::TRIM_STOP;
    }

    // Simulate load center feedback responding to engine start
    if (_cmd_starter) {
        _state.starter_on = true; // Simulate starter on
    } else {
        _state.starter_on = false; // Simulate starter off
    }

    // Simulate load center feedback responding to ignition
    if (_cmd_ignition) {
        _state.ignition_on = true; // Simulate ignition on
    } else {
        _state.ignition_on = false; // Simulate ignition off
    }
}

void AP_MarineICE_Simulator::simulate_water_depth() {
    // Simulate water depth feedback from rangefinder
    _state.water_depth_m = 3.0f; // Simulate a constant water depth of 3 meters
}

#endif // HAL_MARINEICE_ENABLED
