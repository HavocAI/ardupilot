#pragma once

#include "AP_MarineICE_config.h"

#if HAL_MARINEICE_ENABLED

#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>
#include "AP_MarineICE_Params.h"
#include "AP_MarineICE_Types.h"
#include <array>

using namespace MarineICE::Types;

// Inherit from AP_ESC_Telem_Backend to provide telemetry backend functionality.

class AP_MarineICE_Backend : public AP_ESC_Telem_Backend {
public:
    AP_MarineICE_Backend(AP_MarineICE_Params &params);

    // do not allow copies
    CLASS_NO_COPY(AP_MarineICE_Backend);

    // initialize driver
    virtual void init() = 0;

    // Check for fault conditions and set _status.faults
    // This function should be called in the main loop
    virtual void monitor_faults() = 0;

    // Struct for engine data
    struct EngineData {
        uint16_t rpm;
        float alternator_voltage_v;
        float engine_time_hr;
        float temp_degc;
        float trim_pct;                 // trim percentage 0-100 (0 is full down)
        uint16_t total_engine_hours;
        GearPosition transmission_gear;
        float fuel_rate_lpm;
        float trip_fuel_used_l;
    };

    //** Virtual command setters, to be overridden by implementation
    virtual void set_cmd_shift_throttle(GearPosition gear, float throttle_pct) = 0;
    virtual void set_cmd_trim(TrimCommand trim) = 0;
    virtual void set_cmd_ignition(bool enable) = 0;
    virtual void set_cmd_starter(bool enable) = 0;

    // Called in the main loop to update the ESC telemetry
    void update_esc_telemetry();

    // Getters for passing the internal state to the frontend / state machine
    const EngineData& get_engine_data() const { return _state.engine_data; }
    const GearPosition& get_actuator_gear_position() const { return _state.actuator_gear_position; }
    const float& get_actuator_throttle_pct() const { return _state.actuator_throttle_pct; }
    const bool& get_starter_on() const { return _state.starter_on; }
    const TrimCommand& get_trim_command() const { return _state.trim_command; }
    const float& get_water_depth_m() const { return _state.water_depth_m; }

    // Get the current health status of the backend
    // Returns true if no faults are set
    bool healthy() const;

    // Clear all active fault conditions
    // Note: Does not clear the number of start attempts
    void clear_faults() { 
        _status.faults.fill(false); 
    }

    // Get a fault condition in the array
    bool get_fault(FaultIndex fault) const { 
        return (fault < NUM_FAULTS) ? _status.faults[fault] : false; 
    }
    
    // Set the number of start attempts
    void set_num_start_attempts(uint8_t attempts) { _status.num_start_attempts = attempts; }

    // Get the number of start attempts
    uint8_t get_num_start_attempts() const { return _status.num_start_attempts; }

protected:
    AP_MarineICE_Params &_params; // parameters for this backend

    // Struct to encapsulate internal state variables
    struct State {
        EngineData      engine_data;
        GearPosition    actuator_gear_position = GearPosition::GEAR_NEUTRAL;
        float           actuator_throttle_pct = 0.0f;
        bool            starter_on = false;
        bool            ignition_on = false;
        TrimCommand     trim_command = TrimCommand::TRIM_STOP;
        float           water_depth_m = 0.0f;
    };

    // Current state (feedback)
    State _state;

    struct Status {
        // Fault as an array of bools (due to issues using bitset on stm32)
        std::array<bool, NUM_FAULTS> faults = {false};
        uint8_t num_start_attempts;
    };

    // Current status (faults)
    Status _status;

    // Set a fault condition in the array
    void set_fault(FaultIndex fault, bool state);

};

#endif // HAL_MARINEICE_ENABLED
