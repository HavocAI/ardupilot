#pragma once

#include "AP_MarineICE_config.h"

#if HAL_MARINEICE_ENABLED

#include "AP_MarineICE.h"
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>

// Inherit from AP_ESC_Telem_Backend to provide telemetry backend functionality.

class AP_MarineICE_Backend : public AP_ESC_Telem_Backend {
    public:
        AP_MarineICE_Backend(AP_MarineICE_Params &params);
    
        // do not allow copies
        CLASS_NO_COPY(AP_MarineICE_Backend);
    
        // initialize driver
        virtual void init();
    
        // returns true if communicating with all required interfaces
        virtual bool healthy();

        // Gear Position corresponds to Dometic shift control byte
        enum class GearPosition {
            GEAR_FORWARD,
            GEAR_NEUTRAL,
            GEAR_REVERSE
        };

        enum class TrimCommand {
            TRIM_STOP,
            TRIM_UP,
            TRIM_DOWN
        };

        // Struct for engine data
        struct EngineData {
            uint16_t rpm;
            float alternator_voltage_v;
            float engine_time_hr;
            float temp_degc;
            float trim_deg;
            float fuel_rate_lpm;
            float fuel_used_l;
            float seasonal_fuel_used_l;
            float trip_fuel_used_l;
        };

        //** Virtual command setters, to be overridden by implementation
        virtual void set_cmd_throttle(uint16_t throttle_pct);
        virtual void set_cmd_gear(GearPosition gear);
        virtual void set_cmd_trim(TrimCommand trim);
        virtual void set_cmd_starter(bool enable);

        // Called in the main loop to update the ESC telemetry
        void update_esc_telemetry();

    protected:
        //** Parameter helper functions
        // get the instance of the range finder
        int8_t get_rng_fndr() const { return _params.rng_fndr.get(); }
    
        AP_MarineICE_Params &_params;    // parameters for this backend

        // Struct to encapsulate internal state variables
        struct State {
            EngineData      engine_data;
            GearPosition    gear_position = GearPosition::GEAR_NEUTRAL;
            uint16_t        throttle_pct = 0;
            bool            starter_on = false;
            TrimCommand     trim_command = TrimCommand::TRIM_STOP;
            float           water_depth_m = 0.0f;
        };

        // Current state (feedback)
        State _state;

    };

#endif // HAL_MARINEICE_ENABLED
