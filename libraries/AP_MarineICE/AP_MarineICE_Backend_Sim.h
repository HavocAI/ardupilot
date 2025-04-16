#pragma once

#include "AP_MarineICE_config.h"

#if HAL_MARINEICE_ENABLED

#include "AP_MarineICE_Backend.h"

class AP_MarineICE_Backend_Sim : public AP_MarineICE_Backend {
public:
    // inherit constructor
    using AP_MarineICE_Backend::AP_MarineICE_Backend;

    // do not allow copies
    CLASS_NO_COPY(AP_MarineICE_Backend_Sim);

    // initialise driver
    void init() override;

    // returns true if communicating with all required interfaces
    bool healthy() override;

    void set_cmd_shift_throttle(GearPosition gear, float throttle_pct) override { 
        _cmd_throttle_pct = throttle_pct; 
        _cmd_gear = gear;
    };
    void set_cmd_trim(TrimCommand trim) override {
        _cmd_trim = trim;
    };
    void set_cmd_ignition(bool enable) override {
        _cmd_ignition = enable;
    };
    void set_cmd_starter(bool enable) override {
        _cmd_starter = enable;
    };

private:

    void thread_main();

    //** NMEA2000 data handlers

    // Reads the Suzuki NMEA2000 engine data
    void handle_suzuki_engine_feedback( void );

    // Reads the load channel feedback for trim and starter channel status
    void handle_load_channel_feedback( void );

    // Reads the water depth feedback from NMEA2000
    void handle_water_depth_feedback( void );

    //** Simulation functions
    void simulate_motor();
    void simulate_dometic_shift_throttle();
    void simulate_maretron_load_center();
    void simulate_water_depth();

    // Commands
    float _cmd_throttle_pct = 0.0f;
    GearPosition _cmd_gear = GearPosition::GEAR_NEUTRAL;
    TrimCommand _cmd_trim = TrimCommand::TRIM_STOP;
    bool _cmd_starter = false;
    bool _cmd_ignition = false;

};

#endif // HAL_MARINEICE_ENABLED
