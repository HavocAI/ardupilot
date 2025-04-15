#include "AP_MarineICE_Backend.h"

#if HAL_MARINEICE_ENABLED

extern const AP_HAL::HAL& hal;

// constructor
AP_MarineICE_Backend::AP_MarineICE_Backend(AP_MarineICE_Params &params) :
    _params(params)
{}


void AP_MarineICE_Backend::update_esc_telemetry()
{
#if HAL_WITH_ESC_TELEM
    uint8_t telem_esc_index = 0;
    update_rpm(0, float(_state.engine_data.rpm));
    const AP_ESC_Telem_Backend::TelemetryData &telemetry_data = {
        .temperature_cdeg = _state.engine_data.temp_degc * 100,
        .voltage = _state.engine_data.alternator_voltage_v,
        .current = _state.engine_data.fuel_rate_lpm,
    };
    update_telem_data(telem_esc_index, telemetry_data,
                      AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE |
                      AP_ESC_Telem_Backend::TelemetryType::VOLTAGE |
                      AP_ESC_Telem_Backend::TelemetryType::CURRENT);
#endif // HAL_WITH_ESC_TELEM
}

#endif // HAL_MARINEICE_ENABLED
