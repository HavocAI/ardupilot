#pragma once

#include "AP_GPS_config.h"

#if AP_GPS_NMEA2K_ENABLED

#include "AP_GPS.h"
#include "GPS_Backend.h"
#include <AP_NMEA2K/AP_NMEA2K.h>

class AP_GPS_NMEA2K : public AP_GPS_Backend {
public:
    AP_GPS_NMEA2K(AP_GPS &_gps, AP_GPS::Params &_params, AP_GPS::GPS_State &_state);

    static AP_GPS_NMEA2K* probe(AP_GPS &_gps, AP_GPS::Params &_params, AP_GPS::GPS_State &_state);

    bool read() override;

    const char* name() const override { return "NMEA2K"; }

    bool is_healthy(void) const override;

private:
    AP_GPS::GPS_State _interim_state;
    uint32_t _last_msg_time_ms;
    bool _new_data;
    HAL_Semaphore sem;

    void handle_nmea2k_message(AP_NMEA2K* nmea2k_instance, nmea2k::N2KMessage& msg);
};



#endif  // AP_GPS_NMEA2K_ENABLED