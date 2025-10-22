#pragma once

#include "AP_GPS_config.h"

#if AP_GPS_NMEA2K_ENABLED

#include "AP_GPS.h"
#include "GPS_Backend.h"
#include <AP_NMEA2K/AP_NMEA2K.h>

class AP_GPS_NMEA2K : public AP_GPS_Backend {
public:
    AP_GPS_NMEA2K(AP_GPS &_gps, AP_GPS::Params &_params, AP_GPS::GPS_State &_state);
};


#endif  // AP_GPS_NMEA2K_ENABLED