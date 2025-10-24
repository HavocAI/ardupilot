#include "AP_GPS_NMEA2K.h"

AP_GPS_NMEA2K::AP_GPS_NMEA2K(AP_GPS &_gps, AP_GPS::Params &_params, AP_GPS::GPS_State &_state) :
    AP_GPS_Backend(_gps, _params, _state, nullptr)
{
}

bool AP_GPS_NMEA2K::read()
{
    state = _interim_state;

    // return true when successfully received a valid packet from the GPS
    return true;
}

bool AP_GPS_NMEA2K::is_healthy(void) const
{
    // NMEA2000 GPS health check not yet implemented
    return true;
}


