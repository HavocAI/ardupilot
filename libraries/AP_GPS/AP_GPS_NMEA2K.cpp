#include "AP_GPS_NMEA2K.h"

AP_GPS_NMEA2K::AP_GPS_NMEA2K(AP_GPS &_gps, AP_GPS::Params &_params, AP_GPS::GPS_State &_state) :
    AP_GPS_Backend(_gps, _params, _state, nullptr)
{
}