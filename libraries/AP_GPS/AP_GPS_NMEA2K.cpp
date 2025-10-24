#include "AP_GPS_NMEA2K.h"

AP_GPS_NMEA2K::AP_GPS_NMEA2K(AP_GPS &_gps, AP_GPS::Params &_params, AP_GPS::GPS_State &_state) :
    AP_GPS_Backend(_gps, _params, _state, nullptr)
{
}

AP_GPS_NMEA2K* AP_GPS_NMEA2K::probe(AP_GPS &_gps, AP_GPS::Params &_params, AP_GPS::GPS_State &_state)
{
    // find a NMEA2K CAN driver instance from the CANManager
    AP_CANManager* can_manager = AP_CANManager::get_singleton();
    AP_NMEA2K* nmea2k = nullptr;
    for (size_t i = 0; i < can_manager->get_num_drivers(); i++) {
        if (can_manager->get_driver_type(i) == AP_CAN::Protocol::NMEA2K) {
            nmea2k = static_cast<AP_NMEA2K*>(can_manager->get_driver(i));
            break;
        }
    }

    // create GPS backend
    AP_GPS_NMEA2K* backend = NEW_NOTHROW AP_GPS_NMEA2K(_gps, _params, _state);
    if (backend == nullptr) {
        return nullptr;
    }


    // register NMEA2K message handler
    AP_NMEA2K::NMEA2K_HandleN2KMessage_Functor handle_n2k_message = FUNCTOR_BIND(backend, &AP_GPS_NMEA2K::handle_nmea2k_message, void, AP_NMEA2K*, nmea2k::N2KMessage&);
    nmea2k->register_handle_n2k_message(handle_n2k_message);

    return backend;
}

void AP_GPS_NMEA2K::handle_nmea2k_message(AP_NMEA2K* nmea2k_instance, nmea2k::N2KMessage& msg)
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


