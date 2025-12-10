#include <AP_Anello/AP_Anello.h>

#if HAL_ANELLO_ENABLED

#include <AP_NMEA2K/AP_NMEA2K.h>

const AP_Param::GroupInfo AP_Anello::var_info[] = {

    AP_GROUPINFO("ANELLO_GPS_EN", 1, AP_Anello, _gps_param, 0),

    AP_GROUPEND
};

AP_Anello::AP_Anello() 
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_Anello::update(void)
{
    AP_CANManager* can_manager = AP_CANManager::get_singleton();
    if (can_manager == nullptr) {
        return;
    }

    for (size_t i = 0; i < can_manager->get_num_drivers(); i++) {
        if (can_manager->get_driver_type(i) == AP_CAN::Protocol::NMEA2K) {
            AP_NMEA2K* driver = static_cast<AP_NMEA2K*>(can_manager->get_driver(i));
            

            // send the GPS enable/disable message
            nmea2k::N2KMessage msg;
            msg.SetPGN(129025);
            msg.AddByte(0x01);
            msg.AddByte(_gps_param ? 0x01 : 0x00);

            driver->send_message(msg);
        }
    }
}

#endif // HAL_ANELLO_ENABLED