#include <AP_NMEA2K/AP_NMEA2K.h>

#include <AP_CANManager/AP_CANManager.h>
#include <AP_J1939_CAN/AP_J1939_CAN.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>

#include <AP_NMEA2K/AP_NMEA2K_msg.h>

#if HAL_NMEA2K_ENABLED


static void send_pgn_127488(AP_J1939_CAN* driver)
{
    float rpm = 0.0f;
    // get the current engine RPM from the vehicle
    AP_ESC_Telem* esc_telem = AP_ESC_Telem::get_singleton();
    if (esc_telem == nullptr) {
        return;
    }
    if (esc_telem->get_rpm(0, rpm)) {

        nmea2k::N2KMessage msg;
        msg.SetPGN(127488);

        msg.AddByte(0);
        msg.Add2ByteUInt(rpm);
        msg.Add2ByteInt(0);
        msg.AddByte(0);

    }

}

void AP_NMEA2K::update(void)
{
    AP_CANManager* can_mgr = AP_CANManager::get_singleton();
    if (can_mgr == nullptr) {
        return;
    }

    for (uint8_t can_port=0; can_port < can_mgr->get_num_drivers(); can_port++) {
        if (can_mgr->get_driver_type(can_port) == AP_CAN::Protocol::J1939) {
            AP_J1939_CAN* driver = static_cast<AP_J1939_CAN*>(can_mgr->get_driver(can_port));
            if (driver == nullptr) {
                continue;
            }

            send_pgn_127488(driver);
        }
    }
}


#endif // HAL_NMEA2K_ENABLED