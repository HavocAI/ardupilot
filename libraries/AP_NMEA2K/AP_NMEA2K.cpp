#include <AP_NMEA2K/AP_NMEA2K.h>

#include <AP_CANManager/AP_CANManager.h>
#include <AP_J1939_CAN/AP_J1939_CAN.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>

#include <AP_NMEA2K/AP_NMEA2K_msg.h>

#include "can-msg-definitions/n2k.h"

#if HAL_NMEA2K_ENABLED

const AP_Param::GroupInfo AP_NMEA2K::var_info[] = {

    AP_GROUPEND
};


static void send_pgn_127488(AP_NMEA2K* driver)
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

        const n2k_pgn_127488_engine_parameters_rapid_update_t data = {
            .instance = 0,
            .speed = static_cast<uint16_t>(rpm),
        };

        n2k_pgn_127488_engine_parameters_rapid_update_pack(
            msg.DataPtrForPack(),
            &data,
            nmea2k::N2KMessage::MAX_DATA_SIZE
        );

        msg.AddByte(0);
        msg.Add2ByteUInt(rpm);
        msg.Add2ByteInt(0);
        msg.AddByte(0);

        AP_HAL::CANFrame frame;
        frame.id = msg.FormatToCanId() | AP_HAL::CANFrame::FlagEFF;
        frame.dlc = msg.data_length();
        msg.CopyDataToBuffer(frame.data, sizeof(frame.data), msg.data_length());
        driver->write_frame(frame, 10);

    }

}

AP_NMEA2K::AP_NMEA2K() :
CANSensor("NMEA2K", 2048)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_NMEA2K::handle_frame(AP_HAL::CANFrame &frame)
{

}

void AP_NMEA2K::update(void)
{
    AP_CANManager* can_mgr = AP_CANManager::get_singleton();
    if (can_mgr == nullptr) {
        return;
    }

    for (uint8_t can_port=0; can_port < can_mgr->get_num_drivers(); can_port++) {
        if (can_mgr->get_driver_type(can_port) == AP_CAN::Protocol::NMEA2K) {
            AP_NMEA2K* driver = static_cast<AP_NMEA2K*>(can_mgr->get_driver(can_port));
            if (driver == nullptr) {
                continue;
            }

            send_pgn_127488(driver);
        }
    }
}


#endif // HAL_NMEA2K_ENABLED