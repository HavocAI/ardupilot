#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_NMEA2K_ENABLED
#define HAL_NMEA2K_ENABLED (HAL_MAX_CAN_PROTOCOL_DRIVERS && BOARD_FLASH_SIZE > 1024)
#endif

#if HAL_NMEA2K_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_CANManager/AP_CANSensor.h>
class AP_NMEA2K :
    public CANSensor
{
public:
    AP_NMEA2K();
    CLASS_NO_COPY(AP_NMEA2K);

    static const struct AP_Param::GroupInfo var_info[];

    // called from the main loop
    static void update(void);

    void handle_frame(AP_HAL::CANFrame &frame) override;
};

#endif // HAL_NMEA2K_ENABLED