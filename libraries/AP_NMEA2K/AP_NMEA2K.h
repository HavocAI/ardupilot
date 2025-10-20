#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_NMEA2K_ENABLED
#define HAL_NMEA2K_ENABLED (HAL_MAX_CAN_PROTOCOL_DRIVERS && BOARD_FLASH_SIZE > 1024)
#endif

#if HAL_NMEA2K_ENABLED

class AP_NMEA2K {
    public:

    // called from the main loop
    void update(void);
};

#endif // HAL_NMEA2K_ENABLED