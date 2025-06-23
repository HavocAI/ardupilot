#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_FORTVSC_ENABLED
#define HAL_FORTVSC_ENABLED BOARD_FLASH_SIZE > 1024
#endif
