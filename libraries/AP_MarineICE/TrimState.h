// Enum class for Trim States

#pragma once

#include <AP_Common/AP_Common.h>

enum class TrimState : uint8_t {
    MANUAL,
    AUTO_STOP,
    AUTO_UP,
    AUTO_DOWN
};
