// Enum class for Engine States

#pragma once

#include <AP_Common/AP_Common.h>

enum class EngineState : uint8_t {
    INIT,
    RUN_NEUTRAL,
    RUN_FORWARD,
    RUN_REVERSE,
    START,
    START_WAIT,
    FAULT
};
