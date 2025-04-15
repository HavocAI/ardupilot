#pragma once

#include <cstdint>

namespace MarineICE {
    namespace States {
        enum class EngineState : uint8_t {
            ENGINE_INIT,
            ENGINE_RUN_NEUTRAL,
            ENGINE_RUN_FORWARD,
            ENGINE_RUN_REVERSE,
            ENGINE_START,
            ENGINE_START_WAIT,
            ENGINE_FAULT
        };

        enum class TrimState {
            TRIM_MANUAL,
            TRIM_AUTO_STOP,
            TRIM_AUTO_UP,
            TRIM_AUTO_DOWN
        };
    }
}
