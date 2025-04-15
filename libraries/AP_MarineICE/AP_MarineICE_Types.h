#pragma once

#include <cstdint>

namespace MarineICE {
    namespace Types {
        // Gear Position corresponds to Dometic shift control byte
        enum class GearPosition {
            GEAR_FORWARD,
            GEAR_NEUTRAL,
            GEAR_REVERSE
        };

        enum class TrimCommand {
            TRIM_STOP,
            TRIM_UP,
            TRIM_DOWN
        };

        // Backend type parameter values
        enum class BackendType : uint8_t {
            BACKEND_TYPE_DISABLED = 0,
            BACKEND_TYPE_SIMULATED = 1,
            BACKEND_TYPE_NMEA2000 = 2
        };
    }
}
