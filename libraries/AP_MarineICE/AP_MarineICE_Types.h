#pragma once

#include <cstdint>

namespace MarineICE {
    namespace Types {
        // Gear Position corresponds to Dometic and Suzuki standard,
        // which appears to be the NMEA2000 gear status enumeration standard
        enum class GearPosition {
            GEAR_FORWARD = 0,
            GEAR_NEUTRAL = 1,
            GEAR_REVERSE = 2
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

        // Faults
        static constexpr size_t NUM_FAULTS = 7;
        enum FaultIndex {
            ENGINE_OVERSPEED,
            ENGINE_OVERTEMP,
            THROTTLE_ACTUATOR_FAILURE,
            GEAR_ACTUATOR_FAILURE,
            ALTERNATOR_VOLTAGE_LOW,
            ENGINE_START_ATTEMPTS_EXCEEDED,
            WATER_DEPTH_TOO_LOW
        };

        inline const char* fault_to_string(FaultIndex fault) {
            switch (fault) {
                case ENGINE_OVERSPEED: return "ENGINE_OVERSPEED";
                case ENGINE_OVERTEMP: return "ENGINE_OVERTEMP";
                case THROTTLE_ACTUATOR_FAILURE: return "THROTTLE_ACTUATOR_FAILURE";
                case GEAR_ACTUATOR_FAILURE: return "GEAR_ACTUATOR_FAILURE";
                case ALTERNATOR_VOLTAGE_LOW: return "ALTERNATOR_VOLTAGE_LOW";
                case ENGINE_START_ATTEMPTS_EXCEEDED: return "ENGINE_START_ATTEMPTS_EXCEEDED";
                default: return "UNKNOWN_FAULT";
            }
        }
    }
}
