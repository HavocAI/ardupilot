#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_NMEA2K_ENABLED

#include "AP_ExternalAHRS_backend.h"

class AP_ExternalAHRS_NMEA2K : public AP_ExternalAHRS_backend {
    public:

    AP_ExternalAHRS_NMEA2K(AP_ExternalAHRS*, AP_ExternalAHRS::state_t&);

    const char* get_name() const override {
        return "NMEA2K";
    }

    bool healthy(void) const override;

    bool initialised(void) const override;

    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;

    void update() override;

    uint8_t num_gps_sensors(void) const override;
};

#endif // AP_EXTERNAL_AHRS_NMEA2K_ENABLED