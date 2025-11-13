#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_NMEA2K_ENABLED

#include "AP_ExternalAHRS_backend.h"
#include <AP_NMEA2K/AP_NMEA2K.h>

class AP_ExternalAHRS_NMEA2K : public AP_ExternalAHRS_backend {
public:

    AP_ExternalAHRS_NMEA2K(AP_ExternalAHRS*, AP_ExternalAHRS::state_t&);
    CLASS_NO_COPY(AP_ExternalAHRS_NMEA2K);

    const char* get_name() const override {
        return "NMEA2K";
    }

    bool healthy(void) const override;

    bool initialised(void) const override;

    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;

    void update() override;

    uint8_t num_gps_sensors(void) const override;

    void get_filter_status(nav_filter_status &status) const override;

private:
    void handle_nmea2k_message(AP_NMEA2K* nmea2k_instance, nmea2k::N2KMessage& msg);

    AP_ExternalAHRS::gps_data_message_t gps_data;

    struct {
        uint8_t pgn_129029_method;
    } cached_data;

    uint32_t last_att_ms;
    uint32_t last_vel_ms;
    uint32_t last_pos_ms;
    uint32_t last_gps_ms;
};

#endif // AP_EXTERNAL_AHRS_NMEA2K_ENABLED