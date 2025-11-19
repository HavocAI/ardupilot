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

    int8_t get_port(void) const override { return 0; };

    bool healthy(void) const override;

    bool initialised(void) const override;

    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;

    void update() override;

    void get_filter_status(nav_filter_status &status) const override;

    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const override;

    uint8_t num_gps_sensors(void) const override {
        return 1;
    }

private:
    bool init();
    void handle_nmea2k_message(AP_NMEA2K* nmea2k_instance, nmea2k::N2KMessage& msg);

    AP_ExternalAHRS::gps_data_message_t gps_data;

    struct {
        uint8_t pgn_129029_method;
    } cached_data;

    uint32_t last_att_ms;
    uint32_t last_vel_ms;
    uint32_t last_pos_ms;
    uint32_t last_gps_ms;
    bool initialized;
    uint32_t last_send_cmd;
    AP_NMEA2K* nmea2k;

    Location _last_velocity_location;

};

#endif // AP_EXTERNAL_AHRS_NMEA2K_ENABLED