#pragma once


#include <AP_AHRS/AP_AHRS_config.h>

#if AP_AHRS_BOATEKF_ENABLED

#include <AP_NavEKF/AP_Nav_Common.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

class NavBoatEKF {
public:
    NavBoatEKF();

    /* Do not allow copies */
    CLASS_NO_COPY(NavBoatEKF);

    void init(void);

    bool healthy(void) const {
        return true;
    }

    bool get_filter_status(nav_filter_status &status) const;
    void send_status_report(GCS_MAVLINK &link) const;

    // Update Filter - this should be called from AP_AHRS::update in the main loop (400Hz?)
    void update(bool disable_gps, bool disable_compass);

    bool set_origin(const Location &loc);
    bool get_origin(Location &ret) const;

    bool getLLH(Location &loc) const;
    bool get_relative_position_NE(Vector2f &pos) const;
    bool get_velocity(Vector2f &vel) const;
    bool get_quaternion(Quaternion &quat) const;
    bool wind_estimate(Vector3f &wind) const;

    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const;

private:
    uint32_t _last_time_predict_ms;
    uint32_t _last_time_update_gps_ms;
    uint32_t _last_time_update_compass;
    bool _have_origin;
    Location _origin_location;
    
};

#endif // AP_AHRS_BOATEKF_ENABLED


