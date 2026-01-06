#pragma once


#include <AP_AHRS/AP_AHRS_config.h>

#if AP_AHRS_BOATEKF_ENABLED

#include <AP_NavEKF/AP_Nav_Common.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>

class NavBoatEKF {
public:
    NavBoatEKF();

    /* Do not allow copies */
    CLASS_NO_COPY(NavBoatEKF);

    void init(void);

    // Update Filter - this should be called from AP_AHRS::update in the main loop (400Hz?)
    void update(void);

    bool set_origin(const Location &loc);

    bool getLLH(Location &loc) const;
    bool get_velocity(Vector2f &vel) const;
    bool get_quaternion(Quaternion &quat) const;
    bool wind_estimate(Vector3f &wind) const;

private:
    uint32_t _last_time_predict_ms;
    uint32_t _last_time_update_gps_ms;
    bool _have_origin;
    Location _origin_location;
    
};

#endif // AP_AHRS_BOATEKF_ENABLED


