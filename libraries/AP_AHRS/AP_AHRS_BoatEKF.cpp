#include <AP_AHRS/AP_AHRS_BoatEKF.h>

#if AP_AHRS_BOATEKF_ENABLED

#include <SRV_Channel/SRV_Channel.h>
#include <AP_Common/AP_Common.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_AHRS/AP_AHRS.h>

#define BOATEKF_DEBUG 1

extern "C" {
    void boatekf_init(float dt);
    void boatekf_predict(float rudder, float throttle);
    void boatekf_update_gps(float north, float east, float var);
    void boatekf_update_compass(float yaw, float var);
    void boatekf_get_position(float *north, float *east);
    float boatekf_get_position_variance();
    float boatekf_get_heading();
    float boatekf_get_theta_variance();
    void boatekf_get_velocity(float *north, float *east);
    float boatekf_get_speed_variance();
    void boatekf_get_wind(float *wind_north, float *wind_east);
    void boatekf_update_origin();
}

NavBoatEKF::NavBoatEKF()
    : _last_time_predict_ms(0),
    _last_time_update_gps_ms(0),
    _last_time_update_compass(500)
{
    _have_origin = false;
}

void NavBoatEKF::init(void)
{
    boatekf_init(0.100f); // 100 ms timestep
}

void NavBoatEKF::update(void)
{
    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t dt_ms = now_ms - _last_time_predict_ms;
    if (dt_ms > 100) {
        _last_time_predict_ms = now_ms;

        float rudder = SRV_Channels::get_output_norm(SRV_Channel::k_steering);
        float throttle = SRV_Channels::get_output_norm(SRV_Channel::k_throttle);

        boatekf_predict(rudder, throttle);
    }

    if (now_ms - _last_time_update_gps_ms > 1000 && AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D) {

        const Location &location = AP::gps().location();
        if (!_have_origin) {
            set_origin(location);
        }
        Vector2f ne = _origin_location.get_distance_NE(location);

        float accuracy;
        if (!AP::gps().horizontal_accuracy(accuracy)) {
            accuracy = 1.5f;
        }
        accuracy = constrain_float(accuracy, 0.1f, 1000.0f);

        boatekf_update_gps(ne.x, ne.y, accuracy * accuracy);

        _last_time_update_gps_ms = now_ms;
    }

    if (now_ms - _last_time_update_compass > 1000) {
        const NavEKF3& EKF3 = AP::ahrs().EKF3;

        uint16_t faultInt;
        EKF3.getFilterFaults(faultInt);

        // we just want to ensure that the EKF3's quaternion is valid and the states have been initialized
        const uint16_t mask = (1 << 0) | (1 << 7);
        const bool is_healthy = (faultInt & mask) == 0;

        if (is_healthy) {
            // Matrix3f dcm_matrix;

            Vector3f eulers;
            // EKF3.getRotationBodyToNED(dcm_matrix);
            EKF3.getEulerAngles(eulers);
            // roll  = eulers.x;
            // pitch = eulers.y;
            float yaw   = eulers.z;
            float yaw_variance = 0.4f;

            // Vector3f velInnov;
            // Vector3f posInnov;
            // Vector3f magInnov;
            // float tasInnov = 0;
            // float yawInnov = 0;
            // EKF3.getInnovations(velInnov, posInnov, magInnov, tasInnov, yawInnov);
            // yaw_variance += yawInnov * yawInnov;

            // we use the yaw innovation squared as a variance proxy
            boatekf_update_compass(yaw, yaw_variance);

            _last_time_update_compass = now_ms;
        }
    }
    
}

bool NavBoatEKF::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    velVar = boatekf_get_speed_variance();
    posVar = boatekf_get_position_variance();
    hgtVar = 0.12f;
    const float theta_variance = boatekf_get_theta_variance();
    magVar = Vector3f(theta_variance, theta_variance, theta_variance);
    tasVar = 0.5f;
    return true;
}

bool NavBoatEKF::get_filter_status(nav_filter_status &status) const
{
    status.flags.attitude = true;
    status.flags.horiz_vel = true;
    status.flags.vert_vel = true;
    status.flags.horiz_pos_rel = true;
    status.flags.horiz_pos_abs = _have_origin;
    status.flags.vert_pos = true;
    status.flags.terrain_alt = false;
    status.flags.const_pos_mode = false;
    status.flags.pred_horiz_pos_rel = true;
    status.flags.pred_horiz_pos_abs = true;
    status.flags.takeoff_detected = false;
    status.flags.takeoff = false;
    status.flags.touchdown = false;
    status.flags.using_gps = true;
    status.flags.gps_glitching = false;
    status.flags.gps_quality_good = true;
    status.flags.initalized = true;
    status.flags.rejecting_airspeed = false;
    status.flags.dead_reckoning = false;
    return true;
}

void NavBoatEKF::send_status_report(GCS_MAVLINK &link) const
{
    nav_filter_status status;
    get_filter_status(status);
    uint16_t flags = 0;

    if (status.flags.attitude) {
        flags |= EKF_ATTITUDE;
    }
    if (status.flags.horiz_vel) {
        flags |= EKF_VELOCITY_HORIZ;
    }
    if (status.flags.vert_vel) {
        flags |= EKF_VELOCITY_VERT;
    }
    if (status.flags.horiz_pos_rel) {
        flags |= EKF_POS_HORIZ_REL;
    }
    if (status.flags.horiz_pos_abs) {
        flags |= EKF_POS_HORIZ_ABS;
    }
    if (status.flags.vert_pos) {
        flags |= EKF_POS_VERT_ABS;
    }
    if (status.flags.terrain_alt) {
        flags |= EKF_POS_VERT_AGL;
    }
    if (status.flags.const_pos_mode) {
        flags |= EKF_CONST_POS_MODE;
    }
    if (status.flags.pred_horiz_pos_rel) {
        flags |= EKF_PRED_POS_HORIZ_REL;
    }
    if (status.flags.pred_horiz_pos_abs) {
        flags |= EKF_PRED_POS_HORIZ_ABS;
    }
    if (!status.flags.initalized) {
        flags |= EKF_UNINITIALIZED;
    }


    float velVar = 0, posVar = 0, hgtVar = 0, tasVar = 0;
    Vector3f magVar;
    get_variances(velVar, posVar, hgtVar, magVar, tasVar);
    const float mag_max = fmaxF(fmaxF(magVar.x,magVar.y),magVar.z);
    float temp = 0.0f;

    mavlink_msg_ekf_status_report_send(link.get_chan(), flags, velVar, posVar, hgtVar, mag_max, temp, tasVar);
}

bool NavBoatEKF::set_origin(const Location &loc)
{
#if BOATEKF_DEBUG
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "BoatEKF origin set: lat=%" PRIi32 " lon=%" PRIi32 "", loc.lat, loc.lng);
#endif
    _origin_location = loc;
    _have_origin = true;
    boatekf_update_origin();
    return true;
}

bool NavBoatEKF::get_origin(Location &ret) const
{
    if (!_have_origin) {
        return false;
    }
    ret = _origin_location;
    return true;
}

bool NavBoatEKF::get_relative_position_NE(Vector2f &pos) const
{
    boatekf_get_position(&pos.x, &pos.y);
    return true;
}

bool NavBoatEKF::getLLH(Location &loc) const
{
    if (!_have_origin) {
        return false;
    }

    // get NE offset from origin
    float north, east;
    boatekf_get_position(&north, &east);

#if BOATEKF_DEBUG
    const uint32_t now_ms = AP_HAL::millis();
    static uint32_t last_print_ms = 0;
    if (now_ms - last_print_ms > 1000) {
        last_print_ms = now_ms;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "BoatEKF NE: N=%.4f E=%.4f", north, east);
    }
#endif

    // compute new location
    loc = _origin_location;
    loc.offset(north, east);

    return true;

}

bool NavBoatEKF::get_velocity(Vector2f &vel) const
{
    if (!_have_origin) {
        return false;
    }

    boatekf_get_velocity(&vel.x, &vel.y);
    return true;
}

bool NavBoatEKF::get_quaternion(Quaternion &quat) const
{
    const float heading_rad = boatekf_get_heading();
    quat.from_euler(0.0f, 0.0f, heading_rad);
    return true;
}

bool NavBoatEKF::wind_estimate(Vector3f &wind) const
{
    float wind_north, wind_east;
    boatekf_get_wind(&wind_north, &wind_east);
    wind = Vector3f(wind_north, wind_east, 0.0f);
    return true;
}


#endif // AP_AHRS_BOATEKF_ENABLED