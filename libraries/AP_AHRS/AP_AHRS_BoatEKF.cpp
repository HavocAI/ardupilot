#include <AP_AHRS/AP_AHRS_BoatEKF.h>

#if AP_AHRS_BOATEKF_ENABLED

#include <SRV_Channel/SRV_Channel.h>
#include <AP_Common/AP_Common.h>
#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS.h>

extern "C" {
    void boatekf_init(float dt);
    void boatekf_predict(float rudder, float throttle);
    void boatekf_update_gps(float north, float east, float var);
    void boatekf_get_position(float *north, float *east);
    void boatekf_get_velocity(float *north, float *east);
    void boatekf_get_wind(float *wind_north, float *wind_east);
}

NavBoatEKF::NavBoatEKF()
    : _last_time_predict_ms(0)
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

        float rudder = SRV_Channels::get_output_scaled(SRV_Channel::k_steering);
        float throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);

        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "BoatEKF: dt=%" PRIu32 " rudder=%.2f throttle=%.2f", dt_ms, rudder, throttle);

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
            accuracy = 4.0f;
        }

        boatekf_update_gps(ne.x, ne.y, accuracy);

        _last_time_update_gps_ms = now_ms;
    }
    
}

bool NavBoatEKF::set_origin(const Location &loc)
{
    _origin_location = loc;
    _have_origin = true;
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
    // TODO: implement boat EKF quaternion output
    return false;
}

bool NavBoatEKF::wind_estimate(Vector3f &wind) const
{
    float wind_north, wind_east;
    boatekf_get_wind(&wind_north, &wind_east);
    wind = Vector3f(wind_north, wind_east, 0.0f);
    return true;
}


#endif // AP_AHRS_BOATEKF_ENABLED