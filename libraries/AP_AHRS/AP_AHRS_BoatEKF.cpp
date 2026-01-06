#include <AP_AHRS/AP_AHRS_BoatEKF.h>

#if AP_AHRS_BOATEKF_ENABLED

#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>

extern "C" {
    void boatekf_init(float dt);
    void boatekf_predict(float rudder, float throttle);
}

NavBoatEKF::NavBoatEKF()
    : _last_time_update_ms(0)
{
}

void NavBoatEKF::init(void)
{
    boatekf_init(0.100f); // 100 ms timestep
    
}

void NavBoatEKF::update(void)
{
    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t dt_ms = now_ms - _last_time_update_ms;
    if (dt_ms > 100) {
        _last_time_update_ms = now_ms;

        float rudder = SRV_Channels::get_output_scaled(SRV_Channel::k_steering);
        float throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);

        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "BoatEKF: dt=%" PRIu32 " rudder=%.2f throttle=%.2f", dt_ms, rudder, throttle);

        boatekf_predict(rudder, throttle);
    }
    
}


#endif // AP_AHRS_BOATEKF_ENABLED