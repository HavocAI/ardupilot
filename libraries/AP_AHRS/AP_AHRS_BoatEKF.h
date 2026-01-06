#pragma once


#include <AP_AHRS/AP_AHRS_config.h>

#if AP_AHRS_BOATEKF_ENABLED

#include <AP_NavEKF/AP_Nav_Common.h>

class NavBoatEKF {
public:
    NavBoatEKF();

    /* Do not allow copies */
    CLASS_NO_COPY(NavBoatEKF);

    void init(void);

    // Update Filter - this should be called from AP_AHRS::update in the main loop (400Hz?)
    void update(void);

private:
    uint32_t _last_time_update_ms;
    
};

#endif // AP_AHRS_BOATEKF_ENABLED


