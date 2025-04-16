#pragma once

#include <AP_Param/AP_Param.h>

class AP_MarineICE_Params
{
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_MarineICE_Params(void);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_MarineICE_Params);

    // parameters
    AP_Int8 type;          // backend type used (0:disabled, 1:simulated, 2:NMEA2000-Dometic-Suzuki)
    AP_Int8 auto_start;    // whether to automatically start the engine
    AP_Int8 auto_trim;     // whether to automatically trim the engine
    AP_Float safe_trim;    // maximum trim level to allow the engine to run (deg)
    AP_Int8 start_time;    // max time (sec) that the starter will run during each try
    AP_Int8 start_delay;   // delay (sec) between starting the engine
    AP_Int8 start_retries; // number of retries to start the engine
    AP_Int16 rpm_thres;    // threshold for RPM to consider the engine started
    AP_Int8 thr_slewrate;  // throttle slew rate (%/sec)
    AP_Int8 thr_max;       // maximum throttle level (%)
    AP_Int8 thr_deadband;  // zero deadband for throttle input (%)
    AP_Int16 rpm_max;      // maximum normal RPM (RPM)
    AP_Float temp_max;     // maximum normal temperature (deg C)
    AP_Int8 rng_fndr;      // water depth range finder instance number (-1 if not used, zero indexed)
    AP_Float water_depth_thres; // minimum water depth (m) to start the engine
    AP_Int8 can_port;      // CAN port to use for the backend (0:CAN1, 1:CAN2, -1:disabled)
};
