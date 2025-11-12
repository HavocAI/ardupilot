#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_NMEA2K_ENABLED

#include "AP_ExternalAHRS_NMEA2K.h"
#include <GCS_MAVLink/GCS.h>

AP_ExternalAHRS_NMEA2K::AP_ExternalAHRS_NMEA2K(AP_ExternalAHRS *_frontend, AP_ExternalAHRS::state_t &_state)
 : AP_ExternalAHRS_backend(_frontend, _state)
{}

#endif // AP_EXTERNAL_AHRS_NMEA2K_ENABLED