#pragma once

#include "AP_Anello_Config.h"

#if HAL_ANELLO_ENABLED

#include <AP_Param/AP_Param.h>

class AP_Anello {
public:
    AP_Anello();
    CLASS_NO_COPY(AP_Anello);

    static const struct AP_Param::GroupInfo var_info[];

    void update(void);

private:

    AP_Int8 _gps_param;

};

#endif // HAL_ANELLO_ENABLED