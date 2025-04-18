#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_IRISORCA_ENABLED
#define AP_IRISORCA_ENABLED BOARD_FLASH_SIZE > 1024
#endif

#if AP_IRISORCA_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Common/async.h>
#include "modbus.h"

class AP_IrisOrca
{
public:
    AP_IrisOrca();

    // Do not allow copies
    CLASS_NO_COPY(AP_IrisOrca);

    static const struct AP_Param::GroupInfo var_info[];

    void update();

private:
    

    // parameters
    AP_Int8 _pin_de;                    // Pin number connected to RS485 to Serial converter's DE pin. -1 to disable sending commands to actuator
    AP_Int16 _max_travel_mm;            // maximum travel of actuator in millimeters
    AP_Int16 _pad_travel_mm;            // padding travel of actuator in millimeters
    AP_Int8 _reverse_direction;         // reverse direction of actuator
    AP_Int32 _f_max;                    // position control max force in milliNewtons
    AP_Int16 _gain_p;                   // position control proportional gain
    AP_Int16 _gain_i;                   // position control integral gain
    AP_Int16 _gain_dv;                  // position control derivative gain
    AP_Int16 _gain_de;                  // position control derivative error gain
    AP_Int16 _auto_zero_f_max;          // maximum force for auto zero in Newtons
    AP_Int16 _safety_dgain;             // safety derivative gain
    AP_Int16 _pos_filt;                 // position filter

    // members
    bool _initialized;
    AP_HAL::UARTDriver *_uart;

    typedef struct run_state {
        async_state;
        uint32_t last_send_ms;
    } run_state_t;

    run_state_t _run_state;

    OrcaModbus _modbus;

    uint32_t last_send_ms;
    
    // 
    void init(void);
    async run();
    void run_io();
    void send_position_controller_params();
    void send_actuator_position_cmd();

};

#endif // AP_IRISORCA_ENABLED