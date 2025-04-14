
#include "AP_IrisOrca.h"

#if AP_IRISORCA_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_IrisOrca::var_info[] = {

    // @Param: DE_PIN
    // @DisplayName: Iris Orca DE pin
    // @Description: Pin number connected to RS485 to Serial converter's DE pin. -1 to use serial port's CTS pin if available
    // @Values: -1:Disabled,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("DE_PIN", 1, AP_IrisOrca, _pin_de, -1),

    // @Param: MAX_TRAVEL
    // @DisplayName: Shaft max physical travel distance
    // @Description: The max physical travel distance as measured from the zero position, which will be at one end of the actuator after zeroing.
    // @Units: mm
    // @Range: 0 300
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MAX_TRAVEL", 2, AP_IrisOrca, _max_travel_mm, 205),

    // @Param: REVERSE_DIR
    // @DisplayName: Reverse direction
    // @Description: Reverse the direction of the actuator
    // @Values: 0:Normal,1:Reverse
    // @User: Standard
    AP_GROUPINFO("REVERSE_DIR", 3, AP_IrisOrca, _reverse_direction, 0),

    // @Param: PAD_TRAVEL
    // @DisplayName: Pad travel distance
    // @Description: Amount to pad the physical travel distance by to ensure the actuator does not reach the physical end stops during normal motion.
    // @Units: mm
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("PAD_TRAVEL", 4, AP_IrisOrca, _pad_travel_mm, 10),

    // @Param: F_MAX
    // @DisplayName: Maximum force
    // @Description: Maximum force for position control
    // @Units: mN
    // @Range: 0 1061000
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("F_MAX", 5, AP_IrisOrca, _f_max, 638000),

    // @Param: GAIN_P
    // @DisplayName: Position control P gain
    // @Description: Proportional gain for position control
    // @Units: 64*N/mm
    // @Range: 0 65535
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("GAIN_P", 6, AP_IrisOrca, _gain_p, 200),

    // @Param: GAIN_I
    // @DisplayName: Position control I gain
    // @Description: Integral gain for position control
    // @Units: 64*N*s/mm
    // @Range: 0 65535
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("GAIN_I", 7, AP_IrisOrca, _gain_i, 1000),

    // @Param: GAIN_DV
    // @DisplayName: Position control Dv gain
    // @Description: Derivative gain for position control
    // @Units: 2*N*s/m
    // @Range: 0 65535
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("GAIN_DV", 8, AP_IrisOrca, _gain_dv, 800),

    // @Param: GAIN_DE
    // @DisplayName: Position control De gain
    // @Description: Derivative error gain for position control
    // @Units: 2*N*s/m err
    // @Range: 0 65535
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("GAIN_DE", 9, AP_IrisOrca, _gain_de, 0),

    // @Param: AZ_F_MAX
    // @DisplayName: Auto-zero max force
    // @Description: Force threshold for auto-zero mode
    // @Units: N
    // @Range: 0 1061
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("AZ_F_MAX", 10, AP_IrisOrca, _auto_zero_f_max, 300),

    AP_GROUPEND
};

AP_IrisOrca::AP_IrisOrca(void)
{
    // set defaults from the parmeter table
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_IrisOrca::update()
{
    if (_uart == nullptr) {
        return;
    }
}

#endif // AP_IRISORCA_ENABLED

// #define TIME_PASSED(start, delay_ms) (AP_HAL::millis() - (start) > delay_ms)

// static void send_actuator_position_cmd(uint32_t position_um, OrcaModbus* modbus)
// {
//     using namespace orca;

//     uint16_t i = 0;
//     uint8_t send_buff[MOTOR_COMMAND_STREAM_MSG_LEN];

//     send_buff[i++] = static_cast<uint8_t>(MsgAddress::DEVICE);
//     send_buff[i++] = static_cast<uint8_t>(FunctionCode::MOTOR_COMMAND_STREAM);
//     send_buff[i++] = static_cast<uint8_t>(MotorCommandStreamSubCode::POSITION_CONTROL_STREAM);

//     // data is 32 bits - send as 4 bytes
//     send_buff[i++] = HIGHBYTE(HIGHWORD(position_um));
//     send_buff[i++] = LOWBYTE(HIGHWORD(position_um));
//     send_buff[i++] = HIGHBYTE(LOWWORD(position_um));
//     send_buff[i++] = LOWBYTE(LOWWORD(position_um));

//     modbus->send_data(send_buff, i, MOTOR_COMMAND_STREAM_MSG_RSP_LEN);
// }


// typedef struct example_state {
//     async_state;
//     AP_IrisOrca *self;
//     uint32_t last_send_ms;
// } example_state_t;

// {
    // static async run(example_state_t *pt)
//     uint32_t now_ms;
//     uint8_t err;

//     async_begin(pt);

//     // read ZERO_MODE register and if the "Auto Zero on Boot (3)" is not set, set it now
//     pt->self->_modbus.send_read_register_cmd((uint16_t) orca::Register::ZERO_MODE);
//     await( (err = pt->self->_modbus.message_received()) != MODBUS_MSG_RECV_PENDING );
//     if (err == MODBUS_MSG_RECV_TIMEOUT) {
//         GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IrisOrca: Failed to read ZERO_MODE register");
//         async_init(pt);
//         return ASYNC_CONT;
//     }
//     uint16_t zero_mode = pt->self->_modbus.read_register();

//     if (zero_mode != 0x0003)
//     {
//         // set the auto zero on boot bit
//         pt->self->_modbus.send_write_register_cmd((uint16_t) orca::Register::ZERO_MODE, 0x0003);
//         await( (err = pt->self->_modbus.message_received()) != MODBUS_MSG_RECV_PENDING );
//         GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Auto zero on boot set");

//         // set the auto zero exit mode to 3 (position)
//         pt->self->_modbus.send_write_register_cmd((uint16_t) orca::Register::AUTO_ZERO_EXIT_MODE, 0x003);
//         await( (err = pt->self->_modbus.message_received()) != MODBUS_MSG_RECV_PENDING );

//         // set the persist bit
//         pt->self->_modbus.send_write_register_cmd((uint16_t) orca::Register::CTRL_REG_2, 0x0001);
//         await( (err = pt->self->_modbus.message_received()) != MODBUS_MSG_RECV_PENDING );
//         GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Persist bit set");
        
//     }

//     pt->self->send_position_controller_params();
//     await( (err = pt->self->_modbus.message_received()) != MODBUS_MSG_RECV_PENDING );

    

//     pt->self->_modbus.set_recive_timeout_ms(75);

//     while(true) {

//         pt->self->send_actuator_position_cmd();
//         pt->last_send_ms = AP_HAL::millis();

//         await( (err = pt->self->_modbus.message_received()) != MODBUS_MSG_RECV_PENDING );
//         if (err == MODBUS_MSG_RECV_TIMEOUT) {
//             GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IrisOrca: actuator position return msg timeout");
//         }

//         // TODO: check for errors in the response


//         // send at 10Hz
//         await(TIME_PASSED(pt->last_send_ms, 100));
//     }


//     async_end;
// }

// typedef struct modbus_receive_task {
//     async_state;
//     AP_IrisOrca *self;
// } modbus_receive_task_t;

// static async modbus_receive_fn(modbus_receive_task_t *pt)
// {
//     int16_t b;
//     async_begin(pt);

//     while(true) {
//         pt->self->_modbus.read();
//         async_yield;
//     }

//     async_end;
// }

// void AP_IrisOrca::send_position_controller_params()
// {
//     uint16_t registers[6];
//     registers[0] = _gain_p;
//     registers[1] = _gain_i;
//     registers[2] = _gain_dv;
//     registers[3] = _gain_de;
//     registers[4] = HIGHWORD(_f_max);
//     registers[5] = LOWWORD(_f_max);

//     _modbus.send_write_multiple_registers((uint16_t)orca::Register::PC_PGAIN, 6, registers);
// }