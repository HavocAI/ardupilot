
#include "AP_IrisOrca.h"

#if AP_IRISORCA_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>

#define TIME_PASSED(start, delay_ms) (AP_HAL::millis() - (start) > delay_ms)

#define HIGHWORD(x) ((uint16_t)((x) >> 16) & 0xFFFF)
#define LOWWORD(x) ((uint16_t)(x) & 0xFFFF)

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_IrisOrca::var_info[] = {

    // @Param: DE_PIN
    // @DisplayName: Iris Orca DE pin
    // @Description: Pin number connected to RS485 to Serial converter's DE pin. -1 to use serial port's CTS pin if available
    // @Values: -1:Disabled,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("DPIN", 1, AP_IrisOrca, _pin_de, -1),

    // @Param: MAX_TRAVEL
    // @DisplayName: Shaft max physical travel distance
    // @Description: The max physical travel distance as measured from the zero position, which will be at one end of the actuator after zeroing.
    // @Units: mm
    // @Range: 0 300
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MTRVL", 2, AP_IrisOrca, _max_travel_mm, 205),

    // @Param: REVERSE_DIR
    // @DisplayName: Reverse direction
    // @Description: Reverse the direction of the actuator
    // @Values: 0:Normal,1:Reverse
    // @User: Standard
    AP_GROUPINFO("REVDR", 3, AP_IrisOrca, _reverse_direction, 0),

    // @Param: PAD_TRAVEL
    // @DisplayName: Pad travel distance
    // @Description: Amount to pad the physical travel distance by to ensure the actuator does not reach the physical end stops during normal motion.
    // @Units: mm
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("PTRVL", 4, AP_IrisOrca, _pad_travel_mm, 10),

    // @Param: F_MAX
    // @DisplayName: Maximum force
    // @Description: Maximum force for position control
    // @Units: mN
    // @Range: 0 1061000
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("FMAX", 5, AP_IrisOrca, _f_max, 60000),

    // @Param: GAIN_P
    // @DisplayName: Position control P gain
    // @Description: Proportional gain for position control
    // @Units: 64*N/mm
    // @Range: 0 65535
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("GP", 6, AP_IrisOrca, _gain_p, 40),

    // @Param: GAIN_I
    // @DisplayName: Position control I gain
    // @Description: Integral gain for position control
    // @Units: 64*N*s/mm
    // @Range: 0 65535
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("GI", 7, AP_IrisOrca, _gain_i, 50),

    // @Param: GAIN_DV
    // @DisplayName: Position control Dv gain
    // @Description: Derivative gain for position control
    // @Units: 2*N*s/m
    // @Range: 0 65535
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("GDV", 8, AP_IrisOrca, _gain_dv, 230),

    // @Param: GAIN_DE
    // @DisplayName: Position control De gain
    // @Description: Derivative error gain for position control
    // @Units: 2*N*s/m err
    // @Range: 0 65535
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("GDE", 9, AP_IrisOrca, _gain_de, 0),

    // @Param: AZ_F_MAX
    // @DisplayName: Auto-zero max force
    // @Description: Force threshold for auto-zero mode
    // @Units: N
    // @Range: 0 1061
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("AFMAX", 10, AP_IrisOrca, _auto_zero_f_max, 30),

    // @Param: SAFE_DGAIN
    // @DisplayName: Safety derivative gain
    // @Description: Safety derivative gain for position control
    // @Units: 2*N*s/m
    // @Range: 0 65535
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("SFEDG", 11, AP_IrisOrca, _safety_dgain, 0),

    // @Param: POS_FILT
    // @DisplayName: Position filter
    // @Description: Position filter for position control
    // @Units: 1/10000
    // @Range: 0 9999
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("POSFIL", 12, AP_IrisOrca, _pos_filt, 9950),

    AP_GROUPEND
};

AP_IrisOrca::AP_IrisOrca(void)
 : _initialized(false),
   _uart(nullptr),
  _modbus()
{
    // set defaults from the parameter table
    AP_Param::setup_object_defaults(this, var_info);
    async_init(&_run_state);
}

void AP_IrisOrca::init(void)
{
    const AP_SerialManager &serial_manager = AP::serialmanager();
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_IrisOrca, 0);
    if (_uart) {
        _modbus.init(_uart, _pin_de);
    }

    async_init(&_run_state);
}

void AP_IrisOrca::update()
{
    if (!_initialized) {
        _initialized = true;
        hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_IrisOrca::run_io, void));
    }

}

void AP_IrisOrca::run_io()
{
    if (_uart == nullptr) {
        init();
        return;
    }

    _modbus.tick();
    run();

}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
async AP_IrisOrca::run()
{
    async_begin(&_run_state)

    OrcaModbus::TransceiverState rx;
    uint16_t value;

    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IrisOrca: starting");

    _run_state.last_send_ms = AP_HAL::millis();
    await(TIME_PASSED(_run_state.last_send_ms, 5000));

    // // command sleep mode
    // _modbus.send_write_register_cmd(orca::Register::CTRL_REG_3, orca::OperatingMode::SLEEP);
    // await( (rx = _modbus.receive_state()) != OrcaModbus::ReceiveState::Pending );
    // if (rx != OrcaModbus::ReceiveState::Ready) {
    //     GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IrisOrca: Failed to set sleep mode");
    //     async_init(&_run_state);
    //     return ASYNC_CONT;
    // }

    // _modbus.send_write_register_cmd(orca::Register::CTRL_REG_4, 0x0007);

    _modbus.send_write_register_cmd(orca::Register::AUTO_ZERO_FORCE_N, _auto_zero_f_max);
    await( _modbus.tx_rx_finished() );
    if ( (rx = _modbus.transceiver_state()) != OrcaModbus::TransceiverState::Ready) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IrisOrca: Failed to read ZERO_MODE register");
        async_init(&_run_state);
        return ASYNC_CONT;
    }

    send_position_controller_params();
    await( _modbus.tx_rx_finished() );

    _modbus.send_write_register_cmd(orca::Register::POS_FILT, _pos_filt);
    await( _modbus.tx_rx_finished() );

    _modbus.send_write_register_cmd(orca::Register::SAFETY_DGAIN, _safety_dgain);
    await( _modbus.tx_rx_finished() );


    // undocumented write toe CTRL_REG_1 will make the PID value take effect immediately
    // _modbus.send_write_register_cmd(orca::Register::CTRL_REG_1, 1024);
    // await( (rx = _modbus.receive_state()) != OrcaModbus::ReceiveState::Pending );

    // set the persist bit
    // _modbus.send_write_register_cmd(orca::Register::CTRL_REG_2, 0x0001);
    // await( (rx = _modbus.receive_state()) != OrcaModbus::ReceiveState::Pending );
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Persist bit set");

    // // start auto-zero mode
    // _modbus.send_write_register_cmd(orca::Register::CTRL_REG_3, orca::OperatingMode::AUTO_ZERO);
    // await( (rx = _modbus.receive_state()) != OrcaModbus::ReceiveState::Pending );
    // if (rx != OrcaModbus::ReceiveState::Ready) {
    //     GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IrisOrca: Failed to set auto-zero mode");
    //     async_init(&_run_state);
    //     return ASYNC_CONT;
    // }
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: auto-zero mode set");


    
    // read ZERO_MODE register and if the "Auto Zero on Boot (3)" is not set, set it now
    _modbus.send_read_register_cmd(orca::Register::ZERO_MODE);
    await( _modbus.tx_rx_finished() );
    if ( !_modbus.read_register(value) ) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IrisOrca: Failed to read ZERO_MODE register");
        async_init(&_run_state);
        return ASYNC_CONT;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: ZERO_MODE: %u", value);

    if (value != 0x0003) {
        // set the auto zero on boot bit
        _modbus.send_write_register_cmd(orca::Register::ZERO_MODE, 0x0003);
        await( _modbus.tx_rx_finished() );
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Auto zero on boot set");

        // set auto zero max force
        _modbus.send_write_register_cmd(orca::Register::AUTO_ZERO_FORCE_N, _auto_zero_f_max);
        await( _modbus.tx_rx_finished() );

        // set the auto zero exit mode to 3 (position)
        _modbus.send_write_register_cmd(orca::Register::AUTO_ZERO_EXIT_MODE, 0x003);
        await( _modbus.tx_rx_finished() );

        // set the persist bit
        // _modbus.send_write_register_cmd(orca::Register::CTRL_REG_2, 0x0001);
        // await( (rx = _modbus.receive_state()) != OrcaModbus::ReceiveState::Pending );
        // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Persist bit set");
    }

    // start the auto-zero mode
    _modbus.send_write_register_cmd(orca::Register::CTRL_REG_3, orca::OperatingMode::AUTO_ZERO);
    await( _modbus.tx_rx_finished() );

    // set the user comms timeout to be 300ms
    _modbus.send_write_register_cmd(orca::Register::USER_COMMS_TIMEOUT, 300);
    await( _modbus.tx_rx_finished() );

    while (true) {

        _run_state.last_send_ms = AP_HAL::millis();

        // read the state of the motor.
        orca::write_motor_read_stream(orca::Register::CTRL_REG_3, 1, _modbus);
        await( _modbus.tx_rx_finished() );
        rx = _modbus.transceiver_state();
        if (rx != OrcaModbus::TransceiverState::Ready) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IrisOrca: Failed to read motor state");
            async_init(&_run_state);
            return ASYNC_CONT;
        }
        uint32_t reg_value;
        if (!orca::parse_motor_read_stream_rsp(_modbus._received_buff, _modbus._received_buff_len, _actuator_state, reg_value)) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IrisOrca: Failed to parse motor state");
            async_init(&_run_state);
            return ASYNC_CONT;
        }

        // if it is sleeping, then we need to send the auto-zero command.
        if (_actuator_state.mode == orca::OperatingMode::SLEEP) {
            _modbus.send_write_register_cmd(orca::Register::CTRL_REG_3, orca::OperatingMode::AUTO_ZERO);
            await( _modbus.tx_rx_finished() );

        } else if (_actuator_state.mode == orca::OperatingMode::POSITION) {
            // if it is in position control, then we exit the loop.
            break;
        } else if (_actuator_state.errors) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IrisOrca: auto-zero error: 0x%x", _actuator_state.errors);
            async_init(&_run_state);
            return ASYNC_CONT;
        }

        await(TIME_PASSED(_run_state.last_send_ms, 100));

    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: in position mode");

    // _modbus.set_recive_timeout_ms(75);

    while(true) {

        _run_state.last_send_ms = AP_HAL::millis();

        {
        uint32_t desired_shaft_pos = get_desired_shaft_pos();
        
        // // primitive slew rate limiting. We want to constrain the max velocity to 1mm/s
        // int32_t delta_um = desired_shaft_pos - _actuator_state.shaft_position;
        // int32_t delta = constrain_int32(delta_um, -1000, 1000);
        // uint32_t command_shaft_pos = constrain_uint32(_actuator_state.shaft_position + delta, 0, _max_travel_mm * 1000);

        uint32_t command_shaft_pos = constrain_uint32(desired_shaft_pos, 0, _max_travel_mm * 1000);

        orca::write_motor_command_stream(orca::MotorCommandStreamSubCode::POSITION_CONTROL_STREAM, command_shaft_pos, _modbus);
        }
        await( _modbus.tx_rx_finished() );
        rx = _modbus.transceiver_state();
        
        switch (rx) {
            case OrcaModbus::TransceiverState::Ready:
                orca::parse_motor_command_stream_rsp(_modbus._received_buff, _modbus._received_buff_len, _actuator_state);
                // TODO: if getting close to over temp then ramp down the max force register (keep average power ~30w)

                if (_actuator_state.errors != 0) {
                    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IrisOrca: actuator error: %u", _actuator_state.errors);
                    async_init(&_run_state);
                    return ASYNC_CONT;
                }
                break;
            case OrcaModbus::TransceiverState::Timeout:
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IrisOrca: actuator position msg timeout");
                async_init(&_run_state);
                return ASYNC_CONT;
                break;
            case OrcaModbus::TransceiverState::CRCError:
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IrisOrca: actuator position msg crcerror");
                // async_init(&_run_state);
                // return ASYNC_CONT;
                break;
            default:
                break;
        }

        // send at 10Hz
        await(TIME_PASSED(_run_state.last_send_ms, 100));
    }

    async_end
}
#pragma GCC diagnostic pop

void AP_IrisOrca::send_position_controller_params()
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: PID params: %d %d %d %d", _gain_p.get(), _gain_i.get(), _gain_dv.get(), _gain_de.get());
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: FMAX: %ld", _f_max.get());

    uint16_t registers[6];
    registers[0] = _gain_p.get();
    registers[1] = _gain_i.get();
    registers[2] = _gain_dv.get();
    registers[3] = _gain_de.get();
    registers[4] = HIGHWORD(_f_max.get());
    registers[5] = LOWWORD(_f_max.get());

    _modbus.send_write_multiple_registers(orca::Register::PC_PGAIN, 6, registers);
}

uint32_t AP_IrisOrca::get_desired_shaft_pos()
{
    float yaw = SRV_Channels::get_output_norm(SRV_Channel::Aux_servo_function_t::k_steering);

    const float m = (_reverse_direction ? -1.0 : 1.0) * 0.5 * 1000 * (_max_travel_mm - (2.0 * _pad_travel_mm));
    const float b = 0.5 * 1000 * _max_travel_mm;
    
    const float shaft_position_um = yaw * m + b;

    return shaft_position_um;
}

void orca::write_motor_command_stream(const MotorCommandStreamSubCode sub_code, const uint32_t data, OrcaModbus& modbus)
{
    uint16_t i = 0;
    uint8_t send_buff[16];
    send_buff[i++] = MsgAddress::DEVICE;
    send_buff[i++] = FunctionCode::MOTOR_COMMAND_STREAM;
    send_buff[i++] = sub_code;

    // data is 32 bits - send as 4 bytes
    put_be32_ptr(&send_buff[i], data);
    i += 4;

    modbus.send_data(send_buff, i, MOTOR_COMMAND_STREAM_MSG_RSP_LEN);
}

bool orca::parse_motor_command_stream_rsp(const uint8_t* data, const uint16_t len, ActuatorState& state)
{
    // if (len < MOTOR_COMMAND_STREAM_MSG_RSP_LEN) {
    //     return false;
    // }

    if (data[0] != MsgAddress::DEVICE) {
        return false;
    }
    if (data[1] != FunctionCode::MOTOR_COMMAND_STREAM) {
        return false;
    }

    state.shaft_position = be32toh_ptr(&data[2]);
    state.force_realized = be32toh_ptr(&data[6]);
    state.power_consumed = be16toh_ptr(&data[10]);
    state.temperature = data[12];
    state.voltage = be16toh_ptr(&data[13]);
    state.errors = be16toh_ptr(&data[15]);

    return true;
}

void orca::write_motor_read_stream(const uint16_t reg_addr, const uint8_t reg_width, OrcaModbus& modbus)
{
    // assert reg_width is 1 or 2
    // assert(reg_width == 1 || reg_width == 2);

    uint16_t i = 0;
    uint8_t send_buff[16];
    send_buff[i++] = MsgAddress::DEVICE;
    send_buff[i++] = FunctionCode::MOTOR_READ_STREAM;
    send_buff[i++] = HIGHBYTE(reg_addr);
    send_buff[i++] = LOWBYTE(reg_addr);
    send_buff[i++] = reg_width;

    modbus.send_data(send_buff, i, MOTOR_READ_STREAM_MSG_RSP_LEN);
}

bool orca::parse_motor_read_stream_rsp(const uint8_t* data, const uint16_t len, ActuatorState& state, uint32_t& reg_value)
{
    // if (len < MOTOR_READ_STREAM_MSG_RSP_LEN) {
    //     return false;
    // }

    if (data[0] != MsgAddress::DEVICE) {
        return false;
    }
    if (data[1] != FunctionCode::MOTOR_READ_STREAM) {
        return false;
    }

    reg_value = be32toh_ptr(&data[2]);

    state.mode = (OperatingMode)data[6];
    state.shaft_position = be32toh_ptr(&data[7]);
    state.force_realized = be32toh_ptr(&data[11]);
    state.power_consumed = be16toh_ptr(&data[15]);
    state.temperature = data[17];
    state.voltage = be16toh_ptr(&data[18]);
    state.errors = be16toh_ptr(&data[20]);

    return true;
}

#endif // AP_IRISORCA_ENABLED






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

