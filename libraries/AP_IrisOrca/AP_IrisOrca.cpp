/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_IrisOrca.h"

#if HAL_IRISORCA_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/utility/sparse-endian.h>

#define IRISORCA_SERIAL_BAUD                    19200   // communication is always at 19200
#define IRISORCA_SERIAL_PARITY                  2       // communication is always even parity
#define IRISORCA_LOG_ORCA_INTERVAL_MS           5000    // log ORCA message at this interval in milliseconds
#define IRISORCA_SEND_ACTUATOR_CMD_INTERVAL_MS  100     // actuator commands sent at 10hz if connected to actuator
#define IRISORCA_REPLY_TIMEOUT_MS               25      // stop waiting for replies after 25ms
#define IRISORCA_ERROR_REPORT_INTERVAL_MAX_MS   10000   // errors reported to user at no less than once every 10 seconds

#define HIGHWORD(x) ((uint16_t)((x) >> 16))
#define LOWWORD(x) ((uint16_t)(x))
#define TIME_PASSED(start, delay_ms) (AP_HAL::millis() - (start) > delay_ms)

namespace orca {

bool parse_write_register(uint8_t *rcvd_buff, uint8_t buff_len, ActuatorState &state) {
  if (buff_len < WRITE_REG_MSG_RSP_LEN) {
    return false;
  }

  // Switch on the register address (bytes 2 and 3)
  switch ((rcvd_buff[WriteRegRsp::Idx::REG_ADDR_HI] << 8) |
          rcvd_buff[WriteRegRsp::Idx::REG_ADDR_LO]) {
    case static_cast<uint16_t>(Register::CTRL_REG_3):
      // Mode of operation was set
      break;
    case static_cast<uint16_t>(Register::SAFETY_DGAIN):
        state.safety_dgain_set = true;
        break;
    case static_cast<uint16_t>(Register::POS_FILT):
        state.pos_filter_set = true;
        break;
    default:
      GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                    "IrisOrca: Unsupported write register.");
      return false;
  }

  return true;
}

bool parse_multiple_write_registers(uint8_t *rcvd_buff, uint8_t buff_len,
                                    ActuatorState &state) {
  if (buff_len < MULTIPLE_WRITE_REG_MSG_RSP_LEN) {
    return false;
  }

  // Switch on the register address (bytes 2 and 3)
  switch ((rcvd_buff[MultipleWriteRegRsp::Idx::REG_ADDR_HI] << 8) |
          rcvd_buff[MultipleWriteRegRsp::Idx::REG_ADDR_LO]) {
    case static_cast<uint16_t>(Register::PC_PGAIN):
      // Position params (starting with P gain) were set
      GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Position params set");
      state.pc_params_set = true;
      break;
    case static_cast<uint16_t>(Register::ZERO_MODE):
      // Zero Mode params were set
      GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Zero mode params set");
      state.auto_zero_params_set = true;
      break;
    default:
      GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                    "IrisOrca: Unsupported multiple write registers");
      return false;
  }

  return true;
}

bool parse_motor_command_stream(uint8_t *rcvd_buff, uint8_t buff_len,
                                ActuatorState &state) {
  if (buff_len < MOTOR_COMMAND_STREAM_MSG_RSP_LEN) {
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                  "IrisOrca: Motor Command Stream response too short.");
    return false;
  }

  state.shaft_position =
      u32_from_be(rcvd_buff, MotorCommandStreamRsp::Idx::POSITION_MSB_HI);
  state.force_realized =
      u32_from_be(rcvd_buff, MotorCommandStreamRsp::Idx::FORCE_MSB_HI);
  state.power_consumed =
      u16_from_be(rcvd_buff, MotorCommandStreamRsp::Idx::POWER_HI);
  state.temperature = rcvd_buff[MotorCommandStreamRsp::Idx::TEMP];
  state.voltage =
      u16_from_be(rcvd_buff, MotorCommandStreamRsp::Idx::VOLTAGE_HI);
  state.errors = u16_from_be(rcvd_buff, MotorCommandStreamRsp::Idx::ERROR_HI);

  return true;
}

bool parse_motor_read_stream(uint8_t *rcvd_buff, uint8_t buff_len,
                             ActuatorState &state) {
  if (buff_len < MOTOR_READ_STREAM_MSG_RSP_LEN) {
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                  "IrisOrca: Motor Read Stream response too short.");
    return false;
  }
  // Ignore the read register value and set the other state members
  state.mode =
      static_cast<OperatingMode>(rcvd_buff[MotorReadStreamRsp::Idx::MODE]);
  state.shaft_position =
      u32_from_be(rcvd_buff, MotorReadStreamRsp::Idx::POSITION_MSB_HI);
  state.force_realized =
      u32_from_be(rcvd_buff, MotorReadStreamRsp::Idx::FORCE_MSB_HI);
  state.power_consumed =
      u16_from_be(rcvd_buff, MotorReadStreamRsp::Idx::POWER_HI);
  state.temperature = rcvd_buff[MotorReadStreamRsp::Idx::TEMP];
  state.voltage = u16_from_be(rcvd_buff, MotorReadStreamRsp::Idx::VOLTAGE_HI);
  state.errors = u16_from_be(rcvd_buff, MotorReadStreamRsp::Idx::ERROR_HI);

  return true;
}

void add_crc_modbus(uint8_t *buff, uint8_t len) {
  uint16_t crc = calc_crc_modbus(buff, len);
  buff[len] = (uint8_t)(crc & 0xFF);
  buff[len + 1] = (uint8_t)((crc >> 8) & 0xFF);
}

void write_motor_command_stream(const MotorCommandStreamSubCode sub_code, const uint32_t data, OrcaModbus& modbus)
{
    uint16_t i = 0;
    uint8_t send_buff[16];
    send_buff[i++] = static_cast<uint8_t>(MsgAddress::DEVICE);
    send_buff[i++] = static_cast<uint8_t>(FunctionCode::MOTOR_COMMAND_STREAM);
    send_buff[i++] = static_cast<uint8_t>(sub_code);

    // data is 32 bits - send as 4 bytes
    put_be32_ptr(&send_buff[i], data);
    i += 4;

    modbus.send_data(send_buff, i, MOTOR_COMMAND_STREAM_MSG_RSP_LEN);
}

bool parse_motor_command_stream_rsp(const uint8_t* data, const uint16_t len, ActuatorState& state)
{
    // if (len < MOTOR_COMMAND_STREAM_MSG_RSP_LEN) {
    //     return false;
    // }

    if (data[0] != static_cast<uint8_t>(MsgAddress::DEVICE)) {
        return false;
    }
    if (data[1] != static_cast<uint8_t>(FunctionCode::MOTOR_COMMAND_STREAM)) {
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

void write_motor_read_stream(const uint16_t reg_addr, const uint8_t reg_width, OrcaModbus& modbus)
{
    // assert reg_width is 1 or 2
    // assert(reg_width == 1 || reg_width == 2);

    uint16_t i = 0;
    uint8_t send_buff[16];
    send_buff[i++] = static_cast<uint8_t>(MsgAddress::DEVICE);
    send_buff[i++] = static_cast<uint8_t>(FunctionCode::MOTOR_READ_STREAM);
    send_buff[i++] = HIGHBYTE(reg_addr);
    send_buff[i++] = LOWBYTE(reg_addr);
    send_buff[i++] = reg_width;

    modbus.send_data(send_buff, i, MOTOR_READ_STREAM_MSG_RSP_LEN);
}

bool parse_motor_read_stream_rsp(const uint8_t* data, const uint16_t len, ActuatorState& state, uint32_t& reg_value)
{
    // if (len < MOTOR_READ_STREAM_MSG_RSP_LEN) {
    //     return false;
    // }

    if (data[0] != static_cast<uint8_t>(MsgAddress::DEVICE)) {
        return false;
    }
    if (data[1] != static_cast<uint8_t>(FunctionCode::MOTOR_READ_STREAM)) {
        return false;
    }

    reg_value = be32toh_ptr(&data[2]);

    state.mode = static_cast<OperatingMode>(data[6]);
    state.shaft_position = be32toh_ptr(&data[7]);
    state.force_realized = be32toh_ptr(&data[11]);
    state.power_consumed = be16toh_ptr(&data[15]);
    state.temperature = data[17];
    state.voltage = be16toh_ptr(&data[18]);
    state.errors = be16toh_ptr(&data[20]);

    return true;
}

}  // namespace orca

extern const AP_HAL::HAL& hal;

// parameters
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
    AP_GROUPINFO("F_MAX", 5, AP_IrisOrca, _f_max, 60000),

    // @Param: GAIN_P
    // @DisplayName: Position control P gain
    // @Description: Proportional gain for position control
    // @Units: 64*N/mm
    // @Range: 0 65535
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("GAIN_P", 6, AP_IrisOrca, _gain_p, 40),

    // @Param: GAIN_I
    // @DisplayName: Position control I gain
    // @Description: Integral gain for position control
    // @Units: 64*N*s/mm
    // @Range: 0 65535
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("GAIN_I", 7, AP_IrisOrca, _gain_i, 50),

    // @Param: GAIN_DV
    // @DisplayName: Position control Dv gain
    // @Description: Derivative gain for position control
    // @Units: 2*N*s/m
    // @Range: 0 65535
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("GAIN_DV", 8, AP_IrisOrca, _gain_dv, 230),

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
    AP_GROUPINFO("AZ_F_MAX", 10, AP_IrisOrca, _auto_zero_f_max, 30),

    // @Param: SAFE_DGAIN
    // @DisplayName: Safety derivative gain
    // @Description: Safety derivative gain for position control
    // @Units: 2*N*s/m
    // @Range: 0 65535
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("SAFE_DGAIN", 11, AP_IrisOrca, _safety_dgain, 0),

    // @Param: POS_FILT
    // @DisplayName: Position filter
    // @Description: Position filter for position control
    // @Units: 1/10000
    // @Range: 0 9999
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("POS_FILT", 12, AP_IrisOrca, _pos_filt, 9950),

    AP_GROUPEND
};

AP_IrisOrca::AP_IrisOrca()
 : _healthy(false)
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_IrisOrca::init()
{
    

    // create background thread to process serial input and output
    hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_IrisOrca::thread_main, void), "irisorca", 2048, AP_HAL::Scheduler::PRIORITY_RCOUT, 1);
}


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
async AP_IrisOrca::run()
{
    async_begin(&_run_state)

    OrcaModbus::ReceiveState rx;
    uint16_t value;

    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: starting");
    _healthy = false;

    _run_state.last_send_ms = AP_HAL::millis();
    await(TIME_PASSED(_run_state.last_send_ms, 5000));

    // send the PID parameters
    send_position_controller_params();
    await( (rx = _modbus.receive_state()) != OrcaModbus::ReceiveState::Pending );

    _modbus.send_write_register_cmd(orca::Register::POS_FILT, _pos_filt);
    await( (rx = _modbus.receive_state()) != OrcaModbus::ReceiveState::Pending );

    _modbus.send_write_register_cmd(orca::Register::SAFETY_DGAIN, _safety_dgain);
    await( (rx = _modbus.receive_state()) != OrcaModbus::ReceiveState::Pending );

    // read ZERO_MODE register and if the "Auto Zero on Boot (3)" is not set, set it now
    _modbus.send_read_register_cmd(orca::Register::ZERO_MODE);
    await( (rx = _modbus.receive_state()) != OrcaModbus::ReceiveState::Pending );
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: rx: %d", (uint8_t)rx);
    if (rx != OrcaModbus::ReceiveState::Ready || !_modbus.read_register(value)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Failed to read ZERO_MODE register");
        async_init(&_run_state);
        return ASYNC_CONT;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: ZERO_MODE: %u", value);

    if (value != 0x0003) {
        // set the auto zero on boot bit
        _modbus.send_write_register_cmd(orca::Register::ZERO_MODE, 0x0003);
        await( (rx = _modbus.receive_state()) != OrcaModbus::ReceiveState::Pending );
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Auto zero on boot set");

        // set auto zero max force
        _modbus.send_write_register_cmd(orca::Register::AUTO_ZERO_FORCE_N, _auto_zero_f_max);
        await( (rx = _modbus.receive_state()) != OrcaModbus::ReceiveState::Pending );

        // set the auto zero exit mode to 3 (position)
        _modbus.send_write_register_cmd(orca::Register::AUTO_ZERO_EXIT_MODE, 0x003);
        await( (rx = _modbus.receive_state()) != OrcaModbus::ReceiveState::Pending );

        // // set the persist bit
        // _modbus.send_write_register_cmd(orca::Register::CTRL_REG_2, 0x0001);
        // await( (rx = _modbus.receive_state()) != OrcaModbus::ReceiveState::Pending );
        // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Persist bit set");
    }

    // set the user comms timeout to be 300ms
    _modbus.send_write_register_cmd(orca::Register::USER_COMMS_TIMEOUT, 300);
    await( (rx = _modbus.receive_state()) != OrcaModbus::ReceiveState::Pending );

    while (true) {

        // read the state of the motor.
        orca::write_motor_read_stream(orca::Register::CTRL_REG_3, 1, _modbus);
        await( (rx = _modbus.receive_state()) != OrcaModbus::ReceiveState::Pending );
        if (rx != OrcaModbus::ReceiveState::Ready) {
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
            _modbus.send_write_register_cmd(orca::Register::CTRL_REG_3, static_cast<uint16_t>(orca::OperatingMode::AUTO_ZERO));
            await((rx = _modbus.receive_state()) != OrcaModbus::ReceiveState::Pending);

        } else if (_actuator_state.mode == orca::OperatingMode::POSITION) {
            // if it is in position control, then we exit the loop.
            break;
        }

        if (_actuator_state.errors != 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IrisOrca: actuator error: %u", _actuator_state.errors);
            async_init(&_run_state);
            return ASYNC_CONT;
        }

        _run_state.last_send_ms = AP_HAL::millis();
        await(TIME_PASSED(_run_state.last_send_ms, 100));

    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: in position mode");

    _modbus.set_recive_timeout_ms(75);

    while(true) {

        _healthy = true;

        _run_state.last_send_ms = AP_HAL::millis();

        {
        uint32_t desired_shaft_pos = get_desired_shaft_pos();
        orca::write_motor_command_stream(orca::MotorCommandStreamSubCode::POSITION_CONTROL_STREAM, desired_shaft_pos, _modbus);
        }
        await( (rx = _modbus.receive_state()) != OrcaModbus::ReceiveState::Pending );
        
        switch (rx) {
            case OrcaModbus::ReceiveState::Ready:
                orca::parse_motor_command_stream_rsp(_modbus._received_buff, _modbus._received_buff_len, _actuator_state);
                // TODO: if getting close to over temp then ramp down the max force register (keep average power ~30w)

                if (_actuator_state.errors != 0) {
                    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IrisOrca: actuator error: %u", _actuator_state.errors);
                    async_init(&_run_state);
                    return ASYNC_CONT;
                }
                break;
            case OrcaModbus::ReceiveState::Timeout:
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IrisOrca: actuator position msg timeout");
                async_init(&_run_state);
                return ASYNC_CONT;
                break;
            case OrcaModbus::ReceiveState::CRCError:
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

// consume incoming messages from actuator, reply with latest actuator speed
// runs in background thread
void AP_IrisOrca::thread_main()
{
    
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Initialized");

    // find serial driver and initialise
    const AP_SerialManager &serial_manager = AP::serialmanager();
    AP_HAL::UARTDriver* uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_IrisOrca, 0);
    if (uart != nullptr) {
        _modbus.init(uart, _pin_de);
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IrisOrca: Serial port not found");
        return;
    }

    async_init(&_run_state);
    
    while (true) {
        run();
        

        hal.scheduler->delay_microseconds(100);

        _modbus.tick();
    }
}

void AP_IrisOrca::send_position_controller_params()
{
    uint16_t registers[6];
    registers[0] = _gain_p;
    registers[1] = _gain_i;
    registers[2] = _gain_dv;
    registers[3] = _gain_de;
    registers[4] = HIGHWORD(_f_max);
    registers[5] = LOWWORD(_f_max);

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

// returns true if communicating with the actuator
bool AP_IrisOrca::healthy()
{
    return _healthy;
}




// get the AP_IrisOrca singleton
AP_IrisOrca *AP_IrisOrca::get_singleton()
{
    return _singleton;
}

AP_IrisOrca *AP_IrisOrca::_singleton = nullptr;

namespace AP {
AP_IrisOrca *irisorca()
{
    return AP_IrisOrca::get_singleton();
}
};

#endif // HAL_IRISORCA_ENABLED
