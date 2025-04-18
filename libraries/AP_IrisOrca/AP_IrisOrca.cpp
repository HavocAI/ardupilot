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

// #if HAL_IRISORCA_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_SerialManager/AP_SerialManager.h>

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

    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IrisOrca: starting");

    _run_state.wait_timer = AP_HAL::millis();
    await(TIME_PASSED(_run_state.wait_timer, 5000));

    _modbus.send_write_register_cmd(orca::Register::CTRL_REG_3, 0x1);
    await( (rx = _modbus.receive_state()) != OrcaModbus::ReceiveState::Pending );

    _run_state.wait_timer = AP_HAL::millis();
    await(TIME_PASSED(_run_state.wait_timer, 1000));

    // read ZERO_MODE register and if the "Auto Zero on Boot (3)" is not set, set it now
    _modbus.send_read_register_cmd(orca::Register::ZERO_MODE);
    await( (rx = _modbus.receive_state()) != OrcaModbus::ReceiveState::Pending );
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: rx: %u", rx);
    if (rx != OrcaModbus::ReceiveState::Ready || !_modbus.read_register(value)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IrisOrca: Failed to read ZERO_MODE register");
        async_init(&_run_state);
        return ASYNC_CONT;
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

// returns true if communicating with the actuator
bool AP_IrisOrca::healthy()
{
    return true;
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

// #endif // HAL_IRISORCA_ENABLED
