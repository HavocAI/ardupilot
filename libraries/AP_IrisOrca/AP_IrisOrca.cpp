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
#include <AP_Common/async.h>

#define IRISORCA_SERIAL_BAUD 19200                  // communication is always at 19200
#define IRISORCA_SERIAL_PARITY 2                    // communication is always even parity
#define IRISORCA_LOG_ORCA_INTERVAL_MS 5000          // log ORCA message at this interval in milliseconds
#define IRISORCA_SEND_ACTUATOR_CMD_INTERVAL_MS 100  // actuator commands sent at 10hz if connected to actuator
#define IRISORCA_REPLY_TIMEOUT_MS 25                // stop waiting for replies after 25ms
#define IRISORCA_ERROR_REPORT_INTERVAL_MAX_MS 10000 // errors reported to user at no less than once every 10 seconds

#define HIGHWORD(x) ((uint16_t)((x) >> 16))
#define LOWWORD(x) ((uint16_t)(x))

namespace orca
{

    /**
     * @brief Parse the response to a 0x06 write
     * register response message. Currently only supports
     * handling the response to a write to register 3.
     *
     * @param[in] rcvd_buff The buffer containing received response data
     * @param[in] buff_len The length of the received buffer
     * @return true response successfully parsed
     * @return false response parsing failed
     */
    static bool parse_write_register(uint8_t *rcvd_buff, uint8_t buff_len)
    {
        if (buff_len < WRITE_REG_MSG_RSP_LEN)
        {
            return false;
        }

        // Switch on the register address (bytes 2 and 3)
        switch ((rcvd_buff[WriteRegRsp::Idx::REG_ADDR_HI] << 8) |
                rcvd_buff[WriteRegRsp::Idx::REG_ADDR_LO])
        {
        case static_cast<uint16_t>(Register::CTRL_REG_3):
            // Mode of operation was set
            break;
        default:
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "IrisOrca: Unsupported write register.");
            return false;
        }

        return true;
    }

    /**
     * @brief Parse the response to a 0x10 Multiple Write Registers message.
     *
     * @param[in] rcvd_buff The buffer containing received response data
     * @param[in] buff_len The length of the received buffer
     * @param[out] state (output parameter) State data of the actuator to populate with response.
     * Currently only updates the position params and zero mode config.
     * @return true response successfully parsed
     * @return false response parsing failed
     */
    static bool parse_multiple_write_registers(uint8_t *rcvd_buff, uint8_t buff_len,
                                               ActuatorState &state)
    {
        if (buff_len < MULTIPLE_WRITE_REG_MSG_RSP_LEN)
        {
            return false;
        }

        // Switch on the register address (bytes 2 and 3)
        switch ((rcvd_buff[MultipleWriteRegRsp::Idx::REG_ADDR_HI] << 8) |
                rcvd_buff[MultipleWriteRegRsp::Idx::REG_ADDR_LO])
        {
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

    static void handle_error(uint16_t error_reg)
    {
        if (error_reg)
        {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR,
                          "IrisOrca: error_reg: 0x%x",
                          error_reg);
        }
    }

    /**
     * @brief Add the CRC to a modbus message
     *
     * @param buff Input buffer (message) to add CRC to final two bytes
     * @param len Length of the message WITHOUT 2 trailing CRC bytes.
     * Lengthstruc must be > 2 since the CRC itself is two bytes.
     */
    static void add_crc_modbus(uint8_t *buff, uint16_t len)
    {
        uint16_t crc = calc_crc_modbus(buff, len);
        buff[len] = (uint8_t)(crc & 0xFF);
        buff[len + 1] = (uint8_t)((crc >> 8) & 0xFF);
    }

} // namespace orca

/**
 * @brief Parse the response to a 0x64 Motor Command Stream message.
 *
 * @param[in] rcvd_buff The buffer containing received response data
 * @param[in] buff_len The length of the received buffer
 * @param[out] state (output parameter) Newly read state data of the actuator
 * @return true response successfully parsed
 * @return false response parsing failed
 */
bool AP_IrisOrca::parse_motor_command_stream(uint8_t *rcvd_buff, uint8_t buff_len, orca::ActuatorState &state)
{
    using namespace orca;

    if (buff_len < orca::MOTOR_COMMAND_STREAM_MSG_RSP_LEN)
    {
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

    // temperature is in degrees celsius
    state.temperature = rcvd_buff[MotorCommandStreamRsp::Idx::TEMP];

    // voltage is in units of mV
    state.voltage = u16_from_be(rcvd_buff, MotorCommandStreamRsp::Idx::VOLTAGE_HI);
    state.errors = u16_from_be(rcvd_buff, MotorCommandStreamRsp::Idx::ERROR_HI);

    handle_error(state.errors);

    struct AP_ESC_Telem_Backend::TelemetryData t = {
        .temperature_cdeg = static_cast<int16_t>(state.temperature * 100),
        .voltage = state.voltage * 0.001f,
    };
    update_telem_data(1, t,
        AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE |
        AP_ESC_Telem_Backend::TelemetryType::VOLTAGE);

    return true;
}

/**
 * @brief Parse the response to a 0x68 Motor Read Stream message.
 *
 * @param[in] rcvd_buff The buffer containing received response data
 * @param[in] buff_len The length of the received buffer
 * @param[out] state Newly read state data of the actuator
 * @return true response successfully parsed
 * @return false response parsing failed
 */
bool AP_IrisOrca::parse_motor_read_stream(uint8_t *rcvd_buff, uint8_t buff_len, orca::ActuatorState &state)
{
    using namespace orca;

    if (buff_len < orca::MOTOR_READ_STREAM_MSG_RSP_LEN)
    {
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

    // temperature is in degrees celsius
    state.temperature = rcvd_buff[MotorReadStreamRsp::Idx::TEMP];

    // voltage is in units of mV
    state.voltage = u16_from_be(rcvd_buff, MotorReadStreamRsp::Idx::VOLTAGE_HI);
    state.errors = u16_from_be(rcvd_buff, MotorReadStreamRsp::Idx::ERROR_HI);

    handle_error(state.errors);

    struct AP_ESC_Telem_Backend::TelemetryData t = {
        .temperature_cdeg = static_cast<int16_t>(state.temperature * 100),
        .voltage = state.voltage * 0.001f,
    };
    update_telem_data(1, t,
        AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE |
        AP_ESC_Telem_Backend::TelemetryType::VOLTAGE);

    return true;
}

extern const AP_HAL::HAL &hal;

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

    AP_GROUPEND};

AP_IrisOrca::AP_IrisOrca()
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_IrisOrca::init()
{
    // only init once
    // Note: a race condition exists here if init is called multiple times quickly before thread_main has a chance to set _initialise
    if (_initialised)
    {
        return;
    }

    if (!init_internals())
    {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IrisOrca: Failed to init");
        return;
    }

    // create background thread to process serial input and output
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_IrisOrca::thread_main, void), "irisorca", 2048, AP_HAL::Scheduler::PRIORITY_RCOUT, 1))
    {
        return;
    }

    _initialised = true;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Initialized");
}

// initialise serial port (run from background thread)
bool AP_IrisOrca::init_internals()
{
    // find serial driver and initialise
    const AP_SerialManager &serial_manager = AP::serialmanager();
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_IrisOrca, 0);
    if (_uart == nullptr)
    {
        return false;
    }

    _modbus.init(_uart, _pin_de);

    return true;
}

typedef struct example_state {
    async_state;
    AP_IrisOrca *self;
} example_state_t;

static async run(example_state_t *pt)
{
    async_begin(pt);

    // read ZERO_MODE register and if the "Auto Zero on Boot (3)" is not set, set it now
    pt->self->_modbus.send_read_register_cmd((uint16_t) orca::Register::ZERO_MODE);
    
    await(message_received);


    async_end;
}


static async modbus_receive()
{
    int16_t b;
    async_begin(pt);

    while(true) {
        

        if((b = _uart->read()) > 0) {

        }

    }

    async_end;
}


// consume incoming messages from actuator, reply with latest actuator speed
// runs in background thread
void AP_IrisOrca::thread_main()
{

    _control_state = orca::MotorControlState::CONFIGURING;
    bool auto_zero_commanded = false;
    bool auto_zero_in_progress = false;

    while (true)
    {
        // send a single command depending on the control state
        // or send a sleep command if there is an active error
        uint32_t now_ms = AP_HAL::millis();
        if (now_ms - _last_send_actuator_ms > IRISORCA_SEND_ACTUATOR_CMD_INTERVAL_MS)
        {
            if (_actuator_state.errors != 0)
            {
                // send sleep command if in error state to attempt to clear the error
                // Note: Errors are initialized to 2048 (comm error) so that sleep should be the
                // first command sent on boot
                if (safe_to_send())
                {
                    send_actuator_sleep_cmd();
                }
                if (now_ms - _last_error_report_ms > IRISORCA_ERROR_REPORT_INTERVAL_MAX_MS)
                {
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IrisOrca: Error %i", _actuator_state.errors);
                    _last_error_report_ms = now_ms;
                }
            }

            // no errors - execute per control state
            else
            {
                switch (_control_state)
                {
                case orca::MotorControlState::CONFIGURING:
                    // Send a write multiple registers command to set the position controller params
                    // and the auto-zero params
                    // Exit this mode to auto-zero mode if both are set
                    if (!_actuator_state.pc_params_set)
                    {
                        if (safe_to_send())
                        {
                            send_position_controller_params();
                            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Configuring position controller params");
                        }
                    }
                    else if (!_actuator_state.auto_zero_params_set)
                    {
                        if (safe_to_send())
                        {
                            send_auto_zero_params();
                            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Configuring zero params");
                        }
                    }
                    else
                    {
                        // both sets of params have been set
                        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Configuration complete");
                        _control_state = orca::MotorControlState::AUTO_ZERO;
                        if (safe_to_send())
                        {
                            // might as well send a status request during the state transition
                            send_actuator_status_request();
                        }
                    }
                    break;

                case orca::MotorControlState::AUTO_ZERO:
                    // Auto-zero mode is initiated by sending a write register command to the actuator with the
                    // mode set to AUTO_ZERO. The actuator will then transition to AUTO_ZERO op mode and will exit this mode
                    // to another op mode (we set it to enter POSITION) when the zero position is found.
                    if (!auto_zero_commanded)
                    {
                        // Initiate auto-zero mode
                        if (safe_to_send())
                        {
                            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Auto-zero commanded");
                            send_auto_zero_mode_cmd();
                            auto_zero_commanded = true;
                            auto_zero_in_progress = false;
                            break;
                        }
                    }
                    else if (auto_zero_commanded && !auto_zero_in_progress)
                    {
                        // check if the actuator reports that it is in auto-zero mode
                        if (_actuator_state.mode == orca::OperatingMode::AUTO_ZERO)
                        {
                            auto_zero_in_progress = true;
                            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Starting auto-zero");
                        }
                        else
                        {
                            // try to set the mode again on next loop (this always happens once)
                            auto_zero_commanded = false;
                            // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Auto-zero command failed");
                        }
                    }
                    else if (auto_zero_in_progress)
                    {
                        if (_actuator_state.mode == orca::OperatingMode::POSITION)
                        {
                            // auto-zero complete (Orca exits to position mode), exit to position control mode
                            auto_zero_commanded = false;
                            auto_zero_in_progress = false;
                            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Auto-zero complete");
                            _control_state = orca::MotorControlState::POSITION_CONTROL;
                        }
                        else if (_actuator_state.mode == orca::OperatingMode::SLEEP)
                        {
                            // there was an error during the auto-zero and we put the actuator into sleep mode
                            // try to set the mode again on next loop
                            auto_zero_commanded = false;
                            auto_zero_in_progress = false;
                        }
                        // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Auto-zero in progress...");
                    }
                    // read the mode of operation
                    if (safe_to_send())
                    {
                        send_actuator_status_request();
                    }
                    break;

                case orca::MotorControlState::POSITION_CONTROL:
                    // Send a position control command
                    if (safe_to_send())
                    {
                        send_actuator_position_cmd();
                    }
                    break;

                default:
                    break;
                }
            }
        }

        // 1ms loop delay
        hal.scheduler->delay_microseconds(1000);

        // check if transmit pin should be unset
        check_for_send_end();

        // check for timeout waiting for reply
        check_for_reply_timeout();

        // parse incoming characters
        uint32_t nbytes = MIN(_uart->available(), 1024U);
        while (nbytes-- > 0)
        {
            int16_t b = _uart->read();
            if (b >= 0)
            {
                if (parse_byte((uint8_t)b))
                {
                    // complete message received, parse it!
                    parse_message();
                    // clear wait-for-reply because if we are waiting for a reply, this message must be it
                    set_reply_received();
                }
            }
        }
    }
}

// returns true if communicating with the actuator
bool AP_IrisOrca::healthy()
{
    if (!_initialised)
    {
        return false;
    }

    // healthy if both receive and send have occurred in the last 3 seconds
    WITH_SEMAPHORE(_last_healthy_sem);
    const uint32_t now_ms = AP_HAL::millis();
    return ((now_ms - _last_received_ms < 3000) && (now_ms - _last_send_actuator_ms < 3000));
}

// set DE Serial CTS pin to enable sending commands to actuator
void AP_IrisOrca::send_start()
{
    // set gpio pin or serial port's CTS pin
    if (_pin_de > -1)
    {
        hal.gpio->write(_pin_de, 1);
    }
    else
    {
        _uart->set_CTS_pin(true);
    }
}

// check for timeout after sending and unset pin if required
void AP_IrisOrca::check_for_send_end()
{
    if (_send_delay_us == 0)
    {
        // not sending
        return;
    }

    if (AP_HAL::micros() - _send_start_us < _send_delay_us)
    {
        // return if delay has not yet elapsed
        return;
    }
    _send_delay_us = 0;

    // unset gpio or serial port's CTS pin
    if (_pin_de > -1)
    {
        hal.gpio->write(_pin_de, 0);
    }
    else
    {
        _uart->set_CTS_pin(false);
    }
}

// calculate delay require to allow bytes to be sent
uint32_t AP_IrisOrca::calc_send_delay_us(uint8_t num_bytes)
{
    // baud rate of 19200 bits/sec
    // total number of bits = 10 x num_bytes (no parity)
    // or 11 x num_bytes (with parity)
    // convert from seconds to micros by multiplying by 1,000,000
    // plus additional 300us safety margin
    uint8_t parity = IRISORCA_SERIAL_PARITY == 0 ? 0 : 1;
    uint8_t bits_per_data_byte = 10 + parity;
    const uint32_t delay_us = 1e6 * num_bytes * bits_per_data_byte / IRISORCA_SERIAL_BAUD + 300;
    return delay_us;
}

// check for timeout waiting for reply message
void AP_IrisOrca::check_for_reply_timeout()
{
    // return immediately if not waiting for reply
    if (_reply_wait_start_ms == 0)
    {
        return;
    }
    if (AP_HAL::millis() - _reply_wait_start_ms > IRISORCA_REPLY_TIMEOUT_MS)
    {
        _reply_wait_start_ms = 0;
    }
}

// mark reply received. should be called whenever a message is received regardless of whether we are actually waiting for a reply
void AP_IrisOrca::set_reply_received()
{
    _reply_wait_start_ms = 0;
}

void AP_IrisOrca::send_read_register_cmd(uint16_t reg_addr)
{
    using namespace orca;
    // buffer for outgoing message
    uint8_t send_buff[16];

    // set expected reply message length
    _reply_msg_len = 7;

    // build message
    uint16_t i = 0;
    send_buff[i++] = static_cast<uint8_t>(MsgAddress::DEVICE);
    send_buff[i++] = static_cast<uint8_t>(FunctionCode::READ_REGISTER);
    send_buff[i++] = HIGHBYTE(reg_addr);
    send_buff[i++] = LOWBYTE(reg_addr);
    send_buff[i++] = HIGHBYTE(1); // number of registers to read
    send_buff[i++] = LOWBYTE(1);  // number of registers to read

    // Add Modbus CRC-16
    orca::add_crc_modbus(send_buff, i);

    // set send pin
    send_start();

    // write message
    _uart->write(send_buff, i + 2);

    // record start and expected delay to send message
    _send_start_us = AP_HAL::micros();
    _send_delay_us = calc_send_delay_us(sizeof(send_buff));

    _reply_wait_start_ms = AP_HAL::millis();
}

// send a 0x06 Write Register message to the actuator
// returns true on success
bool AP_IrisOrca::write_register(uint16_t reg_addr, uint16_t reg_value)
{
    using namespace orca;
    // buffer for outgoing message
    uint8_t send_buff[WRITE_REG_MSG_LEN];

    // set expected reply message length
    _reply_msg_len = WRITE_REG_MSG_RSP_LEN;

    // build message
    send_buff[WriteReg::Idx::DEVICE_ADDR] = static_cast<uint8_t>(MsgAddress::DEVICE);
    send_buff[WriteReg::Idx::FUNCTION_CODE] = static_cast<uint8_t>(FunctionCode::WRITE_REGISTER);
    send_buff[WriteReg::Idx::REG_ADDR_HI] = HIGHBYTE(reg_addr);
    send_buff[WriteReg::Idx::REG_ADDR_LO] = LOWBYTE(reg_addr);
    send_buff[WriteReg::Idx::WRITE_DATA_HI] = HIGHBYTE(reg_value);
    send_buff[WriteReg::Idx::WRITE_DATA_LO] = LOWBYTE(reg_value);

    // Add Modbus CRC-16
    orca::add_crc_modbus(send_buff, WRITE_REG_MSG_LEN - CRC_LEN);

    // set send pin
    send_start();

    // write message
    _uart->write(send_buff, sizeof(send_buff));

    // record start and expected delay to send message
    _send_start_us = AP_HAL::micros();
    _send_delay_us = calc_send_delay_us(sizeof(send_buff));

    _reply_wait_start_ms = AP_HAL::millis();

    return true;
}

// send a 0x10 Multiple Write Registers message to the actuator
// returns true on success
bool AP_IrisOrca::write_multiple_registers(uint16_t reg_addr, uint16_t reg_count, uint8_t *data)
{
    using namespace orca;
    uint8_t msg_len = MultipleWriteReg::getMessageLength(reg_count);
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Multiple Write Registers message length: %d bytes", msg_len);
    uint8_t send_buff[msg_len];

    // set expected reply message length
    _reply_msg_len = MULTIPLE_WRITE_REG_MSG_RSP_LEN;

    // build message
    send_buff[MultipleWriteReg::Idx::DEVICE_ADDR] = static_cast<uint8_t>(MsgAddress::DEVICE);
    send_buff[MultipleWriteReg::Idx::FUNCTION_CODE] = static_cast<uint8_t>(FunctionCode::WRITE_MULTIPLE_REGISTERS);
    send_buff[MultipleWriteReg::Idx::REG_ADDR_HI] = HIGHBYTE(reg_addr);
    send_buff[MultipleWriteReg::Idx::REG_ADDR_LO] = LOWBYTE(reg_addr);
    send_buff[MultipleWriteReg::Idx::REG_COUNT_HI] = HIGHBYTE(reg_count);
    send_buff[MultipleWriteReg::Idx::REG_COUNT_LO] = LOWBYTE(reg_count);
    send_buff[MultipleWriteReg::Idx::BYTE_COUNT] = LOWBYTE(reg_count * 2);

    // copy data into message
    for (uint16_t i = 0; i < reg_count * 2; i++)
    {
        send_buff[MultipleWriteReg::DATA_START + i] = data[i];
    }

    // Add Modbus CRC-16
    orca::add_crc_modbus(send_buff, msg_len - CRC_LEN);

    // set send pin
    send_start();

    // write message
    _uart->write(send_buff, sizeof(send_buff));

    // record start and expected delay to send message
    _send_start_us = AP_HAL::micros();
    _send_delay_us = calc_send_delay_us(sizeof(send_buff));

    _reply_wait_start_ms = AP_HAL::millis();

    return true;
}

// send a 100/0x64 Motor Command Stream message to the actuator
// returns true on success
bool AP_IrisOrca::write_motor_command_stream(const uint8_t sub_code, const uint32_t data)
{
    using namespace orca;
    // buffer for outgoing message
    uint8_t send_buff[MOTOR_COMMAND_STREAM_MSG_LEN];

    // set expected reply message length
    _reply_msg_len = MOTOR_COMMAND_STREAM_MSG_RSP_LEN;

    // build message
    send_buff[MotorCommandStream::Idx::DEVICE_ADDR] = static_cast<uint8_t>(MsgAddress::DEVICE);
    send_buff[MotorCommandStream::Idx::FUNCTION_CODE] = static_cast<uint8_t>(FunctionCode::MOTOR_COMMAND_STREAM);
    send_buff[MotorCommandStream::Idx::SUB_CODE] = sub_code;
    // data is 32 bits - send as 4 bytes
    send_buff[MotorCommandStream::Idx::DATA_MSB_HI] = HIGHBYTE(HIGHWORD(data));
    send_buff[MotorCommandStream::Idx::DATA_MSB_LO] = LOWBYTE(HIGHWORD(data));
    send_buff[MotorCommandStream::Idx::DATA_LSB_HI] = HIGHBYTE(LOWWORD(data));
    send_buff[MotorCommandStream::Idx::DATA_LSB_LO] = LOWBYTE(LOWWORD(data));

    // Add Modbus CRC-16
    orca::add_crc_modbus(send_buff, MOTOR_COMMAND_STREAM_MSG_LEN - CRC_LEN);

    // set send pin
    send_start();

    // write message
    _uart->write(send_buff, sizeof(send_buff));

    // record start and expected delay to send message
    _send_start_us = AP_HAL::micros();
    _send_delay_us = calc_send_delay_us(sizeof(send_buff));

    _reply_wait_start_ms = AP_HAL::millis();

    return true;
}

// send a 0x68 Motor Read Stream message to the actuator
// returns true on success
bool AP_IrisOrca::write_motor_read_stream(const uint16_t reg_addr, const uint8_t reg_width)
{
    using namespace orca;
    // buffer for outgoing message
    uint8_t send_buff[MOTOR_READ_STREAM_MSG_LEN];

    // set expected reply message length
    _reply_msg_len = MOTOR_READ_STREAM_MSG_RSP_LEN;

    // build message
    send_buff[MotorReadStream::Idx::DEVICE_ADDR] = static_cast<uint8_t>(MsgAddress::DEVICE);
    send_buff[MotorReadStream::Idx::FUNCTION_CODE] = static_cast<uint8_t>(FunctionCode::MOTOR_READ_STREAM);
    send_buff[MotorReadStream::Idx::REG_ADDR_HI] = HIGHBYTE(reg_addr);
    send_buff[MotorReadStream::Idx::REG_ADDR_LO] = LOWBYTE(reg_addr);
    send_buff[MotorReadStream::Idx::REG_WIDTH] = reg_width;

    // Add Modbus CRC-16
    orca::add_crc_modbus(send_buff, MOTOR_READ_STREAM_MSG_LEN - CRC_LEN);

    // set send pin
    send_start();

    // write message
    _uart->write(send_buff, sizeof(send_buff));

    // record start and expected delay to send message
    _send_start_us = AP_HAL::micros();
    _send_delay_us = calc_send_delay_us(sizeof(send_buff));

    _reply_wait_start_ms = AP_HAL::millis();

    return true;
}

// perform an auto-zero by sending an auto zero command
// and waiting for the actuator to complete the zeroing process
// (transition to a mode other than auto-zero)
// returns true on success
void AP_IrisOrca::send_auto_zero_mode_cmd()
{
    // send a message
    if (write_register((uint16_t)orca::Register::CTRL_REG_3,
                       static_cast<uint8_t>(orca::OperatingMode::AUTO_ZERO)))
    {
        // record time of send for health reporting
        WITH_SEMAPHORE(_last_healthy_sem);
        _last_send_actuator_ms = AP_HAL::millis();
    }
}

// send an actuator speed position command as a value from 0 to max_travel_mm
void AP_IrisOrca::send_actuator_position_cmd()
{
    // convert yaw output to actuator output in range _pad_travel_mm to _max_travel_mm - _pad_travel_mm
    _actuator_position_desired = constrain_uint32(
        (SRV_Channels::get_output_norm(SRV_Channel::Aux_servo_function_t::k_steering) + 1) * _max_travel_mm * 0.5 * 1000,
        _pad_travel_mm * 1000, _max_travel_mm * 1000 - (_pad_travel_mm * 1000));
    // reverse direction if required
    if (_reverse_direction)
    {
        _actuator_position_desired = _max_travel_mm * 1000 - _actuator_position_desired;
    }

    // send message
    if (write_motor_command_stream((uint8_t)orca::MotorCommandStreamSubCode::POSITION_CONTROL_STREAM,
                                   _actuator_position_desired))
    {
        // record time of send for health reporting
        WITH_SEMAPHORE(_last_healthy_sem);
        _last_send_actuator_ms = AP_HAL::millis();
    }
}

// send an actuator sleep command
void AP_IrisOrca::send_actuator_sleep_cmd()
{
    // send message
    if (write_motor_command_stream((uint8_t)orca::MotorCommandStreamSubCode::SLEEP_DATA_STREAM, 0))
    {
        // record time of send for health reporting
        WITH_SEMAPHORE(_last_healthy_sem);
        _last_send_actuator_ms = AP_HAL::millis();

        // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Actuator sleep command sent");
    }
}

// send a request for actuator status
void AP_IrisOrca::send_actuator_status_request()
{
    // send message
    if (write_motor_read_stream((uint16_t)orca::Register::CTRL_REG_3, 1))
    {
        // record time of send for health reporting
        WITH_SEMAPHORE(_last_healthy_sem);
        _last_send_actuator_ms = AP_HAL::millis();
    }
}

// send a write multiple registers message to the actuator to set the position controller params
void AP_IrisOrca::send_position_controller_params()
{
    // buffer for outgoing message
    uint8_t data[12];
    data[0] = HIGHBYTE(_gain_p);
    data[1] = LOWBYTE(_gain_p);
    data[2] = HIGHBYTE(_gain_i);
    data[3] = LOWBYTE(_gain_i);
    data[4] = HIGHBYTE(_gain_dv);
    data[5] = LOWBYTE(_gain_dv);
    data[6] = HIGHBYTE(_gain_de);
    data[7] = LOWBYTE(_gain_de);
    data[8] = HIGHBYTE(static_cast<uint16_t>(_f_max << 16 >> 16));
    data[9] = LOWBYTE(static_cast<uint16_t>(_f_max << 16 >> 16));
    data[10] = HIGHBYTE(static_cast<uint16_t>(_f_max >> 16));
    data[11] = LOWBYTE(static_cast<uint16_t>(_f_max >> 16));

    // send message
    if (write_multiple_registers((uint16_t)orca::Register::PC_PGAIN, 6, (uint8_t *)data))
    {
        // record time of send for health reporting
        WITH_SEMAPHORE(_last_healthy_sem);
        _last_send_actuator_ms = AP_HAL::millis();
    }
}

// send a write multiple registers message to the actuator to set the auto-zero params
void AP_IrisOrca::send_auto_zero_params()
{
    // buffer for outgoing message
    uint8_t data[6];
    data[0] = 0x00;
    data[1] = 0x02; // Auto-zero enabled
    data[2] = HIGHBYTE(_auto_zero_f_max);
    data[3] = LOWBYTE(_auto_zero_f_max);
    data[4] = 0x00;
    data[5] = 0x03; // Exit to position mode after auto-zero

    // send message
    if (write_multiple_registers((uint16_t)orca::Register::ZERO_MODE, 3, (uint8_t *)data))
    {
        // record time of send for health reporting
        WITH_SEMAPHORE(_last_healthy_sem);
        _last_send_actuator_ms = AP_HAL::millis();
    }
}

// process a single byte received on serial port
// return true if a complete message has been received (the message will be held in _received_buff)
bool AP_IrisOrca::parse_byte(uint8_t b)
{
    bool complete_msg_received = false;

    // add b to buffer

    _received_buff[_received_buff_len] = b;
    _received_buff_len++;

    // check for a complete message
    if (_received_buff_len >= _reply_msg_len)
    {
        // check CRC of the message
        uint16_t crc_expected = calc_crc_modbus(_received_buff, _received_buff_len - orca::CRC_LEN);
        uint16_t crc_received = (_received_buff[_received_buff_len - 2]) | (_received_buff[_received_buff_len - 1] << 8);
        if (crc_expected != crc_received)
        {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IrisOrca: CRC error");
            _received_buff_len = 0;
            _parse_error_count++;
        }
        else
        {
            // CRC is correct - reset buffer and set flag
            _received_buff_len = 0;
            _parse_success_count++;
            // record time of receive for health reporting
            WITH_SEMAPHORE(_last_healthy_sem);
            _last_received_ms = AP_HAL::millis();
            complete_msg_received = true;
        }
    }
    return complete_msg_received;
}

// process message held in _received_buff
// return true if there are no errors and the message is as expected
bool AP_IrisOrca::parse_message()
{
    // check for expected reply
    switch (static_cast<orca::FunctionCode>(_received_buff[1]))
    {
    case orca::FunctionCode::WRITE_REGISTER:
        return orca::parse_write_register(_received_buff, _reply_msg_len);
    case orca::FunctionCode::WRITE_MULTIPLE_REGISTERS:
        return orca::parse_multiple_write_registers(_received_buff, _reply_msg_len, _actuator_state);
    case orca::FunctionCode::MOTOR_COMMAND_STREAM:
        return parse_motor_command_stream(_received_buff, _reply_msg_len, _actuator_state);
    case orca::FunctionCode::MOTOR_READ_STREAM:
        return parse_motor_read_stream(_received_buff, _reply_msg_len, _actuator_state);
    default:
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IrisOrca: Unexpected message");
        return false;
    }
}

// get the AP_IrisOrca singleton
AP_IrisOrca *AP_IrisOrca::get_singleton()
{
    return _singleton;
}

AP_IrisOrca *AP_IrisOrca::_singleton = nullptr;

namespace AP
{
    AP_IrisOrca *irisorca()
    {
        return AP_IrisOrca::get_singleton();
    }
};

// #endif // HAL_IRISORCA_ENABLED