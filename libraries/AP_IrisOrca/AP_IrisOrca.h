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
 
/*
   This driver supports communicating with the Iris Orca linear actuator via RS485
   ModBus protocol.
   For further documentation, see: <https://irisdynamics.com/downloads>
 */

#pragma once

#include "AP_IrisOrca_config.h"
#include "modbus.h"

#if HAL_IRISORCA_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>

#define IRISORCA_MESSAGE_LEN_MAX    35  // messages are no more than 35 bytes

namespace orca {

    

    static constexpr uint8_t CRC_LEN = 2;

    // message addresses
    enum class MsgAddress : uint8_t {
        BUS_MASTER = 0x00,
        DEVICE = 0x01
    };

    // function codes
    enum class FunctionCode : uint8_t {
        READ_REGISTER = 0x03,
        WRITE_REGISTER = 0x06,
        WRITE_MULTIPLE_REGISTERS = 0x10,
        MOTOR_COMMAND_STREAM = 0x64,
        MOTOR_READ_STREAM = 0x68
    };

    // sub codes for MOTOR_COMMAND_STREAM
    enum class MotorCommandStreamSubCode : uint8_t {
        FORCE_CONTROL_STREAM = 0x1C,
        POSITION_CONTROL_STREAM = 0x1E,
        SLEEP_DATA_STREAM = 0x00 // sleep data stream is everything else
    };

    // registers
    enum class Register : uint16_t {
        CTRL_REG_0 = 0,
        CTRL_REG_2 = 2,
        CTRL_REG_3 = 3,
        CTRL_REG_4 = 4,
        POS_CMD = 30,
        POS_CMD_H = 31,
        PC_PGAIN = 133,
        PC_IGAIN = 134,
        PC_DVGAIN = 135,
        PC_DEGAIN = 136,
        PC_FSATU = 137,
        PC_FSATU_H = 138,
        ZERO_MODE = 171,
        AUTO_ZERO_FORCE_N = 172,
        AUTO_ZERO_EXIT_MODE = 173
    };

    enum class OperatingMode : uint8_t {
        SLEEP = 1,
        FORCE = 2,
        POSITION = 3,
        HAPTIC = 4,
        KINEMATIC = 5,
        AUTO_ZERO = 55,
    };

    // motor control states
    enum class MotorControlState : uint8_t {
        CONFIGURING = 0,
        AUTO_ZERO = 1,
        POSITION_CONTROL = 2
    };


    // Specifies the meaning of the indices used for the modbus rtu
    // function 0x06, write single register
    static constexpr uint8_t WRITE_REG_MSG_LEN = 8;
    struct WriteReg {
        enum Idx {
            DEVICE_ADDR = 0,
            FUNCTION_CODE = 1,
            REG_ADDR_HI = 2,
            REG_ADDR_LO = 3,
            WRITE_DATA_HI = 4,
            WRITE_DATA_LO = 5,
            CRC_LO = 6,
            CRC_HI = 7
        };
    };


    // Specifies the meaning of the indices used for the response to the 
    // modbus rtu function 0x06, write single register
    static constexpr uint8_t WRITE_REG_MSG_RSP_LEN = 8;
    struct WriteRegRsp {
        enum Idx {
            DEVICE_ADDR = 0,
            FUNCTION_CODE = 1,
            REG_ADDR_HI = 2,
            REG_ADDR_LO = 3,
            DATA_HI = 4,
            DATA_LO = 5,
            CRC_LO = 6,
            CRC_HI = 7
        };
    };

    // Specifies the meaning of the indices used for the modbus rtu
    // function 0x10, write multiple registers
    struct MultipleWriteReg {
        enum Idx {
            DEVICE_ADDR = 0,
            FUNCTION_CODE = 1,
            REG_ADDR_HI = 2,
            REG_ADDR_LO = 3,
            REG_COUNT_HI = 4,
            REG_COUNT_LO = 5,
            BYTE_COUNT = 6,
            DATA_START = 7
            // CRC indices will be calculated based on the number of registers
        };

        // Calculate the length of the message based on the number of registers
        static constexpr uint8_t getMessageLength(uint8_t numRegisters) {
            return 9 + (numRegisters * 2); // 2 bytes per register + 2 bytes for CRC
        }
    };

    // Specifies the meaning of the indices used for the response to the
    // modbus rtu function 0x10, write multiple registers
    static constexpr uint8_t MULTIPLE_WRITE_REG_MSG_RSP_LEN = 8;
    struct MultipleWriteRegRsp {
        enum Idx {
            DEVICE_ADDR = 0,
            FUNCTION_CODE = 1,
            REG_ADDR_HI = 2,
            REG_ADDR_LO = 3,
            REG_COUNT_HI = 4,
            REG_COUNT_LO = 5,
            CRC_LO = 6,
            CRC_HI = 7
        };
    };

    // Specifies the data layout of the orca-specific motor
    // command stream message, 0x64
    static constexpr uint8_t MOTOR_COMMAND_STREAM_MSG_LEN  = 9; 
    struct MotorCommandStream {
        enum Idx {
            DEVICE_ADDR = 0,
            FUNCTION_CODE = 1,
            SUB_CODE = 2,
            DATA_MSB_HI = 3,
            DATA_MSB_LO = 4,
            DATA_LSB_HI = 5,
            DATA_LSB_LO = 6,
            CRC_LO = 7,
            CRC_HI = 8,
        };
    };


    static constexpr uint8_t MOTOR_COMMAND_STREAM_MSG_RSP_LEN = 19;
    struct MotorCommandStreamRsp {
        enum Idx {
            DEVICE_ADDR = 0,
            FUNCTION_CODE = 1,
            POSITION_MSB_HI = 2,
            POSITION_MSB_LO = 3,
            POSITION_LSB_HI = 4,
            POSITION_LSB_LO = 5,
            FORCE_MSB_HI = 6,
            FORCE_MSB_LO = 7,
            FORCE_LSB_HI = 8,
            FORCE_LSB_LO = 9,
            POWER_HI = 10,
            POWER_LO = 11,
            TEMP = 12,
            VOLTAGE_HI = 13,
            VOLTAGE_LO = 14,
            ERROR_HI = 15,
            ERROR_LO = 16,
            CRC_LO = 17,
            CRC_HI = 18,
        };
    };


    // Specifies the data layout of the orca-specific motor
    // read stream message, 0x68
    static constexpr uint8_t MOTOR_READ_STREAM_MSG_LEN = 7;
    struct MotorReadStream {
        enum Idx {
            DEVICE_ADDR = 0,
            FUNCTION_CODE = 1,
            REG_ADDR_HI = 2,
            REG_ADDR_LO = 3,
            REG_WIDTH = 4,
            CRC_LO = 5,
            CRC_HI = 6,
        };
    };


    static constexpr uint8_t MOTOR_READ_STREAM_MSG_RSP_LEN = 24;
    struct MotorReadStreamRsp {
        enum Idx {
            DEVICE_ADDR = 0,
            FUNCTION_CODE = 1,
            REG_DATA_MSB_HI = 2,
            REG_DATA_MSB_LO = 3,
            REG_DATA_LSB_HI = 4,
            REG_DATA_LSB_LO = 5,
            MODE = 6,
            POSITION_MSB_HI = 7,
            POSITION_MSB_LO = 8,
            POSITION_LSB_HI = 9,
            POSITION_LSB_LO = 10,
            FORCE_MSB_HI = 11,
            FORCE_MSB_LO = 12, 
            FORCE_LSB_HI = 13,
            FORCE_LSB_LO = 14,
            POWER_HI = 15,
            POWER_LO = 16,
            TEMP = 17,
            VOLTAGE_HI = 18,
            VOLTAGE_LO = 19,
            ERROR_HI = 20,
            ERROR_LO = 21,
            CRC_LO = 22,
            CRC_HI = 23,
        };
    };

    struct ActuatorState {
        OperatingMode mode;
        uint32_t shaft_position;
        uint32_t force_realized;
        uint16_t power_consumed;
        uint8_t temperature;
        uint16_t voltage;
        uint16_t errors{2048};
        bool pc_params_set{false};
        bool auto_zero_params_set{false};
    };

    inline uint16_t u16_from_be(uint8_t *bytes, uint8_t start_idx) {
        return uint16_t(bytes[start_idx] << 8 | bytes[start_idx + 1]);
    }

    inline uint32_t u32_from_be(uint8_t *bytes, uint8_t start_idx) {
        return uint32_t(bytes[start_idx] << 24 |
                        bytes[start_idx + 1] << 16 |
                        bytes[start_idx + 2] << 8 |
                        bytes[start_idx + 3]);
    }

}

class AP_IrisOrca : public AP_ESC_Telem_Backend {
public:
    AP_IrisOrca();

    CLASS_NO_COPY(AP_IrisOrca);

    static AP_IrisOrca* get_singleton();

    // initialise driver
    void init();

    // consume incoming messages from actuator, reply with latest actuator position
    // runs in background thread
    void thread_main();

    // returns true if communicating with the actuator
    bool healthy();

    static const struct AP_Param::GroupInfo var_info[];

    OrcaModbus _modbus;

private:

    orca::ActuatorState _actuator_state;
    orca::MotorControlState _control_state;

    // initialise serial port (run from background thread)
    // returns true on success
    bool init_internals();

    // returns true if it is safe to send a message
    bool safe_to_send() const { return ((_send_delay_us == 0) && (_reply_wait_start_ms == 0)); }

    // set pin to enable sending a message
    void send_start();

    // check for timeout after sending a message and unset pin if required
    void check_for_send_end();

    // calculate delay required to allow message to be completely sent
    uint32_t calc_send_delay_us(uint8_t num_bytes);

    // record msgid of message to wait for and set timer for reply timeout handling
    void set_expected_reply_msgid(uint8_t msg_id);

    // check for timeout waiting for reply
    void check_for_reply_timeout();

    // mark reply received. should be called whenever a message is received 
    // regardless of whether we are actually waiting for a reply
    void set_reply_received();

    void send_read_register_cmd(uint16_t reg_addr);

    // send a 0x06 Write Register message to the actuator
    // returns true on success
    bool write_register(uint16_t reg_addr, uint16_t reg_value);

    // send a 0x10 Multiple Write Registers message to the actuator
    // starting from the given register address
    // returns true on success
    bool write_multiple_registers(uint16_t reg_addr, uint16_t reg_count, uint8_t *data);

    // send a 0x64 Motor Command Stream message to the actuator
    // returns true on success
    bool write_motor_command_stream(uint8_t sub_code, uint32_t data);

    // send a 0x68 Motor Read Stream message to the actuator
    // returns true on success
    bool write_motor_read_stream(uint16_t reg_addr, uint8_t reg_width);

    // send an auto zero mode command
    void send_auto_zero_mode_cmd();

    // send an actuator position command as a value from 0 to the maximum travel
    // value is taken directly from the steering servo channel
    void send_actuator_position_cmd();

    // send a actuator sleep command
    void send_actuator_sleep_cmd();

    // send a request for actuator status
    void send_actuator_status_request();

    // send a write multiple registers message to the actuator to set the 
    // position controller params
    void send_position_controller_params();

    // send a write multiple registers message to the actuator to set the
    // auto zero params
    void send_auto_zero_params();

    // process a single byte received on serial port
    // return true if a complete message has been received (the message will be held in _received_buff)
    bool parse_byte(uint8_t b);

    // process message held in _received_buff
    // return true if the message was as expected and there are no actuator errors
    bool parse_message();

    bool parse_motor_command_stream(uint8_t *rcvd_buff, uint8_t buff_len, orca::ActuatorState &state);
    bool parse_motor_read_stream(uint8_t *rcvd_buff, uint8_t buff_len, orca::ActuatorState &state);


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

    // members
    AP_HAL::UARTDriver *_uart;          // serial port to communicate with actuator
    bool _initialised;                  // true once driver has been initialised
    uint32_t _actuator_position_desired; // desired actuator position (set from within update method)
    uint32_t _last_send_actuator_ms;    // system time (in millis) last actuator position command was sent (used for health reporting)
    uint32_t _send_start_us;            // system time (in micros) when last message started being sent (used for timing to unset DE pin)
    uint32_t _send_delay_us;            // delay (in micros) to allow bytes to be sent after which pin can be unset.  0 if not delaying
    uint32_t _last_error_report_ms;     // system time (in millis) that last error was reported (used to avoid spamming errors)

    // health reporting
    HAL_Semaphore _last_healthy_sem;// semaphore protecting reading and updating of _last_send_actuator_ms and _last_received_ms

    // message parsing members
    uint32_t _parse_error_count;    // total number of parsing errors (for reporting)
    uint32_t _parse_success_count;  // number of messages successfully parsed (for reporting)
    uint8_t _received_buff[IRISORCA_MESSAGE_LEN_MAX];   // characters received
    uint8_t _received_buff_len;     // number of characters received
    uint32_t _last_received_ms;     // system time (in millis) that a message was successfully parsed (for health reporting)

    // reply message handling
    uint8_t _reply_msgid;           // replies expected msgid (reply often does not specify the msgid so we must record it)
    uint8_t _reply_msg_len;         // length of reply message expected (total including CRC)
    uint32_t _reply_wait_start_ms;  // system time that we started waiting for a reply message

    static AP_IrisOrca *_singleton;     // singleton instance

};

namespace AP {
    AP_IrisOrca *irisorca();
};

#endif // HAL_IRISORCA_ENABLED