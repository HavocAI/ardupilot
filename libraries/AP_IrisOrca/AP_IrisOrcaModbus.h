#pragma once

#include "AP_IrisOrca_config.h"

#if HAL_IRISORCA_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/async.h>

#define MODBUS_MAX_MSG_LEN 64


namespace orca {
    // message addresses
    enum MsgAddress : uint8_t {
        BUS_MASTER = 0x00,
        DEVICE = 0x01
    };

    // function codes
    enum FunctionCode : uint8_t {
        WRITE_REGISTER = 0x06,
        WRITE_MULTIPLE_REGISTERS = 0x10,
        MOTOR_COMMAND_STREAM = 0x64,
        MOTOR_READ_STREAM = 0x68
    };

    // sub codes for MOTOR_COMMAND_STREAM
    enum MotorCommandStreamSubCode : uint8_t {
        FORCE_CONTROL_STREAM = 0x1C,
        POSITION_CONTROL_STREAM = 0x1E,
        SLEEP_DATA_STREAM = 0x00 // sleep data stream is everything else
    };

    // registers
    enum Register : uint16_t {
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
        USER_MAX_FORCE = 140,
        USER_MAX_FORCE_H = 141,
        USER_COMMS_TIMEOUT = 163,
        ZERO_MODE = 171,
        AUTO_ZERO_FORCE_N = 172,
        AUTO_ZERO_EXIT_MODE = 173,
        SERIAL_NUMBER_LOW = 406,
        SERIAL_NUMBER_HIGH = 407,
        MAJOR_VERSION = 408,
        RELEASE_STATE = 409,
        REVISION_NUMBER = 410,
        COMMIT_ID_LOW = 411,
        COMMIT_ID_HIGH = 412,
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

    struct ActuatorState {
        int32_t shaft_position;
        int32_t force_realized;
        uint16_t power_consumed;
        uint8_t temperature;
        uint16_t voltage;
        uint16_t errors;
    };
}

void init_uart_for_modbus(AP_HAL::UARTDriver *uart);

class AP_ModbusTransaction
{
public:
    typedef struct Buffer {
        uint8_t data[MODBUS_MAX_MSG_LEN];
        uint8_t len;
    } Buffer;

    AP_ModbusTransaction() {};
    AP_ModbusTransaction(AP_HAL::UARTDriver *uart);
    // CLASS_NO_COPY(AP_ModbusTransaction);

    void set_tx_data(uint8_t* data, uint8_t len);
    
    /**
     * run the transaction state machine
     * @return true if the transaction is finished
     */
    bool run();

    bool is_finished() const;
    bool is_timeout() const;

protected:
    virtual bool parse_response(uint8_t byte) = 0;
    Buffer buffer;

private:
    AP_HAL::UARTDriver *uart;
    uint8_t read_len;
    uint32_t start_wait_us;
    uint32_t last_received_ms;

    /**
     * Wait for a specified number of microseconds.
     * Keep calling this function until the wait is finished.
     * @param us The number of microseconds to wait.
     * @return true if the wait is finished, false otherwise.
     */
    bool wait_us(uint32_t us);
    

    enum ModbusState {
        Init = 0,
        Send,
        WaitingToFinshSend,
        WaitingForResponse,
        Finished,
        Timeout,
    };

    ModbusState state;

};

class WriteRegisterTransaction : public AP_ModbusTransaction {
    public:
        WriteRegisterTransaction() {};
        WriteRegisterTransaction(AP_HAL::UARTDriver *uart, uint16_t reg_addr, uint16_t reg_value);
        // CLASS_NO_COPY(WriteRegisterTransaction);
    
    protected:
        bool parse_response(uint8_t byte) override;
    private:
        uint16_t _reg_addr;
        uint16_t _reg_value;

        enum class ParseState {
            Init = 0,
            Shift,
            Finished
        };

        ParseState _parse_state;

        bool parse_msg(uint8_t *rcvd_buff, uint8_t buff_len);
        
};

class ReadRegisterTransaction : public AP_ModbusTransaction {
    public:
        ReadRegisterTransaction() {};
        ReadRegisterTransaction(AP_HAL::UARTDriver *uart, uint16_t reg_addr);
        // CLASS_NO_COPY(ReadRegisterTransaction);
    
        uint16_t reg_value() const { return _reg_value; }

    protected:
        bool parse_response(uint8_t byte) override;
    private:
        uint16_t _reg_value;

        enum class ParseState {
            Init = 0,
            Shift,
            Finished
        };

        ParseState _parse_state;

        bool parse_msg(uint8_t *rcvd_buff, uint8_t buff_len);
};

class WriteMotorCmdStreamTransaction : public AP_ModbusTransaction {
    public:
        WriteMotorCmdStreamTransaction() {};
        WriteMotorCmdStreamTransaction(AP_HAL::UARTDriver *uart, uint8_t sub_code, uint32_t data);

        orca::ActuatorState actuator_state() const { return _actuator_state; }
    
    private:
        orca::ActuatorState _actuator_state;

        enum class ParseState {
            Init = 0,
            Shift,
            Finished
        };

        ParseState _parse_state;

        bool parse_response(uint8_t byte) override;

        bool parse_msg(uint8_t *rcvd_buff, uint8_t buff_len);
};

class ReadMotorStreamTransaction : public AP_ModbusTransaction {
    public:
        ReadMotorStreamTransaction() {};
        ReadMotorStreamTransaction(AP_HAL::UARTDriver *uart, uint16_t regaddr, uint8_t reg_width);

        orca::ActuatorState actuator_state() const { return _actuator_state; }
        orca::OperatingMode operating_mode() const { return _operating_mode; }
        uint32_t reg_value() const { return _reg_value; }
    
    private:
        orca::OperatingMode _operating_mode;
        orca::ActuatorState _actuator_state;
        uint32_t _reg_value;

        enum class ParseState {
            Init = 0,
            Shift,
            Finished
        };

        ParseState _parse_state;

        bool parse_response(uint8_t byte) override;

        bool parse_msg(uint8_t *rcvd_buff, uint8_t buff_len);
};



#endif // HAL_IRISORCA_ENABLED
