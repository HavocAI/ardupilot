#ifndef ORCA_MODBUS_H
#define ORCA_MODBUS_H

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

#define IRISORCA_MESSAGE_LEN_MAX    35  // messages are no more than 35 bytes

class OrcaModbus;

namespace orca {
    
    enum Register : uint16_t {
        CTRL_REG_0 = 0,
        CTRL_REG_1 = 1,
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
        SAFETY_DGAIN = 143,
        PC_SOFTSTART_PERIOD = 150,
        USER_COMMS_TIMEOUT = 163,
        POS_FILT = 167,
        ZERO_MODE = 171,
        AUTO_ZERO_FORCE_N = 172,
        AUTO_ZERO_EXIT_MODE = 173
    };

    enum MsgAddress : uint8_t {
        BUS_MASTER = 0x00,
        DEVICE = 0x01
    };

    enum OperatingMode : uint8_t {
        SLEEP = 1,
        FORCE = 2,
        POSITION = 3,
        HAPTIC = 4,
        KINEMATIC = 5,
        AUTO_ZERO = 55,
    };

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
        KINEMATIC_CONTROL_STREAM = 0x20,
        HAPTIC_CONTROL_STREAM = 0x22,
        SLEEP_DATA_STREAM = 0x00 // sleep data stream is everything else
    };

    struct ActuatorState {
        OperatingMode mode;
        uint32_t shaft_position;
        uint32_t force_realized;
        uint16_t power_consumed;
        uint8_t temperature;
        uint16_t voltage;
        uint16_t errors;
    };

    static constexpr uint16_t MOTOR_COMMAND_STREAM_MSG_RSP_LEN = 19;
    static constexpr uint16_t MOTOR_READ_STREAM_MSG_RSP_LEN = 24;

    // void write_position_ctl_max_force(uint16_t max_force, OrcaModbus* modbus)
    // {
    //     uint16_t registers[2];
    //     registers[0] = LOWWORD(max_force);
    //     registers[1] = HIGHWORD(max_force);
        
    //     modbus->send_write_multiple_registers(Register::PC_FSATU, 2, registers);
    // }

    void write_motor_command_stream(const MotorCommandStreamSubCode sub_code, const uint32_t data, OrcaModbus& modbus);
    bool parse_motor_command_stream_rsp(const uint8_t* data, const uint16_t len, ActuatorState& state);

    void write_motor_read_stream(const uint16_t reg_addr, const uint8_t reg_width, OrcaModbus& modbus);
    bool parse_motor_read_stream_rsp(const uint8_t* data, const uint16_t len, ActuatorState& state, uint32_t& reg_value);

}

class OrcaModbus
{
    public:

        enum class TransceiverState {
            Idle,
            Sending,
            Receiving,
            Ready,
            Timeout,
            CRCError,
        };

        friend class AP_IrisOrca;
        
        OrcaModbus();
        CLASS_NO_COPY(OrcaModbus);

        void init(AP_HAL::UARTDriver*, int pin_de);
        void send_read_register_cmd(uint16_t reg_addr);
        void send_write_register_cmd(uint16_t reg_addr, uint16_t reg_value);
        void send_write_multiple_registers(uint16_t reg_addr, uint16_t reg_count, uint16_t *registers);
        
        /**
         * adds CRC-16 to the end of the message and then transmits
         */
        void send_data(uint8_t *data, uint16_t len, uint16_t expected_reply_len);
        

        /**
         * @breif read a 16-bit modbus register from the receive buf
         * @param reg[out] reference to the register to be read
         * @returns true if the register was read successfully
         * Note: the `send_read_register_cmd()` should have been called before this
         * and the `message received()` should be true
         */
        bool read_register(uint16_t& reg);

        TransceiverState transceiver_state() { return _transceiver_state; };

        inline bool tx_rx_finished() {
            return _transceiver_state == TransceiverState::Ready ||
                    _transceiver_state == TransceiverState::Timeout ||
                    _transceiver_state == TransceiverState::CRCError;
        }

        // void set_recive_timeout_ms(uint32_t timeout_ms);

        /**
         * @brief perform various tasks such as reading from the serial port
         * and checking for timeouts
         * @note this should be continuously called from the main loop
         */
        void tick();

        
    
    private:
        AP_HAL::UARTDriver *_uart;          // serial port to communicate with actuator
        int _pin_de;
        uint16_t _reply_msg_len;
        uint8_t _received_buff[IRISORCA_MESSAGE_LEN_MAX];
        uint16_t _received_buff_len;
        uint32_t _send_start_us;            // system time (in micros) when last message started being sent (used for timing to unset DE pin)
        uint32_t _tx_time_us;               // time to transmit the message
        uint32_t _reply_wait_start_ms;  // system time that we started waiting for a reply message
        
        TransceiverState _transceiver_state;
        


};

#endif // ORCA_MODBUS_H