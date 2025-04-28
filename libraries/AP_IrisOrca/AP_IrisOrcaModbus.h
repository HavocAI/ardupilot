#pragma once

#include "AP_IrisOrca_config.h"

#if HAL_IRISORCA_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/async.h>

#define MODBUS_MAX_MSG_LEN 64

void init_uart_for_modbus(AP_HAL::UARTDriver *uart);

class AP_ModbusTransaction
{
public:
    typedef struct Buffer {
        uint8_t data[MODBUS_MAX_MSG_LEN];
        uint8_t len;
    } Buffer;

    AP_ModbusTransaction() = default;
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

    void wait_us(uint32_t us);
    

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
        WriteRegisterTransaction() = default;
        WriteRegisterTransaction(AP_HAL::UARTDriver *uart, uint16_t reg_addr, uint16_t reg_value);
        CLASS_NO_COPY(WriteRegisterTransaction);
    
    protected:
        virtual bool parse_response(uint8_t byte) override;
    private:
        uint16_t _reg_addr;
        uint16_t _reg_value;

        enum class ParseState {
            Init = 0,
            Start,
            Finished
        };

        ParseState _parse_state;

        bool parse_msg(uint8_t *rcvd_buff, uint8_t buff_len);
        
};

class ReadRegisterTransaction : public AP_ModbusTransaction {
    public:
        ReadRegisterTransaction() = default;
        ReadRegisterTransaction(AP_HAL::UARTDriver *uart, uint16_t reg_addr);
        CLASS_NO_COPY(ReadRegisterTransaction);
    
        uint16_t reg_value() const { return _reg_value; }

    protected:
        virtual bool parse_response(uint8_t byte) override;
    private:
        uint16_t _reg_value;

        enum class ParseState {
            Init = 0,
            Start,
            Finished
        };

        ParseState _parse_state;

        bool parse_msg(uint8_t *rcvd_buff, uint8_t buff_len);
};



#endif // HAL_IRISORCA_ENABLED
