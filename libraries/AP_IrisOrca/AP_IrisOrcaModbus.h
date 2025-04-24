#pragma once

#include "AP_IrisOrca_config.h"

#if HAL_IRISORCA_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/async.h>

#define MODBUS_MAX_MSG_LEN 64

class AP_ModbusTransaction
{
public:

    // callback function type to try to parse the response
    typedef bool (*ParseResponseCallback)(void*, uint8_t *rcvd_buff, uint8_t buff_len);

    typedef struct Buffer {
        uint8_t data[MODBUS_MAX_MSG_LEN];
        uint8_t len;
    } Buffer;

    AP_ModbusTransaction(AP_HAL::UARTDriver *uart, ParseResponseCallback callback, void* cb_arg = nullptr);

    void set_tx_data(uint8_t* data, uint8_t len);
    
    void run();

    bool is_finished() const;
    bool is_timeout() const;

protected:
    Buffer buffer;

private:
    AP_HAL::UARTDriver *uart;
    ParseResponseCallback parse_response_callback;
    void* callback_arg;
    size_t read_len;
    uint32_t sent_ts_ms;
    uint32_t last_received_ms;
    

    enum ModbusState {
        Init = 0,
        Send,
        WaitingForResponse,
        Finished,
        Timeout,
    };

    ModbusState state;

};

class WriteRegisterTransaction : AP_ModbusTransaction {
    public:
        WriteRegisterTransaction(AP_HAL::UARTDriver *uart, uint16_t reg_addr, uint16_t reg_value);
    
    private:
        uint16_t _reg_addr;
        uint16_t _reg_value;

        static bool parse_fn(void* context, uint8_t *rcvd_buff, uint8_t buff_len);
        
};

class ReadRegisterTransaction : AP_ModbusTransaction {
    public:
        ReadRegisterTransaction(AP_HAL::UARTDriver *uart, uint16_t reg_addr);
    
        uint16_t reg_value() const { return _reg_value; }
    
    private:
        uint16_t _reg_value;

        static bool parse_fn(void* context, uint8_t *rcvd_buff, uint8_t buff_len);
};



#endif // HAL_IRISORCA_ENABLED
