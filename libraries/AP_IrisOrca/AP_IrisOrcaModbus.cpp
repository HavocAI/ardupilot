#include "AP_IrisOrcaModbus.h"
#include <AP_Math/AP_Math.h>
#include <AP_HAL/utility/sparse-endian.h>

AP_ModbusTransaction::AP_ModbusTransaction(AP_HAL::UARTDriver *uart_serial, ParseResponseCallback callback, void* cb_arg)
 :  uart(uart_serial),
    parse_response_callback(callback),
    callback_arg(cb_arg),
    state(Init)
{
}

void AP_ModbusTransaction::set_tx_data(uint8_t* data, uint8_t len)
{
    // copy the data to the write buffer
    buffer.len = MIN(len, MODBUS_MAX_MSG_LEN);
    memcpy(buffer.data, data, len);
}

void AP_ModbusTransaction::run()
{
    switch (state) {
        case Init:
            uart->discard_input();
            read_len = 0;
            // send the write buffer
            state = Send;
        FALLTHROUGH;

        case Send:
            uart->write(buffer.data, buffer.len);
            last_received_ms = AP_HAL::millis();
            state = WaitingForResponse;    
        FALLTHROUGH;

        case WaitingForResponse: {
            ssize_t bytes_read = uart->read(&buffer.data[read_len], buffer.len - read_len);
            if (bytes_read > 0) {
                last_received_ms = AP_HAL::millis();
                read_len += bytes_read;
                if (parse_response_callback(callback_arg, buffer.data, read_len)) {
                    state = Finished;
                }
            } else if (AP_HAL::millis() - last_received_ms > 1000) {
                // timeout waiting for response
                state = Timeout;
            }
            }break;

        case Finished:
            // transaction is finished
            break;
        case Timeout:
            // transaction timed out
            break;
    }
    
}

bool AP_ModbusTransaction::is_finished() const
{
    return state == Finished;
}

bool AP_ModbusTransaction::is_timeout() const
{
    return state == Timeout;
}

static void add_crc(AP_ModbusTransaction::Buffer& buffer)
{
    uint16_t crc = calc_crc_modbus(buffer.data, buffer.len);
    put_le16_ptr(&buffer.data[buffer.len], crc);
    buffer.len += 2;
}

static bool eval_crc(const uint8_t* buf, uint8_t len)
{
    uint16_t crc = calc_crc_modbus(buf, len - 2);
    uint16_t rcvd_crc = le16toh_ptr(&buf[len - 2]);
    return (crc == rcvd_crc);
}

WriteRegisterTransaction::WriteRegisterTransaction(AP_HAL::UARTDriver *uart_serial, uint16_t reg_addr, uint16_t reg_value)
 : AP_ModbusTransaction(uart_serial, parse_fn, this),
   _reg_addr(reg_addr),
   _reg_value(reg_value)
{
    
    // prepare the write buffer
    buffer.len = 0;
    buffer.data[buffer.len++] = 0x01; // device address
    buffer.data[buffer.len++] = 0x06; // function code
    put_be16_ptr(&buffer.data[buffer.len], _reg_addr); // register address
    buffer.len += 2;
    put_be16_ptr(&buffer.data[buffer.len], _reg_value); // register value
    buffer.len += 2;

    add_crc(buffer);
}

bool WriteRegisterTransaction::parse_fn(void* self, uint8_t *rcvd_buff, uint8_t buff_len)
{
    const WriteRegisterTransaction* tx = static_cast<WriteRegisterTransaction*>(self);

    if (buff_len != 8) {
        return false;
    }

    if (rcvd_buff[0] != 0x01) {
        return false;
    }
    if (rcvd_buff[1] != 0x06) {
        return false;
    }

    if (tx->_reg_addr != be16toh_ptr(&rcvd_buff[2])) {
        return false;
    }

    if (tx->_reg_value != be16toh_ptr(&rcvd_buff[4])) {
        return false;
    }

    return eval_crc(rcvd_buff, buff_len);
}

ReadRegisterTransaction::ReadRegisterTransaction(AP_HAL::UARTDriver *uart_serial, uint16_t reg_addr)
 : AP_ModbusTransaction(uart_serial, parse_fn, this)
{
    // prepare the write buffer
    buffer.len = 0;
    buffer.data[buffer.len++] = 0x01; // device address
    buffer.data[buffer.len++] = 0x03; // function code
    put_be16_ptr(&buffer.data[buffer.len], reg_addr); // register address
    buffer.len += 2;
    put_be16_ptr(&buffer.data[buffer.len], 1); // number of registers to read
    buffer.len += 2;

    add_crc(buffer);
}

bool ReadRegisterTransaction::parse_fn(void* self, uint8_t *rcvd_buff, uint8_t buff_len)
{
    ReadRegisterTransaction* tx = static_cast<ReadRegisterTransaction*>(self);

    if (buff_len != 7) {
        return false;
    }

    if (rcvd_buff[0] != 0x01) {
        return false;
    }
    if (rcvd_buff[1] != 0x03) {
        return false;
    }

    tx->_reg_value = be16toh_ptr(&rcvd_buff[3]);

    return eval_crc(rcvd_buff, buff_len);
}

