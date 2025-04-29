#include "AP_IrisOrcaModbus.h"
#include <AP_Math/AP_Math.h>
#include <AP_HAL/utility/sparse-endian.h>

#define IRISORCA_SERIAL_BAUD                    19200   // communication is always at 19200
#define IRISORCA_SERIAL_PARITY                  2       // communication is always even parity


#define DEBUG
#define SOFTWARE_FLOWCONTROL
#define IO_ASYNC_WAIT

#ifdef DEBUG
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>

static void debug_print_buf(uint8_t* buf, int len, const char* prefix)
{
    char str[256];
    for (int i = 0; i < len; i++) {
        snprintf(str + (i * 3), sizeof(str) - (i * 3), "%02X ", buf[i]);
    }
    str[len * 3] = '\0'; // null terminate the string

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: %s %s", prefix, str);
}
#endif


#ifdef SOFTWARE_FLOWCONTROL
static uint32_t calc_transmit_time_us(uint8_t num_bytes)
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
#endif

void init_uart_for_modbus(AP_HAL::UARTDriver *uart)
{
    uart->end();
    
    uart->configure_parity(2); // even parity
    uart->set_stop_bits(1);

#ifdef SOFTWARE_FLOWCONTROL
    uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    uart->set_unbuffered_writes(true);
    uart->set_CTS_pin(false);
#else
    uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_RTS_DE);
#endif // SOFTWARE_FLOWCONTROL

    uart->set_options(
                    uart->get_options()
                     | AP_HAL::UARTDriver::OPTION_NODMA_RX
                    // | AP_HAL::UARTDriver::OPTION_NOFIFO
                );

    // uart->set_unbuffered_writes(true);
    

    uart->begin(IRISORCA_SERIAL_BAUD, 16, 16);
    uart->discard_input();

    // uart->set_options(uart->get_options() |
    //                 AP_HAL::UARTDriver::OPTION_NODMA_RX
    //                 // AP_HAL::UARTDriver::OPTION_NOFIFO
    //             );
}

AP_ModbusTransaction::AP_ModbusTransaction(AP_HAL::UARTDriver *uart_serial)
 :  uart(uart_serial),
    state(ModbusState::Init)
{
}

void AP_ModbusTransaction::set_tx_data(uint8_t* data, uint8_t len)
{
    // copy the data to the write buffer
    buffer.len = MIN(len, MODBUS_MAX_MSG_LEN);
    memcpy(buffer.data, data, len);
}

bool AP_ModbusTransaction::wait_us(uint32_t us)
{
#ifdef IO_ASYNC_WAIT
    if (AP_HAL::micros() - start_wait_us > us) {
        return true;
    } else {
        return false;
    }
#else
    AP_HAL::get_HAL().scheduler->delay_microseconds(us);
    return true;
#endif
}

bool AP_ModbusTransaction::run()
{
    switch (state) {
        case Init:
            uart->discard_input();
            read_len = 0;
            // send the write buffer
            state = Send;
        FALLTHROUGH;

        case Send:
#ifdef DEBUG
            debug_print_buf(buffer.data, buffer.len, "=> ");
#endif // DEBUG

#ifdef SOFTWARE_FLOWCONTROL
            uart->set_CTS_pin(true);
#endif
            {
                size_t i = 0;
                size_t bytes_written;
                while (i < buffer.len) {
                    // write the data to the UART
                    bytes_written = uart->write(&buffer.data[i], buffer.len - i);
                    i += bytes_written;
                }
            }
            uart->flush();
            last_received_ms = AP_HAL::millis();
            state = WaitingToFinshSend;
#ifdef IO_ASYNC_WAIT
            start_wait_us = AP_HAL::micros();
#endif
        FALLTHROUGH;

        case WaitingToFinshSend:
#ifdef SOFTWARE_FLOWCONTROL
            if (!wait_us(calc_transmit_time_us(buffer.len))) {
                return false;
            }
            uart->set_CTS_pin(false);
#endif
            state = WaitingForResponse;
        FALLTHROUGH;

        case WaitingForResponse: 
        {
            uint8_t b;
            if (uart->read(&b, 1) == 1) {
                last_received_ms = AP_HAL::millis();
                if (parse_response(b)) {
                    state = Finished;
                }
            } else if (AP_HAL::millis() - last_received_ms > 2000) {
                // timeout waiting for response
                state = Timeout;
            }

            // if (AP_HAL::millis() - last_received_ms > 2000) {
            //     // timeout waiting for response
            //     state = Timeout;
            // }
        } break;

        case Finished:
            // transaction is finished
            break;
        case Timeout:
            // transaction timed out
            break;
    }

    return is_finished();
    
}

bool AP_ModbusTransaction::is_finished() const
{
    return state == Finished || state == Timeout;
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
 : AP_ModbusTransaction(uart_serial),
   _reg_addr(reg_addr),
   _reg_value(reg_value),
   _parse_state(ParseState::Init)
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

bool WriteRegisterTransaction::parse_response(uint8_t byte)
{
#ifdef DEBUG
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: w parse_response %02X %d", byte, (uint8_t)_parse_state);
#endif // DEBUG

    switch (_parse_state) {
        case ParseState::Init:
            // check for start of message
            if (byte == 0x01) {
                buffer.len = 0;
                buffer.data[buffer.len++] = byte;
                _parse_state = ParseState::Shift;
            }
            break;

        case ParseState::Shift:
            buffer.data[buffer.len++] = byte;
            // check for end of message
            if (buffer.len < 8) {
                return false;
            }
            _parse_state = ParseState::Finished;
            FALLTHROUGH;
            
        case ParseState::Finished:
            return parse_msg(buffer.data, buffer.len);
    }

    return false;
}

bool WriteRegisterTransaction::parse_msg(uint8_t *rcvd_buff, uint8_t buff_len)
{
#ifdef DEBUG
    debug_print_buf(rcvd_buff, buff_len, "<=");
#endif // DEBUG

    if (buff_len != 8) {
        return false;
    }

    if (rcvd_buff[0] != 0x01) {
        return false;
    }
    if (rcvd_buff[1] != 0x06) {
        return false;
    }

    uint16_t reg_addr = be16toh_ptr(&rcvd_buff[2]);
    uint16_t reg_value = be16toh_ptr(&rcvd_buff[4]);

#ifdef DEBUG
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: reg_addr %04X reg_value %04X", reg_addr, reg_value);
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: tx reg_addr %04X tx reg_value %04X", _reg_addr, _reg_value);
#endif // DEBUG

    if (_reg_addr != reg_addr) {
        return false;
    }

    if (_reg_value != reg_value) {
        return false;
    }

    return eval_crc(rcvd_buff, buff_len);
}

ReadRegisterTransaction::ReadRegisterTransaction(AP_HAL::UARTDriver *uart_serial, uint16_t reg_addr)
 : AP_ModbusTransaction(uart_serial),
    _reg_value(0),
    _parse_state(ParseState::Init)
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

bool ReadRegisterTransaction::parse_response(uint8_t byte)
{
#ifdef DEBUG
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: r parse_response %02X %d", byte, (uint8_t)_parse_state);
#endif // DEBUG

    switch (_parse_state) {
        case ParseState::Init:
            // check for start of message
            if (byte == 0x01) {
                buffer.len = 0;
                buffer.data[buffer.len++] = byte;
                _parse_state = ParseState::Shift;
            }
            return false;

        case ParseState::Shift:
            buffer.data[buffer.len++] = byte;
            // check for end of message
            if (buffer.len < 7) {    
                return false;
            }
            _parse_state = ParseState::Finished;
            FALLTHROUGH;

        case ParseState::Finished:
            return parse_msg(buffer.data, buffer.len);
    }

    return false;
}

bool ReadRegisterTransaction::parse_msg(uint8_t *rcvd_buff, uint8_t buff_len)
{
#ifdef DEBUG
    debug_print_buf(rcvd_buff, buff_len, "<=");
#endif // DEBUG

    if (buff_len != 7) {
        return false;
    }

    if (rcvd_buff[0] != 0x01) {
        return false;
    }
    if (rcvd_buff[1] != 0x03) {
        return false;
    }

    _reg_value = be16toh_ptr(&rcvd_buff[3]);

    return eval_crc(rcvd_buff, buff_len);
}

WriteMotorCmdStreamTransaction::WriteMotorCmdStreamTransaction(AP_HAL::UARTDriver *uart_serial, uint8_t sub_code, uint32_t data)
 : AP_ModbusTransaction(uart_serial),
   _parse_state(ParseState::Init)
{
    // prepare the write buffer
    buffer.len = 0;
    buffer.data[buffer.len++] = 0x01; // device address
    buffer.data[buffer.len++] = 0x64; // function code
    buffer.data[buffer.len++] = sub_code; // sub code
    put_be32_ptr(&buffer.data[buffer.len], data); // data
    buffer.len += 4;
    
    add_crc(buffer);
}

bool WriteMotorCmdStreamTransaction::parse_response(uint8_t byte)
{
#ifdef DEBUG
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: m parse_response %02X %d", byte, (uint8_t)_parse_state);
#endif // DEBUG

    switch (_parse_state) {
        case ParseState::Init:
            // check for start of message
            if (byte == 0x01) {
                buffer.len = 0;
                buffer.data[buffer.len++] = byte;
                _parse_state = ParseState::Shift;
            }
            return false;

        case ParseState::Shift:
            buffer.data[buffer.len++] = byte;
            // check for end of message
            if (buffer.len < 19) {    
                return false;
            }
            _parse_state = ParseState::Finished;
            FALLTHROUGH;

        case ParseState::Finished:
            return parse_msg(buffer.data, buffer.len);
    }

    return false;
}

bool WriteMotorCmdStreamTransaction::parse_msg(uint8_t *rcvd_buff, uint8_t buff_len)
{
#ifdef DEBUG
    debug_print_buf(rcvd_buff, buff_len, "<=");
#endif // DEBUG

    if (buff_len != 19) {
        return false;
    }

    if (rcvd_buff[0] != 0x01) {
        return false;
    }
    if (rcvd_buff[1] != 0x64) {
        return false;
    }

    _actuator_state.shaft_position = be32toh_ptr(&rcvd_buff[3]);
    _actuator_state.force_realized = be32toh_ptr(&rcvd_buff[7]);
    _actuator_state.power_consumed = be16toh_ptr(&rcvd_buff[11]);
    _actuator_state.temperature = rcvd_buff[12];
    _actuator_state.voltage = be16toh_ptr(&rcvd_buff[13]);
    _actuator_state.errors = be16toh_ptr(&rcvd_buff[15]);
    
    return eval_crc(rcvd_buff, buff_len);
}

ReadMotorStreamTransaction::ReadMotorStreamTransaction(AP_HAL::UARTDriver *uart_serial, uint16_t regaddr, uint8_t reg_width)
 : AP_ModbusTransaction(uart_serial),
   _parse_state(ParseState::Init)
{
    // prepare the write buffer
    buffer.len = 0;
    buffer.data[buffer.len++] = 0x01; // device address
    buffer.data[buffer.len++] = 0x68; // function code
    put_be16_ptr(&buffer.data[buffer.len], regaddr); // register address
    buffer.len += 2;
    buffer.data[buffer.len++] = reg_width; // register width

    add_crc(buffer);
}

bool ReadMotorStreamTransaction::parse_response(uint8_t byte)
{
#ifdef DEBUG
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: r parse_response %02X %d", byte, (uint8_t)_parse_state);
#endif // DEBUG

    switch (_parse_state) {
        case ParseState::Init:
            // check for start of message
            if (byte == 0x01) {
                buffer.len = 0;
                buffer.data[buffer.len++] = byte;
                _parse_state = ParseState::Shift;
            }
            return false;

        case ParseState::Shift:
            buffer.data[buffer.len++] = byte;
            // check for end of message
            if (buffer.len < 24) {    
                return false;
            }
            _parse_state = ParseState::Finished;
            FALLTHROUGH;

        case ParseState::Finished:
            return parse_msg(buffer.data, buffer.len);
    }

    return false;
}

bool ReadMotorStreamTransaction::parse_msg(uint8_t *rcvd_buff, uint8_t buff_len)
{
#ifdef DEBUG
    debug_print_buf(rcvd_buff, buff_len, "<=");
#endif // DEBUG

    if (buff_len != 24) {
        return false;
    }

    if (rcvd_buff[0] != 0x01) {
        return false;
    }
    if (rcvd_buff[1] != 0x68) {
        return false;
    }

    _reg_value = be32toh_ptr(&rcvd_buff[2]);
    _operating_mode = static_cast<orca::OperatingMode>(rcvd_buff[6]);
    _actuator_state.shaft_position = be32toh_ptr(&rcvd_buff[7]);
    _actuator_state.force_realized = be32toh_ptr(&rcvd_buff[11]);
    _actuator_state.power_consumed = be16toh_ptr(&rcvd_buff[15]);
    _actuator_state.temperature = rcvd_buff[17];
    _actuator_state.voltage = be16toh_ptr(&rcvd_buff[18]);
    _actuator_state.errors = be16toh_ptr(&rcvd_buff[20]);
    
    return eval_crc(rcvd_buff, buff_len);
}

