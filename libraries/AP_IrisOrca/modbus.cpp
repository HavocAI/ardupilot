
#include "modbus.h"
#include <AP_Math/crc.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <GCS_MAVLink/GCS.h>

// #define DEBUG 1

#ifdef DEBUG
#include <stdio.h>

static void debug_print_buf(uint8_t* buf, int len);
#endif // DEBUG

#define IRISORCA_SERIAL_BAUD 19200                  // communication is always at 19200
#define IRISORCA_SERIAL_PARITY 2                    // communication is always even parity
#define IRISORCA_LOG_ORCA_INTERVAL_MS 5000          // log ORCA message at this interval in milliseconds
#define IRISORCA_SEND_ACTUATOR_CMD_INTERVAL_MS 100  // actuator commands sent at 10hz if connected to actuator
#define IRISORCA_REPLY_TIMEOUT_MS 25                // stop waiting for replies after 25ms
#define IRISORCA_ERROR_REPORT_INTERVAL_MAX_MS 10000 // errors reported to user at no less than once every 10 seconds

extern const AP_HAL::HAL &hal;

// message addresses
enum MsgAddress : uint8_t
{
    BUS_MASTER = 0x00,
    DEVICE = 0x01
};

// function codes
enum FunctionCode : uint8_t
{
    READ_REGISTER = 0x03,
    WRITE_REGISTER = 0x06,
    WRITE_MULTIPLE_REGISTERS = 0x10,
    MOTOR_COMMAND_STREAM = 0x64,
    MOTOR_READ_STREAM = 0x68
};

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

// calculate delay require to allow bytes to be sent
static uint32_t calc_tx_time_us(uint8_t num_bytes)
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

OrcaModbus::OrcaModbus()
    : _uart(nullptr),
      _reply_msg_len(0),
      _received_buff_len(0),
      _send_start_us(0),
      _transmit_time_us(0),
      _reply_wait_start_ms(0),
      _sending_state(SendingState::Idle),
      _receive_state(ReceiveState::Pending)
{
    // Constructor implementation
}

void OrcaModbus::init(AP_HAL::UARTDriver *uart, int pin_de)
{
    _uart = uart;
    _pin_de = pin_de;

    _uart->lock_port(40, 40);

    // Set the serial port parameters
    _uart->configure_parity(IRISORCA_SERIAL_PARITY);
    _uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    _uart->set_unbuffered_writes(true);
    _uart->begin(IRISORCA_SERIAL_BAUD, 128, 128);
    _uart->discard_input();

    

    // initialise RS485 DE pin (when high, allows send to actuator)
    // if (_pin_de >= 0) {
    //     hal.gpio->pinMode(_pin_de, HAL_GPIO_OUTPUT);
    //     hal.gpio->write(_pin_de, 0);
    // } else {
    //     _uart->set_CTS_pin(false);
    // }

    _uart->set_CTS_pin(false);
    // _uart->set_RTS_pin(true);
}

void OrcaModbus::tick()
{
    // uint8_t b;
    uint32_t now_us = AP_HAL::micros();

    if (_sending_state == SendingState::Sending) {
        if (now_us - _send_start_us > _transmit_time_us) {
            // unset gpio or serial port's CTS pin
            // if (_pin_de > -1) {
            //     hal.gpio->write(_pin_de, 0);
            // } else {
            //     _uart->set_CTS_pin(false);
            //     GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: set CTS pin low");
            // }

            _uart->set_CTS_pin(false);
            // _uart->set_RTS_pin(true);

            _sending_state = SendingState::Idle;
        }
    }

    if (_receive_state == ReceiveState::Pending) {
        if (AP_HAL::millis() - _reply_wait_start_ms > IRISORCA_REPLY_TIMEOUT_MS) {
            // timeout waiting for reply
            _reply_wait_start_ms = 0;
            _receive_state = ReceiveState::Timeout;
        }
    }

    if(_uart->available_locked(40)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: available bytes: %lu", _uart->available_locked(40));
    }
    // while (_uart->read(b)) {
    //     GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: received byte: %u", b);
    //     if (_received_buff_len < IRISORCA_MESSAGE_LEN_MAX) {
    //         _received_buff[_received_buff_len++] = b;
    //         if (_received_buff_len >= _reply_msg_len) {
    //             // check CRC of the message
    //             uint16_t crc_expected = calc_crc_modbus(_received_buff, _received_buff_len - 2);
    //             uint16_t crc_received = (_received_buff[_received_buff_len - 2]) | (_received_buff[_received_buff_len - 1] << 8);
    //             if (crc_expected == crc_received) {
    //                 _receive_state = ReceiveState::Ready;
    //                 _reply_wait_start_ms = 0;
    //             } else {
    //                 // CRC is incorrect
    //                 _receive_state = ReceiveState::CRCError;
    //             }
    //             _received_buff_len = 0;
    //         }
    //     }
    // }
}

void OrcaModbus::set_recive_timeout_ms(uint32_t timeout_ms)
{

}

void OrcaModbus::send_read_register_cmd(uint16_t reg_addr)
{
    // buffer for outgoing message
    uint8_t send_buff[16];

    // build message
    uint16_t i = 0;
    send_buff[i++] = MsgAddress::DEVICE;
    send_buff[i++] = FunctionCode::READ_REGISTER;

    put_be16_ptr(&send_buff[i], reg_addr);
    i += 2;

    put_be16_ptr(&send_buff[i], 1); // number of registers to read
    i += 2;
    
    send_data(send_buff, i, 7);
}

bool OrcaModbus::read_register(uint16_t& reg)
{
    if (_receive_state == ReceiveState::Ready) {

        if (_received_buff[1] != FunctionCode::READ_REGISTER) {
            return false;
        }

        reg = be16toh_ptr(&_received_buff[3]);
        return true;
    }
    return false;
}

void OrcaModbus::send_write_register_cmd(uint16_t reg_addr, uint16_t reg_value)
{
    // buffer for outgoing message
    uint8_t send_buff[16];

    // build message
    uint16_t i = 0;
    send_buff[i++] = MsgAddress::DEVICE;
    send_buff[i++] = FunctionCode::WRITE_REGISTER;
    send_buff[i++] = HIGHBYTE(reg_addr);
    send_buff[i++] = LOWBYTE(reg_addr);
    send_buff[i++] = HIGHBYTE(reg_value);
    send_buff[i++] = LOWBYTE(reg_value);

    send_data(send_buff, i, 8);
}

void OrcaModbus::send_write_multiple_registers(uint16_t reg_addr, uint16_t reg_count, uint16_t *registers)
{
    // buffer for outgoing message
    uint8_t* send_buff = static_cast<uint8_t*>(alloca(7 + 2 + (2 * reg_count)));

    // build message
    uint16_t i = 0;
    send_buff[i++] = MsgAddress::DEVICE;
    send_buff[i++] = FunctionCode::WRITE_MULTIPLE_REGISTERS;
    send_buff[i++] = HIGHBYTE(reg_addr);
    send_buff[i++] = LOWBYTE(reg_addr);
    send_buff[i++] = HIGHBYTE(reg_count);
    send_buff[i++] = LOWBYTE(reg_count);
    send_buff[i++] = LOWBYTE(reg_count * 2);

    // copy data into message
    for (uint16_t j = 0; j < reg_count; j++)
    {
        send_buff[i++] = HIGHBYTE(registers[j]);
        send_buff[i++] = LOWBYTE(registers[j]);
    }

    send_data(send_buff, i, 8);
}

#ifdef DEBUG
static void debug_print_buf(uint8_t* buf, int len)
{
    char str[256];
    for (int i = 0; i < len; i++) {
        snprintf(str + (i * 3), sizeof(str) - (i * 3), "%02X ", buf[i]);
    }
    str[len * 3] = '\0'; // null terminate the string

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: %s", str);
}
#endif

void OrcaModbus::send_data(uint8_t *data, uint16_t len, uint16_t expected_reply_len)
{

    // Add Modbus CRC-16
    add_crc_modbus(data, len);
    len += 2; // add CRC length

#ifdef DEBUG
    debug_print_buf(data, len);
#endif // DEBUG


    // set send pin
    // set gpio pin or serial port's CTS pin
    // if (_pin_de > -1) {
    //     hal.gpio->write(_pin_de, 1);
    // } else {
    //     _uart->set_CTS_pin(true);
    // }

    _uart->set_CTS_pin(true);
    

    // record start and expected delay to send message
    _send_start_us = AP_HAL::micros();
    _transmit_time_us = calc_tx_time_us(len);
    _receive_state = ReceiveState::Pending;
    _sending_state = SendingState::Sending;

    _reply_wait_start_ms = AP_HAL::millis();
    _reply_msg_len = expected_reply_len;

    // write message
    // _uart->write_locked(data, len, 5);
    _uart->write_locked(data, len, 40);
    // _uart->write(data, len);
}

