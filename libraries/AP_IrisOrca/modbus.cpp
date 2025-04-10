
#include "modbus.h"
#include <AP_Math/crc.h>

#define IRISORCA_SERIAL_BAUD 19200                  // communication is always at 19200
#define IRISORCA_SERIAL_PARITY 2                    // communication is always even parity
#define IRISORCA_LOG_ORCA_INTERVAL_MS 5000          // log ORCA message at this interval in milliseconds
#define IRISORCA_SEND_ACTUATOR_CMD_INTERVAL_MS 100  // actuator commands sent at 10hz if connected to actuator
#define IRISORCA_REPLY_TIMEOUT_MS 25                // stop waiting for replies after 25ms
#define IRISORCA_ERROR_REPORT_INTERVAL_MAX_MS 10000 // errors reported to user at no less than once every 10 seconds

extern const AP_HAL::HAL &hal;

// message addresses
enum class MsgAddress : uint8_t
{
    BUS_MASTER = 0x00,
    DEVICE = 0x01
};

// function codes
enum class FunctionCode : uint8_t
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
static uint32_t calc_send_delay_us(uint8_t num_bytes)
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
      _send_delay_us(0),
      _reply_wait_start_ms(0)
{
    // Constructor implementation
}

void OrcaModbus::init(AP_HAL::UARTDriver *uart, AP_Int8 pin_de)
{
    _uart = uart;
    _pin_de = pin_de;

    // Set the serial port parameters
    _uart->begin(IRISORCA_SERIAL_BAUD);
    _uart->configure_parity(IRISORCA_SERIAL_PARITY);
    _uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    _uart->set_unbuffered_writes(true);

    // initialise RS485 DE pin (when high, allows send to actuator)
    if (_pin_de > -1)
    {
        hal.gpio->pinMode(_pin_de, HAL_GPIO_OUTPUT);
        hal.gpio->write(_pin_de, 0);
    }
    else
    {
        _uart->set_CTS_pin(false);
    }
}

void OrcaModbus::send_read_register_cmd(uint16_t reg_addr)
{
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
    add_crc_modbus(send_buff, i);

    send_data(send_buff, i + 2);
}

void OrcaModbus::send_data(uint8_t *data, uint16_t len)
{
    // set send pin
    // set gpio pin or serial port's CTS pin
    if (_pin_de > -1) {
        hal.gpio->write(_pin_de, 1);
    } else {
        _uart->set_CTS_pin(true);
    }

    // record start and expected delay to send message
    _send_start_us = AP_HAL::micros();
    _send_delay_us = calc_send_delay_us(len);

    _reply_wait_start_ms = AP_HAL::millis();

    // write message
    _uart->write(data, len);
}
