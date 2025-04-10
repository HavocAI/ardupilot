
#include "modbus.h"

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