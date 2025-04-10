#ifndef ORCA_MODBUS_H
#define ORCA_MODBUS_H

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#define IRISORCA_MESSAGE_LEN_MAX    35  // messages are no more than 35 bytes

class OrcaModbus
{
    public:

        friend class AP_IrisOrca;
        
        OrcaModbus();
        CLASS_NO_COPY(OrcaModbus);

        void send_read_register_cmd(uint16_t reg_addr);
    
    private:
        AP_HAL::UARTDriver *_uart;          // serial port to communicate with actuator
        uint16_t _reply_msg_len;
        uint8_t _received_buff[IRISORCA_MESSAGE_LEN_MAX];
        uint16_t _received_buff_len;

        void start_send();

};

#endif // ORCA_MODBUS_H