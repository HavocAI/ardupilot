#ifndef ORCA_MODBUS_H
#define ORCA_MODBUS_H

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

#define IRISORCA_MESSAGE_LEN_MAX    35  // messages are no more than 35 bytes

class OrcaModbus
{
    public:

        friend class AP_IrisOrca;
        
        OrcaModbus();
        CLASS_NO_COPY(OrcaModbus);

        void init(AP_HAL::UARTDriver *, AP_Int8 pin_de);
        void send_read_register_cmd(uint16_t reg_addr);
        bool message_received();

        void read();
    
    private:
        AP_HAL::UARTDriver *_uart;          // serial port to communicate with actuator
        AP_Int8 _pin_de;
        uint16_t _reply_msg_len;
        uint8_t _received_buff[IRISORCA_MESSAGE_LEN_MAX];
        uint16_t _received_buff_len;
        uint32_t _send_start_us;            // system time (in micros) when last message started being sent (used for timing to unset DE pin)
        uint32_t _send_delay_us;            // delay (in micros) to allow bytes to be sent after which pin can be unset.  0 if not delaying
        uint32_t _reply_wait_start_ms;  // system time that we started waiting for a reply message

        void send_data(uint8_t *data, uint16_t len);

};

#endif // ORCA_MODBUS_H