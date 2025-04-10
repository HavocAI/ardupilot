#ifndef ORCA_MODBUS_H
#define ORCA_MODBUS_H

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

#define IRISORCA_MESSAGE_LEN_MAX    35  // messages are no more than 35 bytes

#define MODBUS_MSG_RECV_PENDING 0
#define MODBUS_MSG_RECV_READY 1
#define MODBUS_MSG_RECV_TIMEOUT 2

class OrcaModbus
{
    public:

        friend class AP_IrisOrca;
        
        OrcaModbus();
        CLASS_NO_COPY(OrcaModbus);

        void init(AP_HAL::UARTDriver *, AP_Int8 pin_de);
        void send_read_register_cmd(uint16_t reg_addr);
        void send_write_register_cmd(uint16_t reg_addr, uint16_t reg_value);
        void send_write_multiple_registers(uint16_t reg_addr, uint16_t reg_count, uint16_t *registers);
        
        /**
         * adds CRC-16 to the end of the message and then transmits
         */
        void send_data(uint8_t *data, uint16_t len, uint16_t expected_reply_len);
        

        /**
         * @breif read a 16-bit modbus register from the receive buf
         * Note: the `send_read_register_cmd()` should have been called before this
         * and the `message received()` should be true
         */
        uint16_t read_register();

        uint8_t message_received();

        void set_recive_timeout_ms(uint32_t timeout_ms);

        /**
         * @brief perform various tasks such as reading from the serial port
         * and checking for timeouts
         * @note this should be continuously called from the main loop
         */
        void tick();

        
    
    private:
        AP_HAL::UARTDriver *_uart;          // serial port to communicate with actuator
        AP_Int8 _pin_de;
        uint16_t _reply_msg_len;
        uint8_t _received_buff[IRISORCA_MESSAGE_LEN_MAX];
        uint16_t _received_buff_len;
        uint32_t _send_start_us;            // system time (in micros) when last message started being sent (used for timing to unset DE pin)
        uint32_t _send_delay_us;            // delay (in micros) to allow bytes to be sent after which pin can be unset.  0 if not delaying
        uint32_t _reply_wait_start_ms;  // system time that we started waiting for a reply message
        bool _received_msg_ready;
        bool _is_sending;

        void check_send_end();

};

#endif // ORCA_MODBUS_H