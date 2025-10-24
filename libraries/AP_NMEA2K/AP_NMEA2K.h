#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_NMEA2K_ENABLED
#define HAL_NMEA2K_ENABLED (HAL_MAX_CAN_PROTOCOL_DRIVERS && BOARD_FLASH_SIZE > 1024)
#endif

#if HAL_NMEA2K_ENABLED

#include <cstdint>
#include <AP_Param/AP_Param.h>
#include <AP_CANManager/AP_CANSensor.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_NMEA2K/AP_NMEA2K_msg.h>
class AP_NMEA2K :
    public CANSensor
{
public:

    FUNCTOR_TYPEDEF(NMEA2K_HandleN2KMessage_Functor,
                     void,
                     AP_NMEA2K*,
                     nmea2k::N2KMessage&);
    
    FUNCTOR_TYPEDEF(NMEA2K_HandleFrame_Functor,
                     void,
                     AP_NMEA2K*,
                     AP_HAL::CANFrame&);


    AP_NMEA2K();
    CLASS_NO_COPY(AP_NMEA2K);

    static const struct AP_Param::GroupInfo var_info[];

    // called from the main loop
    static void update(void);

    void handle_frame(AP_HAL::CANFrame &frame) override;

    void register_handle_n2k_message(NMEA2K_HandleN2KMessage_Functor handle_n2k_message);

private:

    static constexpr size_t kMaxStoredFastPackets = 4;

    class BufferedFastPacket {
    public:

        BufferedFastPacket() 
        : id(0)
        {}

        void reset(uint32_t new_id, size_t expected_len)
        {
            this->id = new_id;
            this->expected_data_len = expected_len;
            this->msg.data_length_ = 0;
        }

        void clear()
        {
            id = 0;
        }

        uint32_t id;
        size_t expected_data_len;
        nmea2k::N2KMessage msg;
        uint32_t last_update_ms;

    };

    BufferedFastPacket _rx_msg[kMaxStoredFastPackets];

    NMEA2K_HandleN2KMessage_Functor _n2k_message_handlers[4];
    size_t _num_n2k_message_handlers = 0;

    size_t find_free_slot(uint32_t now_ms);
    size_t find_buffered_slot(const uint32_t id);
    void handle_message(nmea2k::N2KMessage& msg);
    
};

#endif // HAL_NMEA2K_ENABLED