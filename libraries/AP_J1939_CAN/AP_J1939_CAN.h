#pragma once

#include "AP_J1939_CAN_config.h"

#if HAL_J1939_CAN_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANSensor.h>
#include <map>
#include <vector>
#include <queue>

class AP_J1939_CAN : public CANSensor
{
public:
    static AP_J1939_CAN* get_instance(uint8_t can_port);
    
    bool register_driver(uint32_t msg_id, CANSensor* driver);
    bool send_message(uint32_t msg_id, const uint8_t* data, uint8_t len);
    void handle_frame(AP_HAL::CANFrame &frame) override;
    
    bool enqueue_message(uint32_t msg_id, const uint8_t* data, uint8_t len);
    void process_queue();

private:
    explicit AP_J1939_CAN(uint8_t can_port);
    
    struct CANMessage {
        uint32_t msg_id;
        uint8_t data[8];
        uint8_t len;
    };

    static std::map<uint8_t, AP_J1939_CAN*> _instances;
    std::map<uint32_t, std::vector<CANSensor*>> _msg_handlers;
    std::queue<CANMessage> _message_queue;
    uint8_t _max_queue_size = 10;
    AP_HAL::Semaphore* _queue_semaphore;

    uint8_t _can_port;
    std::map<uint32_t, uint32_t> _last_sent_time; // Rate limiter storage
};

#endif // HAL_J1939_CAN_ENABLED
