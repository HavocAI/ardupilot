#include <AP_NMEA2K/AP_NMEA2K.h>

#include <AP_CANManager/AP_CANManager.h>
#include <AP_J1939_CAN/AP_J1939_CAN.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>

#include <AP_NMEA2K/AP_NMEA2K_msg.h>
#include <AP_NMEA2K/AP_NMEA2K_pgn_utils.hpp>

#include "can-msg-definitions/n2k.h"

#include <AP_Math/AP_Math.h>

#if HAL_NMEA2K_ENABLED

const AP_Param::GroupInfo AP_NMEA2K::var_info[] = {

    AP_GROUPEND
};



static void send_pgn_127488(AP_NMEA2K* driver)
{
    float rpm = 0.0f;
    // get the current engine RPM from the vehicle
    AP_ESC_Telem* esc_telem = AP_ESC_Telem::get_singleton();
    if (esc_telem == nullptr) {
        return;
    }
    if (esc_telem->get_rpm(0, rpm)) {

        nmea2k::N2KMessage msg;
        msg.SetPGN(127488);

        const n2k_pgn_127488_engine_parameters_rapid_update_t data = {
            .instance = 0,
            .speed = static_cast<uint16_t>(rpm),
        };

        n2k_pgn_127488_engine_parameters_rapid_update_pack(
            msg.DataPtrForPack(),
            &data,
            nmea2k::N2KMessage::MAX_DATA_SIZE
        );

        // msg.AddByte(0);
        // msg.Add2ByteUInt(rpm);
        // msg.Add2ByteInt(0);
        // msg.AddByte(0);

        AP_HAL::CANFrame frame;
        frame.id = msg.FormatToCanId() | AP_HAL::CANFrame::FlagEFF;
        frame.dlc = msg.data_length();
        msg.CopyDataToBuffer(frame.data, sizeof(frame.data), msg.data_length());
        driver->write_frame(frame, 10);

    }

}




AP_NMEA2K::AP_NMEA2K() :
    CANSensor("NMEA2K", 2048)
{
    AP_Param::setup_object_defaults(this, var_info);
}

size_t AP_NMEA2K::find_free_slot(uint32_t now_ms)
{
    for (size_t i=0; i<kMaxStoredFastPackets; i++) {
        if (_rx_msg[i].id == 0 || (now_ms - _rx_msg[i].last_update_ms) > 1000) {
            return i;
        }
    }
    return kMaxStoredFastPackets;
}

size_t AP_NMEA2K::find_buffered_slot(const uint32_t id)
{
    for (size_t i=0; i<kMaxStoredFastPackets; i++) {
        if (_rx_msg[i].id == id) {
            return i;
        }
    }
    return kMaxStoredFastPackets;
}

void AP_NMEA2K::handle_frame(AP_HAL::CANFrame &frame)
{
    const uint32_t now_ms = AP_HAL::millis();

    nmea2k::N2KMessage msg;
    msg.SetInfoFromCanId(frame.id);
    const uint32_t pgn = msg.pgn();

    if (IsSingleFrameSystemMessage(pgn)) {
        memcpy(msg.data_, frame.data, frame.dlc);
        handle_message(msg);
    } else if (IsFastPacketDefaultMessage(pgn) || IsFastPacketSystemMessage(pgn)) {
        const uint8_t frame_counter = frame.data[0] & 0x1F;
        const uint8_t sequence_counter = (frame.data[0] >> 5) & 0x07;
        const uint32_t id = pgn << 8 | sequence_counter;
        BufferedFastPacket* bp = nullptr;
        uint8_t* src = nullptr;
        size_t len = 0;

        if (frame_counter == 0) {
            // find a buffer to use
            const size_t buffer_index = find_free_slot(now_ms);
            if (buffer_index == kMaxStoredFastPackets) {
                // no buffer available
                return;
            }

            const size_t expected_data_len = frame.data[1];
            

            bp = &_rx_msg[buffer_index];
            bp->id = id;
            bp->expected_data_len = expected_data_len;
            bp->msg.data_length_ = 0;
            bp->msg.SetInfoFromCanId(frame.id);

            src = &frame.data[2];
            len = MIN(expected_data_len, frame.dlc - 2);

        } else {
            const size_t buffer_index = find_buffered_slot(id);
            if (buffer_index == kMaxStoredFastPackets) {
                return;
            }
            bp = &_rx_msg[buffer_index];
            src = &frame.data[1];
            len = frame.dlc - 1;
        }

        len = MIN(len, nmea2k::N2KMessage::MAX_DATA_SIZE - bp->msg.data_length_);
        memcpy(&bp->msg.data_[bp->msg.data_length_], src, len);
        bp->msg.data_length_ += len;
        bp->last_update_ms = now_ms;

        if (bp->msg.data_length_ == bp->expected_data_len) {

            if (IsFastPacketSystemMessage(pgn)) {
                // TODO:
            } else {
                handle_message(bp->msg);
            }

            bp->clear();
        }
    }

}

void AP_NMEA2K::handle_message(nmea2k::N2KMessage& msg)
{
}

void AP_NMEA2K::update(void)
{
    AP_CANManager* can_mgr = AP_CANManager::get_singleton();
    if (can_mgr == nullptr) {
        return;
    }

    for (uint8_t can_port=0; can_port < can_mgr->get_num_drivers(); can_port++) {
        if (can_mgr->get_driver_type(can_port) == AP_CAN::Protocol::NMEA2K) {
            AP_NMEA2K* driver = static_cast<AP_NMEA2K*>(can_mgr->get_driver(can_port));
            if (driver == nullptr) {
                continue;
            }

            send_pgn_127488(driver);
        }
    }
}


#endif // HAL_NMEA2K_ENABLED