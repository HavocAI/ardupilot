#include <AP_NMEA2K/AP_NMEA2K.h>

#include <AP_CANManager/AP_CANManager.h>
#include <AP_J1939_CAN/AP_J1939_CAN.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>

#include <AP_NMEA2K/AP_NMEA2K_msg.h>
#include <AP_NMEA2K/AP_NMEA2K_pgn_utils.hpp>

#include "can-msg-definitions/n2k.h"

#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

#if HAL_NMEA2K_ENABLED

#define NMEA2K_DEBUG 1
#define NMEA2K_EMU_MESSAGES 0

const AP_Param::GroupInfo AP_NMEA2K::var_info[] = {

    AP_GROUPINFO("NM_B", 1, AP_NMEA2K, _param, 0),

    AP_GROUPEND
};


#if 0
static void send_pgn_127488(AP_NMEA2K* driver)
{

#if NMEA2K_DEBUG
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NMEA2K: tx PGN 127488");
#endif // NMEA2K_DEBUG


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
#endif



AP_NMEA2K::AP_NMEA2K() :
    CANSensor("NMEA2K", 2048)
{
    AP_Param::setup_object_defaults(this, var_info);
#if NMEA2K_DEBUG
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NMEA2K: Driver Created");
#endif // NMEA2K_DEBUG

    _num_n2k_message_handlers = 0;

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
#if NMEA2K_DEBUG
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NMEA2K: rx ID: 0x%" PRIx32, frame.id);
#endif // NMEA2K_DEBUG

    const uint32_t now_ms = AP_HAL::millis();

    nmea2k::N2KMessage msg;
    msg.SetInfoFromCanId(frame.id);
    const uint32_t pgn = msg.pgn();

    if (IsSingleFrameSystemMessage(pgn)) {
        // TODO: handle system message
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "NMEA2K: unhandled sys pgn: %" PRIu32, pgn);
        
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
            len = MIN(bp->expected_data_len, frame.dlc - 1);
        }

        len = MIN(len, nmea2k::N2KMessage::MAX_DATA_SIZE - bp->msg.data_length_);
        memcpy(&bp->msg.data_[bp->msg.data_length_], src, len);
        bp->msg.data_length_ += len;
        bp->last_update_ms = now_ms;

        if (bp->msg.data_length_ == bp->expected_data_len) {

            if (IsFastPacketSystemMessage(pgn)) {
                // TODO:
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "NMEA2K: unhandled sys pgn: %" PRIu32, pgn);
                   
            } else {
                handle_message(bp->msg);
            }

            bp->clear();
        }
    } else {
        // single frame message
        msg.SetDataFromPack(frame.data, frame.dlc);
        handle_message(msg);
    }

}

void AP_NMEA2K::register_handle_n2k_message(NMEA2K_HandleN2KMessage_Functor handle_n2k_message)
{
    if (_num_n2k_message_handlers < ARRAY_SIZE(_n2k_message_handlers)) {
        _n2k_message_handlers[_num_n2k_message_handlers++] = handle_n2k_message;
    }
}

void AP_NMEA2K::handle_message(nmea2k::N2KMessage& msg)
{
    for (size_t i=0; i<_num_n2k_message_handlers; i++) {
        _n2k_message_handlers[i](this, msg);
    }
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

            // send_pgn_127488(driver);

#if NMEA2K_EMU_MESSAGES
            static uint32_t last_msg_time_ms = 0;
            const uint32_t now_ms = AP_HAL::millis();
            if ((now_ms - last_msg_time_ms) >= 1000) {
                last_msg_time_ms = now_ms;

                // emulate a PGN 127250 Vessel Heading message
                nmea2k::N2KMessage msg;
                msg.SetPGN(127250);

                const float heading_deg = fmodf(static_cast<float>(now_ms) * 0.001f, 360.0f);

                const n2k_pgn_127250_vessel_heading_t data = {
                    .sid = 0,
                    .heading = static_cast<uint16_t>(heading_deg * 10000.0f / RAD_TO_DEG),
                };

                n2k_pgn_127250_vessel_heading_pack(
                    msg.DataPtrForPack(),
                    &data,
                    nmea2k::N2KMessage::MAX_DATA_SIZE
                );
                msg.SetDataLength(8);

                driver->handle_message(msg);

                msg.SetPGN(129029);
                msg.SetDataLength(0);
                msg.AddByte(0); // SID

                // today's fake day is 1761582411
                const uint64_t unix_time = 1761582411ULL;
                
                msg.Add2ByteUInt(unix_time / 86400);
                msg.Add4ByteUInt((unix_time % 86400) * 10000);

                msg.Add8ByteInt(static_cast<int64_t>(37.7749 * 1e16));  // lat
                msg.Add8ByteInt(static_cast<int64_t>(-122.4194 * 1e16)); // lon

                msg.Add8ByteInt(static_cast<int64_t>(15.24 * 1e6));

                driver->handle_message(msg);

            }
#endif // NMEA2K_EMU_MESSAGES
        }
    }


}


#endif // HAL_NMEA2K_ENABLED