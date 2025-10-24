#include "AP_GPS_NMEA2K.h"

#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS.h>

#include <AP_NMEA2K/can-msg-definitions/n2k.h>

#define AP_GPS_NMEA2K_DEBUG 0

AP_GPS_NMEA2K::AP_GPS_NMEA2K(AP_GPS &_gps, AP_GPS::Params &_params, AP_GPS::GPS_State &_state) :
    AP_GPS_Backend(_gps, _params, _state, nullptr)
{
    _last_msg_time_ms = 0;
    _new_data = false;
    state.status = AP_GPS::GPS_Status::NO_GPS;
}

AP_GPS_NMEA2K* AP_GPS_NMEA2K::probe(AP_GPS &_gps, AP_GPS::Params &_params, AP_GPS::GPS_State &_state)
{
    // find a NMEA2K CAN driver instance from the CANManager
    AP_CANManager* can_manager = AP_CANManager::get_singleton();
    AP_NMEA2K* nmea2k = nullptr;
    for (size_t i = 0; i < can_manager->get_num_drivers(); i++) {
        if (can_manager->get_driver_type(i) == AP_CAN::Protocol::NMEA2K) {
            nmea2k = static_cast<AP_NMEA2K*>(can_manager->get_driver(i));
            break;
        }
    }

    // create GPS backend
    AP_GPS_NMEA2K* backend = NEW_NOTHROW AP_GPS_NMEA2K(_gps, _params, _state);
    if (backend == nullptr) {
        return nullptr;
    }


    // register NMEA2K message handler
    AP_NMEA2K::NMEA2K_HandleN2KMessage_Functor handle_n2k_message = FUNCTOR_BIND(backend, &AP_GPS_NMEA2K::handle_nmea2k_message, void, AP_NMEA2K*, nmea2k::N2KMessage&);
    nmea2k->register_handle_n2k_message(handle_n2k_message);

    return backend;
}

void AP_GPS_NMEA2K::handle_nmea2k_message(AP_NMEA2K* nmea2k_instance, nmea2k::N2KMessage& msg)
{

    const uint32_t now_ms = AP_HAL::millis();
    WITH_SEMAPHORE(sem);

#ifdef AP_GPS_NMEA2K_DEBUG
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NMEA2K PGN: %lu", msg.pgn());
#endif // AP_GPS_NMEA2K_DEBUG

    switch (msg.pgn()) {
        case 129025: // Position, Rapid Update
        {
            n2k_pgn_129025_position_rapid_update_t data;
            if (n2k_pgn_129025_position_rapid_update_unpack(&data, msg.DataPtrForUnpack(), msg.data_length()) != 0) {
                return;
            }

            _interim_state.location.lat = data.latitude;  // in 1e-7 degrees
            _interim_state.location.lng = data.longitude; // in 1e-7 degrees

            _interim_state.status = AP_GPS::GPS_Status::GPS_OK_FIX_2D;

            _last_msg_time_ms = now_ms;
            _new_data = true;
            break;
        }

        case 129029:
        {
            const uint8_t* data = msg.DataPtrForUnpack();
            size_t i = 0;

            // const uint8_t sequence_id = data[i];
            i += 1;

            // day comes in as days since Jan 1, 1970
            const uint16_t day = nmea2k::N2KMessage::ReadUInt16(&data[i]);
            i += 2;

            // time comes in as 1e4 seconds since midnight
            const uint32_t time = nmea2k::N2KMessage::ReadUInt32(&data[i]);
            i += 4;

            const uint64_t epoch_ms = static_cast<uint64_t>(day) * 86400000ULL + static_cast<uint64_t>(time / 10);
            const uint64_t gps_ms = epoch_ms - UNIX_OFFSET_MSEC;

            // GPS time started at midnight Jan 6, 1980
            // GPS week number is the number of weeks since the start of GPS time.
            _interim_state.time_week = (uint16_t)(gps_ms / AP_MSEC_PER_WEEK);
            _interim_state.time_week_ms = (uint32_t)(gps_ms - (_interim_state.time_week * AP_MSEC_PER_WEEK));

            // lat/lng comes in at 1e16 degrees. Convert to 1e7 degrees.
            _interim_state.location.lat = static_cast<int32_t>(nmea2k::N2KMessage::ReadInt64(&data[i]) / 1000000000);
            i += 8;

            _interim_state.location.lng = static_cast<int32_t>(nmea2k::N2KMessage::ReadInt64(&data[i]) / 1000000000);
            i += 8;

            // altitude comes in 1e6 meters. Convert to cm.
            _interim_state.location.set_alt_cm(static_cast<int32_t>(nmea2k::N2KMessage::ReadInt64(&data[i]) / 10000), Location::AltFrame::ABSOLUTE);
            i += 8;

            // const uint8_t type_of_system = data[i] >> 4;
            // const uint8_t type_of_fix = data[i] & 0x0F;
            i += 1;

            // uint8_t integrity = data[i] >> 6;
            i += 1;

            _interim_state.status = AP_GPS::GPS_Status::GPS_OK_FIX_2D;

            _last_msg_time_ms = now_ms;
            _new_data = true;
            break;
        }

        case 127250: // Vessel Heading
        {
            n2k_pgn_127250_vessel_heading_t data;
            if (n2k_pgn_127250_vessel_heading_unpack(&data, msg.DataPtrForUnpack(), msg.data_length()) != 0) {
                return;
            }

            _interim_state.gps_yaw_configured = true;

            
            // heading comes in at 1e4 radians. Convert to degrees.
            _interim_state.gps_yaw = data.heading * 0.0001f * RAD_TO_DEG;
            _interim_state.have_gps_yaw = true;
            _interim_state.gps_yaw_time_ms = now_ms;

            _last_msg_time_ms = now_ms;
            _new_data = true;
            break;
        }
    }
}

bool AP_GPS_NMEA2K::read()
{
    WITH_SEMAPHORE(sem);
    const bool retval = _new_data;

    if (_new_data) {
        _new_data = false;
        state = _interim_state;
    }

    if (!is_healthy()) {
        state.status = AP_GPS::GPS_Status::NO_FIX;
    }

    return retval;
}

bool AP_GPS_NMEA2K::is_healthy(void) const
{
    // healthy if we have received a message within the last 2 seconds
    const uint32_t now_ms = AP_HAL::millis();
    return (now_ms - _last_msg_time_ms) < 2000;
}


