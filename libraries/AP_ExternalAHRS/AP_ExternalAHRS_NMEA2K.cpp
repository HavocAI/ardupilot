#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_NMEA2K_ENABLED

#include <stdint.h>
#include "AP_ExternalAHRS_NMEA2K.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_NMEA2K/AP_NMEA2K.h>
#include <AP_NMEA2K/can-msg-definitions/n2k.h>

extern const AP_HAL::HAL &hal;

#define EXTERNAL_AHRS_DEBUG 0

AP_ExternalAHRS_NMEA2K::AP_ExternalAHRS_NMEA2K(AP_ExternalAHRS *_frontend, AP_ExternalAHRS::state_t &_state)
    : AP_ExternalAHRS_backend(_frontend, _state)
{
    initialized = false;
    nmea2k = nullptr;

    set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::GPS) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::COMPASS));
}

bool AP_ExternalAHRS_NMEA2K::init()
{
    WITH_SEMAPHORE(state.sem);
    if (initialized) {
        return true;
    }

    // find a NMEA2K CAN driver instance from the CANManager
    AP_CANManager* can_manager = AP_CANManager::get_singleton();
    for (size_t i = 0; i < can_manager->get_num_drivers(); i++) {
        if (can_manager->get_driver_type(i) == AP_CAN::Protocol::NMEA2K) {
            nmea2k = static_cast<AP_NMEA2K*>(can_manager->get_driver(i));

            // register NMEA2K message handler
            AP_NMEA2K::NMEA2K_HandleN2KMessage_Functor handle_n2k_message = FUNCTOR_BIND(this, &AP_ExternalAHRS_NMEA2K::handle_nmea2k_message, void, AP_NMEA2K*, nmea2k::N2KMessage&);
            nmea2k->register_handle_n2k_message(handle_n2k_message);
            initialized = true;
        }
    }

    return initialized;
}

void AP_ExternalAHRS_NMEA2K::update()
{
    init();
    const uint32_t now = AP_HAL::millis();
    if (nmea2k != nullptr && now - last_send_cmd > 1000) {

        const uint8_t send_gps = option_is_set(AP_ExternalAHRS::OPTIONS::AN_SEND_GPS_MSG);
        if (send_gps) {
            const uint8_t enable_gps = option_is_set(AP_ExternalAHRS::OPTIONS::AN_DISABLE_GNSS) ? 0 : 1;

            nmea2k::N2KMessage msg;
            msg.SetPGN(65281);
            msg.SetPriority(6);
            msg.ManualSetSource(0x23);
            
            std::memset(msg.DataPtrForPack(), 0xff, 8);
            msg.DataPtrForPack()[0] = enable_gps; 
            msg.SetDataLength(8);
            
            nmea2k->send_message(msg);

            uint16_t options_value = frontend.options.get();
            options_value &= ~uint16_t(AP_ExternalAHRS::OPTIONS::AN_SEND_GPS_MSG);
            frontend.options.set(options_value);

            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EAHRS_NMEA2K: sent AN_DISABLE_GNSS=%d", enable_gps);
            last_send_cmd = now;
        }
    }
}

bool AP_ExternalAHRS_NMEA2K::healthy(void) const
{
    WITH_SEMAPHORE(state.sem);
    return AP_HAL::millis() - last_att_ms < 500 ||
           AP_HAL::millis() - last_vel_ms < 500 ||
           AP_HAL::millis() - last_pos_ms < 500;
}

bool AP_ExternalAHRS_NMEA2K::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!initialized) {
        hal.util->snprintf(failure_msg, failure_msg_len, "NMEA2K setup failed");
        return false;
    }
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "NMEA2K unhealthy");
        return false;
    }
    WITH_SEMAPHORE(state.sem);
    uint32_t now = AP_HAL::millis();
    if (now - last_att_ms > 500 ||
            now - last_pos_ms > 500 ||
            now - last_vel_ms > 500) {
        hal.util->snprintf(failure_msg, failure_msg_len, "NMEA2K not up to date");
        return false;
    }
    return true;
}

bool AP_ExternalAHRS_NMEA2K::initialised(void) const
{
    return initialized;
}


void AP_ExternalAHRS_NMEA2K::get_filter_status(nav_filter_status &status) const
{
    WITH_SEMAPHORE(state.sem);
    uint32_t now = AP_HAL::millis();

    const uint32_t dt_limit = 1500;
    const uint32_t dt_limit_gps = 1500;

    // const bool init_ok = state.location.lat != 0 || state.location.lng != 0;

    status.flags.attitude = (now - last_att_ms < dt_limit);
    status.flags.horiz_vel = (now - last_vel_ms < dt_limit);
    status.flags.vert_vel = status.flags.horiz_vel;
    status.flags.horiz_pos_rel = state.have_origin && (now - last_pos_ms < dt_limit);
    status.flags.horiz_pos_abs = status.flags.horiz_pos_rel;
    status.flags.pred_horiz_pos_rel = status.flags.horiz_pos_abs;
    status.flags.pred_horiz_pos_abs = status.flags.horiz_pos_abs;
    // status.flags.vert_pos = (now - last_gps_ms < dt_limit);
    status.flags.vert_pos = true;
    status.flags.using_gps = (now - last_gps_ms < dt_limit_gps);
    status.flags.gps_quality_good = (now - last_gps_ms < dt_limit_gps) && (gps_data.fix_type >= AP_GPS_FixType::FIX_2D);
    status.flags.dead_reckoning = cached_data.pgn_129029_method == 6;

}

bool AP_ExternalAHRS_NMEA2K::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    velVar = 0.5f * vel_gate_scale;
    posVar = 0.5f * pos_gate_scale;
    hgtVar = 0.5f * hgt_gate_scale;
    magVar = Vector3f(mag_variation, mag_variation, mag_variation);
    tasVar = 0;
    return true;
}

void AP_ExternalAHRS_NMEA2K::handle_nmea2k_message(AP_NMEA2K* nmea2k_instance, nmea2k::N2KMessage& msg)
{
    const uint32_t now_ms = AP_HAL::millis();

    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EAHRS_NMEA2K: rx PGN %" PRIu32, msg.pgn());


    switch (msg.pgn()) {
    case 129025:
    {
        n2k_pgn_129025_position_rapid_update_t data;
        if (n2k_pgn_129025_position_rapid_update_unpack(&data, msg.DataPtrForUnpack(), msg.data_length()) != 0) {
            return;
        }

        WITH_SEMAPHORE(state.sem);
        state.location.lat = data.latitude;  // in 1e-7 degrees
        state.location.lng = data.longitude; // in 1e-7 degrees

        state.have_location = true;
        state.last_location_update_us = AP_HAL::micros();
        last_pos_ms = now_ms;

#if EXTERNAL_AHRS_DEBUG
        static uint32_t last_print_ms = 0;
        if (now_ms - last_print_ms > 5000) {
            last_print_ms = now_ms;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EAHRS_NMEA2K: pos lat %.7f lon %.7f",
                          state.location.lat * 1e-7,
                          state.location.lng * 1e-7);
        }
#endif

    }
    break;
    case 129029:
    {
        WITH_SEMAPHORE(state.sem);

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
        gps_data.gps_week = (uint16_t)(gps_ms / AP_MSEC_PER_WEEK);
        gps_data.ms_tow = (uint32_t)(gps_ms - (gps_data.gps_week * AP_MSEC_PER_WEEK));


        // lat/lng comes in at 1e-16 degrees. Convert to 1e-7 degrees.
        state.location.lat = static_cast<int32_t>(nmea2k::N2KMessage::ReadInt64(&data[i]) / 1000000000);
        i += 8;

        state.location.lng = static_cast<int32_t>(nmea2k::N2KMessage::ReadInt64(&data[i]) / 1000000000);
        i += 8;

        // altitude comes in 1e-6 meters. Convert to 1e-2.
        state.location.alt = static_cast<int32_t>(nmea2k::N2KMessage::ReadInt64(&data[i]) / 10000);
        i += 8;

        state.have_location = true;
        state.last_location_update_us = AP_HAL::micros();

        gps_data.latitude = state.location.lat;
        gps_data.longitude = state.location.lng;
        gps_data.msl_altitude = state.location.alt;

        cached_data.pgn_129029_method = (data[i] >> 4) & 0x0F;
        // const uint8_t type_of_system = data[i] & 0x0F;
        i += 1;

        // uint8_t integrity = data[i] >> 6;
        i += 1;

        gps_data.satellites_in_view = data[i];
        i += 1;

        switch (cached_data.pgn_129029_method) {
        case 0:
            gps_data.fix_type = AP_GPS_FixType::NONE;
            break;

        case 1:
            gps_data.fix_type = gps_data.satellites_in_view > 3 ? AP_GPS_FixType::FIX_3D : AP_GPS_FixType::FIX_2D;
            break;

        case 2:
            gps_data.fix_type = AP_GPS_FixType::DGPS;
            break;

        case 3:
            gps_data.fix_type = AP_GPS_FixType::FIX_3D;
            break;

        case 4:
            gps_data.fix_type = AP_GPS_FixType::RTK_FIXED;
            break;

        case 5:
            gps_data.fix_type = AP_GPS_FixType::RTK_FLOAT;
            break;

        case 6:
            // Dead Reckoning
            gps_data.fix_type = AP_GPS_FixType::FIX_2D;
            break;

        case 7:
            // Manual Input Mode
            gps_data.fix_type = AP_GPS_FixType::FIX_3D;
            break;

        case 8:
            // Simulator Mode
            gps_data.fix_type = AP_GPS_FixType::FIX_3D;
            break;

        default:
            gps_data.fix_type = AP_GPS_FixType::NONE;
            break;
        }

        // hdop comes in on 1e-2
        gps_data.hdop = static_cast<float>(nmea2k::N2KMessage::ReadInt16(&data[i])) / 100.0f;
        i += 2;

        // pdop comes in on 1e-2
        // pdop = abs(nmea2k::N2KMessage::ReadInt16(&data[i]));
        i += 2;


        uint8_t instance;
        if (AP::gps().get_first_external_instance(instance)) {
            AP::gps().handle_external(gps_data, instance);
        }

        if (gps_data.satellites_in_view > 3) {
            if (!state.have_origin) {
                state.origin = Location{
                    gps_data.latitude,
                    gps_data.longitude,
                    gps_data.msl_altitude,
                    Location::AltFrame::ABSOLUTE};
                state.have_origin = true;
            }

            last_gps_ms = now_ms;
        }

#if EXTERNAL_AHRS_DEBUG
        static uint32_t last_print_ms = 0;
        if (now_ms - last_print_ms > 5000) {
            last_print_ms = now_ms;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EAHRS_NMEA2K: pos lat %.7f lon %.7f sats %d",
                          gps_data.latitude * 1e-7,
                          gps_data.longitude * 1e-7,
                          gps_data.satellites_in_view);
        }
#endif // EXTERNAL_AHRS_DEBUG

    }
    break;

    case 129026:
    {
        n2k_pgn_129026_cog_sog_rapid_update_t data;
        if (n2k_pgn_129026_cog_sog_rapid_update_unpack(&data, msg.DataPtrForUnpack(), msg.data_length()) != 0) {
            return;
        }

        WITH_SEMAPHORE(state.sem);

        // cog comes in at 1e-4 radians.
        float ground_corse = data.cog * 0.0001f;

        // sog comes in at 1e-2 m/s. Convert to m/s.
        float ground_speed = data.sog * 0.01f;

        gps_data.ned_vel_north = ground_speed * cosf(ground_corse);
        gps_data.ned_vel_east = ground_speed * sinf(ground_corse);
        gps_data.ned_vel_down = 0.0f;

        state.velocity.x = gps_data.ned_vel_north;
        state.velocity.y = gps_data.ned_vel_east;
        state.velocity.z = gps_data.ned_vel_down;

        state.have_velocity = true;

        last_vel_ms = now_ms;

#if EXTERNAL_AHRS_DEBUG
        static uint32_t last_print_ms = 0;
        if (now_ms - last_print_ms > 5000) {
            last_print_ms = now_ms;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EAHRS_NMEA2K: vel N %.2f E %.2f S %.2f",
                          gps_data.ned_vel_north,
                          gps_data.ned_vel_east,
                          gps_data.ned_vel_down);
        }
#endif

    }
    break;

    case 127257:
    {
        n2k_pgn_127257_attitude_t data;
        if (n2k_pgn_127257_attitude_unpack(&data, msg.DataPtrForUnpack(), msg.data_length()) != 0) {
            return;
        }

        WITH_SEMAPHORE(state.sem);

        state.quat.from_euler(
            data.roll * 0.0001f,
            data.pitch * 0.0001f,
            data.yaw * 0.0001f
        );

        state.have_quaternion = true;
        last_att_ms = now_ms;

#if EXTERNAL_AHRS_DEBUG
        static uint32_t last_print_ms = 0;
        if (now_ms - last_print_ms > 5000) {
            last_print_ms = now_ms;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EAHRS_NMEA2K: att R %.0f P %.0f Y %.0f",
                          degrees(data.roll * 0.0001f),
                          degrees(data.pitch * 0.0001f),
                          degrees(data.yaw * 0.0001f));
        }
#endif // EXTERNAL_AHRS_DEBUG

    }
    break;

    case 127258:
    {
        n2k_pgn_127258_magnetic_variation_t data;
        if (n2k_pgn_127258_magnetic_variation_unpack(&data, msg.DataPtrForUnpack(), msg.data_length()) != 0) {
            return;
        }

        WITH_SEMAPHORE(state.sem);

        mag_variation = data.variation * 0.0001f;

#if EXTERNAL_AHRS_DEBUG
        static uint32_t last_print_ms = 0;
        if (now_ms - last_print_ms > 10000) {
            last_print_ms = now_ms;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EAHRS_NMEA2K: mag var %.2f deg",
                          degrees(mag_variation));
        }
#endif // EXTERNAL_AHRS_DEBUG

    } break;

    case 127250:
    {

        n2k_pgn_127250_vessel_heading_t data;
        if (n2k_pgn_127250_vessel_heading_unpack(&data, msg.DataPtrForUnpack(), msg.data_length()) != 0) {
            return;
        }

        WITH_SEMAPHORE(state.sem);

        float pitch, roll, yaw;
        state.quat.to_euler(roll, pitch, yaw);

        yaw = data.heading * 0.0001f;

        state.quat.from_euler(
            roll,
            pitch,
            yaw
        );

        state.have_quaternion = true;
        last_att_ms = now_ms;

#if EXTERNAL_AHRS_DEBUG
        static uint32_t last_print_ms = 0;
        if (now_ms - last_print_ms > 5000) {
            last_print_ms = now_ms;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EAHRS_NMEA2K: heading %.2f", degrees(data.heading * 0.0001f));
        }
#endif // EXTERNAL_AHRS_DEBUG


    } break;

    }


}

#endif // AP_EXTERNAL_AHRS_NMEA2K_ENABLED