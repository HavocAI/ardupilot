/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
* AP_Ilmor.h
*
*      Author: Andrew Gregg
*/

#pragma once

#include <AP_Ilmor/AP_Ilmor_config.h>

#if HAL_ILMOR_ENABLED
#include <AP_Ilmor/ilmor.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANSensor.h>
#include <AP_Param/AP_Param.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>

class AP_Ilmor_Driver : public CANSensor, public AP_ESC_Telem_Backend
{
public:
    AP_Ilmor_Driver();

    // called from SRV_Channels
    void update();

private:
    // handler for incoming frames
    void handle_frame(AP_HAL::CANFrame &frame) override;

    bool send_packet(const uint32_t id, const uint32_t timeout_us, const uint8_t *data, const uint8_t data_len);

    bool send_unmanned_throttle_control(const struct ilmor_unmanned_throttle_control_t &msg);
    bool send_r3_status_frame_2(const struct ilmor_r3_status_frame_2_t &msg);

    void loop();

    void handle_unmanned_throttle_control(const struct ilmor_unmanned_throttle_control_t &msg);
    void handle_r3_status_frame_2(const struct ilmor_r3_status_frame_2_t &msg);
    void handle_icu_status_frame_1(const struct ilmor_icu_status_frame_1_t &msg);
    void handle_icu_status_frame_2(const struct ilmor_icu_status_frame_2_t &msg);
    void handle_inverter_status_frame_1(const struct ilmor_inverter_status_frame_1_t &msg);
    void handle_inverter_status_frame_2(const struct ilmor_inverter_status_frame_2_t &msg);
    void handle_inverter_status_frame_3(const struct ilmor_inverter_status_frame_3_t &msg);
    void handle_inverter_status_frame_4(const struct ilmor_inverter_status_frame_4_t &msg);
    void handle_inverter_status_frame_5(const struct ilmor_inverter_status_frame_5_t &msg);

    // output (command) struct
    struct
    {
        HAL_Semaphore sem;
        bool is_new;
        uint32_t last_new_ms;
        int16_t motor_rpm;
        uint8_t motor_trim;
    } _output;

    uint8_t _current_trim_position;
};

class AP_Ilmor
{
public:
    AP_Ilmor();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Ilmor);

    static const struct AP_Param::GroupInfo var_info[];

    void init();
    void update();

    static AP_Ilmor *get_singleton() { return _singleton; }

    // Method to get the values of params
    int16_t get_min_rpm() const { return _min_rpm.get(); }
    int16_t get_max_rpm() const { return _max_rpm.get(); }
    int8_t get_min_trim() const { return _min_trim.get(); }
    int8_t get_max_trim() const { return _max_trim.get(); }
    int16_t get_trim_fn() const { return _trim_fn.get(); }

private:
    static AP_Ilmor *_singleton;

    AP_Ilmor_Driver *_driver;

    // Parameters
    AP_Int16 _min_rpm;
    AP_Int16 _max_rpm;
    AP_Int8 _min_trim;
    AP_Int8 _max_trim;
    AP_Int8 _trim_fn;

};
namespace AP
{
    AP_Ilmor *ilmor();
};

#endif // HAL_ILMOR_ENABLED
