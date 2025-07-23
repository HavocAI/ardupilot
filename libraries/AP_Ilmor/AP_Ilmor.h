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
#include <AP_J1939_CAN/AP_J1939_CAN.h>

class AP_Ilmor : public CANSensor, public AP_ESC_Telem_Backend
{
public:
    AP_Ilmor();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Ilmor);

    static const struct AP_Param::GroupInfo var_info[];

    void init();
    void update();
    bool healthy() const;

    static AP_Ilmor *get_singleton() { return _singleton; }

    // run pre-arm check.  returns false on failure and fills in failure_msg
    // any failure_msg returned will not include a prefix
    bool pre_arm_checks(char *failure_msg, uint8_t failure_msg_len);

    // Method to get the values of params
    int16_t get_min_rpm() const { return _min_rpm.get(); }
    int16_t get_max_rpm() const { return _max_rpm.get(); }
    int16_t get_trim_fn() const { return _trim_fn.get(); }
    int8_t get_max_run_trim() const { return _max_run_trim.get(); }
    int8_t get_can_port() const { return _can_port.get(); }

private:

    enum TrimCmd : uint8_t {
        TRIM_CMD_STOP = 0,
        TRIM_CMD_UP = 1,
        TRIM_CMD_DOWN = 2,
        TRIM_CMD_BUTTONS = 255,
    };

    enum class TrimState {
        Start,
        CheckSoftStop,
        Manual,
        CheckRelease,
        CmdDown,
        CmdStop,
    };

    enum class MotorState {
        Ready,
        Stop,
        Forward,
        Reverse,
        Error,
    } _motor_state;

    enum class ComsState {
        Waiting,
        Running,
        Unhealthy,
    } _comsState;

    static AP_Ilmor *_singleton;

    AP_J1939_CAN* j1939;
    TrimState _trimState;

    // Parameters
    AP_Int16 _min_rpm;
    AP_Int16 _max_rpm;
    AP_Int8 _trim_fn;
    AP_Int8 _max_run_trim;
    AP_Int8 _can_port;
    AP_Int16 _trim_stop;

    uint8_t _current_trim_position;
    uint8_t _trim_command_from_buttons;
    bool _trim_locked_out = true;
    uint32_t _last_wait_ms;
    int32_t _last_rpm;
    uint32_t _last_com_wait_ms;

    struct run_state {
        run_state() :
            last_send_throttle_ms(0),
            last_send_trim_ms(0),
            last_received_msg_ms(0),
            last_trim_cmd(TRIM_CMD_BUTTONS) {}

        uint32_t last_send_throttle_ms;
        uint32_t last_send_trim_ms;
        uint32_t last_received_msg_ms;
        TrimCmd last_trim_cmd;
    } _run_state;

    struct command_output {
        command_output() :
            motor_rpm(0),
            motor_trim(TRIM_CMD_BUTTONS) {}

        int16_t motor_rpm;
        TrimCmd motor_trim;
    } _output;
    
    void run_io(void);
    void tick(void);
    void send_throttle_cmd();
    void send_trim_cmd();

    // handler for incoming frames
    void handle_frame(AP_HAL::CANFrame &frame) override;

    bool send_unmanned_throttle_control(const struct ilmor_unmanned_throttle_control_t &msg);
    bool send_r3_status_frame_2(const struct ilmor_r3_status_frame_2_t &msg);

    void handle_unmanned_throttle_control(const struct ilmor_unmanned_throttle_control_t &msg);
    void handle_r3_status_frame_2(const struct ilmor_r3_status_frame_2_t &msg);
    void handle_icu_status_frame_1(const struct ilmor_icu_status_frame_1_t &msg);
    void handle_icu_status_frame_7(const struct ilmor_icu_status_frame_7_t &msg);
    void handle_inverter_status_frame_1(const struct ilmor_inverter_status_frame_1_t &msg);
    void handle_inverter_status_frame_2(const struct ilmor_inverter_status_frame_2_t &msg);
    void handle_inverter_status_frame_3(const struct ilmor_inverter_status_frame_3_t &msg);
    void handle_inverter_status_frame_4(const struct ilmor_inverter_status_frame_4_t &msg);
    void handle_inverter_status_frame_5(const struct ilmor_inverter_status_frame_5_t &msg);

    bool soft_stop_exceeded();
    TrimCmd trim_demand();

    void trim_state_machine();

};
namespace AP
{
    AP_Ilmor *ilmor();
};

#endif // HAL_ILMOR_ENABLED
