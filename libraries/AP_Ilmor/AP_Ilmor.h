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

#define AP_ILMOR_DEBUG 1

#include <AP_Ilmor/ilmor.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANSensor.h>
#include <AP_Param/AP_Param.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>
#include <AP_J1939_CAN/AP_J1939_CAN.h>

#define AP_ILMOR_MAX_FAULTS 4 // Maximum number of active faults we can track


class IlmorFwVersion {
    public:
        IlmorFwVersion() : major(0), minor(0), patch(0), dev_stage(0), dev_stage_rev(0) {}
        IlmorFwVersion(uint8_t maj, uint8_t min, uint8_t pat, uint8_t stage, uint8_t stage_rev) :
            major(maj), minor(min), patch(pat), dev_stage(stage), dev_stage_rev(stage_rev) {}

        void print() const;

        bool operator<(const IlmorFwVersion& other) const;

        uint8_t major;  // Major version
        uint8_t minor;  // Minor version
        uint8_t patch;  // Patch version
        uint8_t dev_stage;
        uint8_t dev_stage_rev;
};


class MessageRateIIR {
    public:
        MessageRateIIR(float time_constant_sec = 5.0f);

        void msg_received();
        float rate_hz();
    
    private:
        float _tau;
        uint32_t _last_update_ms;
        float _average_rate_hz;
        HAL_Semaphore sem;

        void update_state();
};

class OneShotTimer {
public:
    OneShotTimer() : _deadline_ms(0), _is_timed_out(true) {}
    OneShotTimer(const uint32_t timeout_ms);
    void reset(const uint32_t timeout_ms);
    bool is_timed_out();

private:
    uint32_t _deadline_ms;
    bool _is_timed_out;
};


class AP_Ilmor : public CANSensor
#if HAL_WITH_ESC_TELEM
, public AP_ESC_Telem_Backend
#endif
{
public:
    AP_Ilmor();

    CLASS_NO_COPY(AP_Ilmor);

    static const struct AP_Param::GroupInfo var_info[];

    static AP_Ilmor *get_ilmor(uint8_t driver_index);

    void init(uint8_t driver_index, bool enable_filters) override;

    // called from SRV_Channels
    void update();

    
    bool healthy();

    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len);

private:

    enum class LEDMode : uint8_t {
        Off = 1,
        Solid = 3,
        Flashing = 4,
        Sweeping = 5,
        Random = 6,
        Fireworks = 7,
    } _led_mode;

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
        EStop,
        AutoDown,
    } _trimState;

    enum class MotorState {
        Ready,
        Init,
        StopWait,
        Stop,
        Forward,
        Reverse,
        Fault,
        WifiOn,
    } _motor_state;


    enum class ClearFaultsState {
        Ready,
        Cleared,
    } _clear_faults_state;

#ifdef AP_ILMOR_DEBUG
    enum class ICULoggingState {
        Idle,
        StartLogging,
        StopLogging,
        Logging,
        Wipe,
    } _icu_logging_state;
#endif


    // Parameters
    AP_Int16 _min_rpm;
    AP_Int16 _max_rpm;
    AP_Int8 _trim_fn;
    AP_Int8 _max_run_trim;
    AP_Int8 _can_port;
    AP_Int16 _trim_stop;
    AP_Int8 _fw_update;
    AP_Int8 _clear_faults_request;
#ifdef AP_ILMOR_DEBUG
    AP_Int8 _icu_logging;
#endif
    AP_Int8 _auto_trim_down;
    AP_Int8 _auto_trim_down_threshold;
    AP_Int32 _auto_trim_down_period;

    uint8_t _current_trim_position;
    int32_t _last_rpm;
    uint32_t _last_motor_wait_ms;
    uint32_t _last_trim_wait_ms;
    uint32_t _last_fault_notify_ms;
    J1939::DiagnosticMessage1::DTC _active_faults[AP_ILMOR_MAX_FAULTS];
    uint8_t _num_active_faults;
    uint8_t _num_tp_packets;
    uint32_t _last_print_faults_ms;
    IlmorFwVersion _ilmor_fw_version;
    uint8_t _led_hue;
    int16_t _rpm_demand;
    uint8_t _server_mode;
    uint32_t _last_send_frame1_ms;
    uint32_t _last_auto_trim_down_ms;
    MessageRateIIR _icu_msg_rate;
    MessageRateIIR _inverter_msg_rate;
    OneShotTimer _print_fw_version_timer;

    struct run_state {
        run_state() :
            last_send_throttle_ms(0),
            last_send_trim_ms(0),
            last_trim_cmd(TRIM_CMD_STOP) {}

        uint32_t last_send_throttle_ms;
        uint32_t last_send_trim_ms;
        TrimCmd last_trim_cmd;
    } _run_state;

    struct command_output {
        command_output() :
            motor_rpm(0),
            motor_trim(TRIM_CMD_BUTTONS) {}

        int16_t motor_rpm;
        TrimCmd motor_trim;
    } _output;

    char _thread_name[10];
    
    void run_io(void);
    void tick(void);
    void send_throttle_cmd();
    void send_trim_cmd();

    // handler for incoming frames
    void handle_frame(AP_HAL::CANFrame &frame) override;

    bool send_unmanned_throttle_control(const struct ilmor_unmanned_throttle_control_t &msg);
    void send_direct_inverter();
    bool send_r3_status_frame_1();
    bool send_r3_status_frame_2(const struct ilmor_r3_status_frame_2_t &msg);

    void handle_unmanned_throttle_control(const struct ilmor_unmanned_throttle_control_t &msg);
    void handle_r3_status_frame_2(const struct ilmor_r3_status_frame_2_t &msg);
    void handle_icu_status_frame_1(const struct ilmor_icu_status_frame_1_t &msg);
    void handle_icu_status_frame_2(const struct ilmor_icu_status_frame_2_t &msg);
    void handle_icu_status_frame_7(const struct ilmor_icu_status_frame_7_t &msg);
    void handle_inverter_status_frame_1(const struct ilmor_inverter_status_frame_1_t &msg);
    void handle_inverter_status_frame_2(const struct ilmor_inverter_status_frame_2_t &msg);
    void handle_inverter_status_frame_3(const struct ilmor_inverter_status_frame_3_t &msg);
    void handle_inverter_status_frame_4(const struct ilmor_inverter_status_frame_4_t &msg);
    void handle_inverter_status_frame_5(const struct ilmor_inverter_status_frame_5_t &msg);

    bool soft_stop_exceeded();
    TrimCmd trim_demand();

    void trim_state_machine();
    void motor_state_machine();
    void clear_faults_state_machine();

#ifdef AP_ILMOR_DEBUG
    void icu_logging_state_machine();
#endif

    void active_fault(J1939::DiagnosticMessage1::DTC& dtc);
    void report_faults();

    /// @brief is the motor allowed to run?
    /// @return true if the motor is locked out, false otherwise
    bool is_locked_out();

    /// @brief is the ICU healthy?
    /// Healthy is determined if we have seen a CAN message from the ICU within the past 1 second.
    /// @return true if the ICU is healthy, false otherwise
    bool icu_healthy();

    /// @brief is the inverter healthy?
    /// Healthy is determined if we have seen a CAN message from the inverter within the past 1 second.
    /// @return true if the inverter is healthy, false otherwise
    bool inverter_healthy();

};

#endif // HAL_ILMOR_ENABLED
