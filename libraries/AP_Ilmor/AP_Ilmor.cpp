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
    AP_Ilmor.cpp

    Author: Andrew Gregg

    Description:
    A Rover CAN driver to control the RPM of the Ilmor Ion motor via the "Throttle" servo function.
    Motor trim can be commanded on a user-definable servo function (see parameters below).
    Motor trim is set using a simple controller in the driver. If trim is not being actively commanded
    up or down by the controller, it will be relinquished to the physical trim buttons. Limited
    telemetry is available via the ESC_ telemetry information.

    Configuring CAN Parameters in ArduRover:
    (Example shows CAN_P2 because this connects to the port marked "CAN0" on the Airbot carrier board)
    CAN_D1_PROTOCOL = 15 -- Sets the driver 1 protocol to Ilmor
    CAN_P2_DRIVER = 1 -- Sets the 2nd CAN port to use driver 1
    CAN_P2_BITRATE = 250000 -- Sets the 2nd CAN port bitrate

    Settable Parameters - Description, Default value:
    ILMOR_ESC_IDX - Defines the index for ESC telemetry reporting ("X" in telemetry outputs below), 0
    ILMOR_MAX_RPM - Maximum RPM in forward, 1600
    ILMOR_MIN_RPM - Minimum RPM in reverse, -500
    ILMOR_MAX_TRIM - Maximum trim (full up), 254
    ILMOR_MIN_TRIM - Minimum trim (full down), 0
    ILMOR_TRIM_FN - Servo function that sets the target motor trim, Mount1 Tilt/Pitch (7)
    (see https://ardupilot.org/rover/docs/parameters.html#servo1-function-servo-output-function)

    Telemetry Outputs:
    escX_curr = Ilmor Battery Current (A)
    escX_rpm = Ilmor eRPM x 5 (aka. prop RPM)
    escX_temp = Ilmor Motor Temperature (centi-deg)
    escX_volt = Ilmor Low Precision Battery Voltage (V)

*/

#include "AP_Ilmor.h"

#if HAL_ILMOR_ENABLED
#include <stdio.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

#define AP_ILMOR_DEBUG 0

// table of user settable CAN bus parameters
const AP_Param::GroupInfo AP_Ilmor::var_info[] = {

    // @Param: MIN_RPM
    // @DisplayName: Ilmor Motor Minimum RPM
    // @Description: Ilmor Motor Minimum RPM, by default this is set to -500
    // @Values: -2000:2000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MIN_RPM", 0, AP_Ilmor, _min_rpm, -500),

    // @Param: MAX_RPM
    // @DisplayName: Ilmor Motor Maximum RPM
    // @Description: Ilmor Motor Maximum RPM, by default this is set to 1600
    // @Values: -2000:2000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MAX_RPM", 1, AP_Ilmor, _max_rpm, 1600),

    // @Param: MIN_TRIM
    // @DisplayName: Ilmor Motor Minimum Trim
    // @Description: Ilmor Motor Minimum Trim, by default this is set to 0
    // @Values: 0:254
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MIN_TRIM", 2, AP_Ilmor, _min_trim, 0),

    // @Param: MAX_TRIM
    // @DisplayName: Ilmor Motor Maximum Trim
    // @Description: Ilmor Motor Maximum Trim, by default this is set to 254
    // @Values: 0:254
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MAX_TRIM", 3, AP_Ilmor, _max_trim, 254),

    // @Param: TRIM_FN
    // @DisplayName: Ilmor Motor Trim Servo Channel
    // @Description: Ilmor Motor Trim Servo Channel, by default this is set to "mount1 tilt/pitch"
    // @Values: 0:255
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("TRIM_FN", 4, AP_Ilmor, _trim_fn, (int16_t)SRV_Channel::k_mount_tilt),

    // @Param: ESC_IDX
    // @DisplayName: Ilmor ESC Index
    // @Description: Ilmor ESC Index, by default this is set to 0
    // @Values: 0:15
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("ESC_IDX", 5, AP_Ilmor, _esc_idx, 0),

    AP_GROUPEND};

AP_Ilmor::AP_Ilmor()
{
    AP_Param::setup_object_defaults(this, var_info);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr)
    {
        AP_HAL::panic("AP_Ilmor must be singleton");
    }
#endif
    _singleton = this;
}

void AP_Ilmor::init()
{
    if (_driver != nullptr)
    {
        // only allow one instance
        return;
    }

    for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++)
    {
        if (CANSensor::get_driver_type(i) == AP_CAN::Protocol::Ilmor)
        {
            _driver = NEW_NOTHROW AP_Ilmor_Driver();
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Ilmor: Initialized using CAN%d", i);
            return;
        }
    }
}

void AP_Ilmor::update()
{
    if (_driver == nullptr)
    {
        return;
    }
    _driver->update();
}

AP_Ilmor_Driver::AP_Ilmor_Driver() : CANSensor("Ilmor")
{
    register_driver(AP_CAN::Protocol::Ilmor);

    // start thread for receiving and sending CAN frames.
    hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Ilmor_Driver::loop, void), "ilmor", 2048, AP_HAL::Scheduler::PRIORITY_CAN, 0);
}

// parse inbound frames
void AP_Ilmor_Driver::handle_frame(AP_HAL::CANFrame &frame)
{
    if (!frame.isExtended()) {
        return;
    }
    const uint32_t ext_id = frame.id & AP_HAL::CANFrame::MaskExtID;

    switch (ext_id)
    {
    case ILMOR_UNMANNED_THROTTLE_CONTROL_FRAME_ID:
    {
        // Monitor in case there are other unmanned controllers on the network
        struct ilmor_unmanned_throttle_control_t msg;
        ilmor_unmanned_throttle_control_unpack(&msg, frame.data, frame.dlc);
        handle_unmanned_throttle_control(msg);
        break;
    }
    case ILMOR_R3_STATUS_FRAME_2_FRAME_ID:
    {
        // Monitor in case there are other unmanned controllers on the network
        struct ilmor_r3_status_frame_2_t msg;
        ilmor_r3_status_frame_2_unpack(&msg, frame.data, frame.dlc);
        handle_r3_status_frame_2(msg);
        break;
    }
    case ILMOR_ICU_STATUS_FRAME_1_FRAME_ID:
    {
        // Trim position adjusted
        struct ilmor_icu_status_frame_1_t msg;
        ilmor_icu_status_frame_1_unpack(&msg, frame.data, frame.dlc);
        handle_icu_status_frame_1(msg);
        break;
    }
    case ILMOR_INVERTER_STATUS_FRAME_1_FRAME_ID:
    {
        // eRPM
        struct ilmor_inverter_status_frame_1_t msg;
        ilmor_inverter_status_frame_1_unpack(&msg, frame.data, frame.dlc);
        handle_inverter_status_frame_1(msg);
        break;
    }
    case ILMOR_INVERTER_STATUS_FRAME_2_FRAME_ID:
    {
        // Ah consumed
        struct ilmor_inverter_status_frame_2_t msg;
        ilmor_inverter_status_frame_2_unpack(&msg, frame.data, frame.dlc);
        handle_inverter_status_frame_2(msg);
        break;
    }
    case ILMOR_INVERTER_STATUS_FRAME_4_FRAME_ID:
    {
        // MOSFET temperature, motor temperature, battery current
        struct ilmor_inverter_status_frame_4_t msg;
        ilmor_inverter_status_frame_4_unpack(&msg, frame.data, frame.dlc);
        handle_inverter_status_frame_4(msg);
        break;
    }
    case ILMOR_INVERTER_STATUS_FRAME_5_FRAME_ID:
    {
        // Battery voltage (low precision)
        struct ilmor_inverter_status_frame_5_t msg;
        ilmor_inverter_status_frame_5_unpack(&msg, frame.data, frame.dlc);
        handle_inverter_status_frame_5(msg);
        break;
    }
    default:
        // Ignore other frames
        break;
    }
}

// update the output from the throttle servo channel
void AP_Ilmor_Driver::update()
{
    WITH_SEMAPHORE(_output.sem);
    if (SRV_Channels::channel_function(0) <= SRV_Channel::Aux_servo_function_t::k_none)
    {
        _output.motor_rpm = 0;
        _output.motor_trim = 255;
        return;
    }

    // get the desired motor speed from the throttle channel and
    // the desired trim from the trim channel
    // normalized by the Ilmor ranges

    _output.motor_rpm = constrain_int16(SRV_Channels::get_output_norm(
        SRV_Channel::k_throttle) *
        AP::ilmor()->get_max_rpm(), AP::ilmor()->get_min_rpm(),  AP::ilmor()->get_max_rpm());

    _output.motor_trim = constrain_int16(SRV_Channels::get_output_norm(
        (SRV_Channel::Aux_servo_function_t)AP::ilmor()->get_trim_fn()) *
        AP::ilmor()->get_max_trim(), AP::ilmor()->get_min_trim(), AP::ilmor()->get_max_trim());
    

    _output.is_new = true;

#if AP_ILMOR_DEBUG
    static uint32_t last_send_ms = 0;
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_send_ms > 1000)
    {
        last_send_ms = now_ms;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Ilmor: RPM command %u, Trim command %u",
                      (unsigned)_output.motor_rpm, (unsigned)_output.motor_trim);
    }
#endif
}

void AP_Ilmor_Driver::loop()
{
    int16_t motor_rpm_cmd = 0;
    uint8_t trim_cmd = 0;

    while (true)
    {
        hal.scheduler->delay_microseconds(20000); // 50Hz

        const uint32_t now_ms = AP_HAL::millis();

        // This should run at 50Hz
        {
            WITH_SEMAPHORE(_output.sem);
            if (_output.is_new)
            {
                _output.last_new_ms = now_ms;
                _output.is_new = false;
                motor_rpm_cmd = _output.motor_rpm;
                trim_cmd = _output.motor_trim;
            }
            else if (_output.last_new_ms && now_ms - _output.last_new_ms > 1000)
            {
                // if we haven't gotten any PWM updates for a bit, stop movement
                // out so we don't just keep sending the same values forever
                motor_rpm_cmd = 0;
                trim_cmd = 255;
                _output.last_new_ms = 0;
            }
        }

        ilmor_unmanned_throttle_control_t throttle_msg;
        // Set the magic number to allow unmanned control
        throttle_msg.unmanned_control_key = 
            ilmor_unmanned_throttle_control_unmanned_control_key_encode(0x4D);
        // Set the desired motor speed
        throttle_msg.unmanned_p_rpm_demand = 
            ilmor_unmanned_throttle_control_unmanned_p_rpm_demand_encode(motor_rpm_cmd);

        send_unmanned_throttle_control(throttle_msg);

        // Use a simple bang-bang controller to set the trim to the desired position
        ilmor_r3_status_frame_2_t r3_status_frame_2_msg;
        if ((trim_cmd < _current_trim_position) && (trim_cmd < 255))
        {
            // Trim up
            r3_status_frame_2_msg.trim_demand = ilmor_r3_status_frame_2_trim_demand_encode(1);
        }
        else if ((trim_cmd > _current_trim_position) && (trim_cmd < 255))
        {
            // Trim down
            r3_status_frame_2_msg.trim_demand = ilmor_r3_status_frame_2_trim_demand_encode(2);
        }
        else
        {
            // Set the trim to physical button control (255)
            r3_status_frame_2_msg.trim_demand = ilmor_r3_status_frame_2_trim_demand_encode(255);
        }
        send_r3_status_frame_2(r3_status_frame_2_msg);

    } // while true
}

bool AP_Ilmor_Driver::send_packet(const uint32_t id, const uint32_t timeout_us, 
    const uint8_t *data, const uint8_t data_len)
{
    AP_HAL::CANFrame frame;
    frame.id = id | AP_HAL::CANFrame::FlagEFF;
    frame.dlc = data_len;
    frame.canfd = false;
    memcpy(frame.data, data, data_len);

    return write_frame(frame, timeout_us);
}

bool AP_Ilmor_Driver::send_unmanned_throttle_control(const struct ilmor_unmanned_throttle_control_t &msg)
{
    // Unmanned control key: Should be set to 0x4D to allow unmanned control
    uint8_t data[ILMOR_UNMANNED_THROTTLE_CONTROL_LENGTH];
    ilmor_unmanned_throttle_control_pack(data, &msg, sizeof(data));
    return send_packet(ILMOR_UNMANNED_THROTTLE_CONTROL_FRAME_ID, 1000, data, sizeof(data));
}

bool AP_Ilmor_Driver::send_r3_status_frame_2(const struct ilmor_r3_status_frame_2_t &msg)
{
    // Trim demand values: 0 = stop, 1 = up, 2 = down, 255 = trim control is given to the physical buttons
    uint8_t data[ILMOR_R3_STATUS_FRAME_2_LENGTH];
    ilmor_r3_status_frame_2_pack(data, &msg, sizeof(data));
    return send_packet(ILMOR_R3_STATUS_FRAME_2_FRAME_ID, 1000, data, sizeof(data));
}

void AP_Ilmor_Driver::handle_unmanned_throttle_control(const struct ilmor_unmanned_throttle_control_t &msg)
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Ilmor: Unmanned Throttle Control msg received from another CAN device");
}

void AP_Ilmor_Driver::handle_r3_status_frame_2(const struct ilmor_r3_status_frame_2_t &msg)
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Ilmor: R3 Status Frame 2 msg received from another CAN device");
}

void AP_Ilmor_Driver::handle_icu_status_frame_1(const struct ilmor_icu_status_frame_1_t &msg)
{
    _current_trim_position = msg.trim_position_adjusted;
}

void AP_Ilmor_Driver::handle_inverter_status_frame_1(const struct ilmor_inverter_status_frame_1_t &msg)
{
    update_rpm(AP::ilmor()->get_esc_idx(), int32_t(msg.e_rpm * 5)); // Multiply by 5 to get Prop RPM
}

void AP_Ilmor_Driver::handle_inverter_status_frame_2(const struct ilmor_inverter_status_frame_2_t &msg)
{
    const TelemetryData t = {
        .consumption_mah = float((msg.ah_consumed) * 1000.0f),
    };
    update_telem_data(AP::ilmor()->get_esc_idx(), t,
                      AP_ESC_Telem_Backend::TelemetryType::CONSUMPTION);
}

void AP_Ilmor_Driver::handle_inverter_status_frame_4(const struct ilmor_inverter_status_frame_4_t &msg)
{
    const TelemetryData t = {
        .temperature_cdeg = int16_t(msg.mosfet_temperature * 100),
        .current = float(msg.battery_current),
        .motor_temp_cdeg = int16_t(msg.motor_temperature * 100),
    };
    update_telem_data(AP::ilmor()->get_esc_idx(), t,
                      AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE |
                          AP_ESC_Telem_Backend::TelemetryType::CURRENT |
                          AP_ESC_Telem_Backend::TelemetryType::MOTOR_TEMPERATURE);
#if AP_ILMOR_DEBUG
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Ilmor: MOSFET Temp %d, Motor Temp %d, Battery Current %f",
                      (int)msg.mosfet_temperature * 100, (int)msg.motor_temperature * 100, float(msg.battery_current));
#endif
}

void AP_Ilmor_Driver::handle_inverter_status_frame_5(const struct ilmor_inverter_status_frame_5_t &msg)
{
    const TelemetryData t = {
        .voltage = float(msg.low_precision_battery_voltage),
    };
    update_telem_data(AP::ilmor()->get_esc_idx(), t,
                      AP_ESC_Telem_Backend::TelemetryType::VOLTAGE);
}

// singleton instance
AP_Ilmor *AP_Ilmor::_singleton;

namespace AP
{
    AP_Ilmor *ilmor()
    {
        return AP_Ilmor::get_singleton();
    }
};

#endif // HAL_ILMOR_ENABLED
