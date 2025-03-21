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
    telemetry is available via the escX_ telemetry information, some of which is hacked in to the
    multiple ESC telemetry slots.

    Configuring CAN Parameters in ArduRover: configure the CAN port to use the J1939 CAN backend
    (See AP_J1939_CAN/AP_J1939_CAN.h for more information)

    Settable Parameters - Description, Default value:
    ILMOR_MAX_RPM - Maximum RPM in forward, 1600
    ILMOR_MIN_RPM - Minimum RPM in reverse, -500
    ILMOR_MAX_TRIM - Maximum trim (full up), 254
    ILMOR_MIN_TRIM - Minimum trim (full down), 0
    ILMOR_TRIM_FN - Servo function that sets the target motor trim, Gripper (28)
    (see https://ardupilot.org/rover/docs/parameters.html#servo1-function-servo-output-function)

    Telemetry Outputs:
    esc1_curr = Ilmor Battery Current (A)
    esc1_rpm  = Ilmor eRPM / 5 (aka. prop RPM)
    esc1_temp = Ilmor Motor Temperature (centi-deg)
    esc1_volt = Ilmor Low Precision Battery Voltage (V)
    esc2_curr = Ilmor Motor Current (A)
    esc2_temp = Ilmor MOSFET Temperature (centi-deg)
    esc3_volt = Ilmor Total Wh consumed since boot (Wh)
    esc3_curr = Ilmor Total Ah consumed since boot (Ah)
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
#define TRIM_DEADBAND 10

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

    // @Param: TRIM_FN
    // @DisplayName: Ilmor Motor Trim Servo Channel
    // @Description: Ilmor Motor Trim Servo Channel, by default this is set to "mount1 tilt/pitch"
    // @Values: 0:255
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("TRIM_FN", 4, AP_Ilmor, _trim_fn, (int16_t)SRV_Channel::k_gripper),

    // @Param: MAX_TRIM
    // @DisplayName: Ilmor Motor Maximum Trim
    // @Description: Ilmor Motor Maximum Trim that the motor will be allowed to run at
    // @Values: 0:127
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MAX_TRIM", 5, AP_Ilmor, _max_run_trim, 127),

    // @Param: CAN_PORT
    // @DisplayName: Ilmor CAN Port
    // @Description: Ilmor CAN Port, by default this is set to 0
    // @Values: 0:1
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("CAN_PORT", 6, AP_Ilmor, _can_port, 0),

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
    if (_can_port < 0 || _can_port >= HAL_NUM_CAN_IFACES)
    {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Ilmor: Invalid CAN port");
        return;
    }

    AP_J1939_CAN* j1939 = AP_J1939_CAN::get_instance(_can_port);

    _driver = NEW_NOTHROW AP_Ilmor_Driver();
    if (!_driver)
    {
        return;
    }

    // Register the driver with the J1939 CAN backend for the Ilmor specific CAN IDs
    if (!j1939->register_driver(ILMOR_UNMANNED_THROTTLE_CONTROL_FRAME_ID, _driver) ||
        !j1939->register_driver(ILMOR_R3_STATUS_FRAME_2_FRAME_ID, _driver) ||
        !j1939->register_driver(ILMOR_ICU_STATUS_FRAME_1_FRAME_ID, _driver) ||
        !j1939->register_driver(ILMOR_ICU_STATUS_FRAME_7_FRAME_ID, _driver) ||
        !j1939->register_driver(ILMOR_INVERTER_STATUS_FRAME_1_FRAME_ID, _driver) ||
        !j1939->register_driver(ILMOR_INVERTER_STATUS_FRAME_2_FRAME_ID, _driver) ||
        !j1939->register_driver(ILMOR_INVERTER_STATUS_FRAME_3_FRAME_ID, _driver) ||
        !j1939->register_driver(ILMOR_INVERTER_STATUS_FRAME_4_FRAME_ID, _driver) ||
        !j1939->register_driver(ILMOR_INVERTER_STATUS_FRAME_5_FRAME_ID, _driver))
    {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Ilmor: Failed to register with J1939");
        return;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Ilmor: Registered with J1939 on CAN%d", static_cast<int>(_can_port));
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
    case ILMOR_ICU_STATUS_FRAME_7_FRAME_ID:
    {
        // Trim demand request from buttons
        struct ilmor_icu_status_frame_7_t msg;
        ilmor_icu_status_frame_7_unpack(&msg, frame.data, frame.dlc);
        handle_icu_status_frame_7(msg);
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
    case ILMOR_INVERTER_STATUS_FRAME_3_FRAME_ID:
    {
        // Wh consumed
        struct ilmor_inverter_status_frame_3_t msg;
        ilmor_inverter_status_frame_3_unpack(&msg, frame.data, frame.dlc);
        handle_inverter_status_frame_3(msg);
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

    _output.motor_rpm = 0;
    _output.motor_trim = 255;

    _output.motor_rpm = constrain_int16(SRV_Channels::get_output_norm(
        SRV_Channel::k_throttle) *
        AP::ilmor()->get_max_rpm(), AP::ilmor()->get_min_rpm(),  AP::ilmor()->get_max_rpm());

    if (AP::ilmor()->get_trim_fn() > 0) // Only set trim if a valid servo function is set
    {
        _output.motor_trim = (uint8_t) ((SRV_Channels::get_output_norm(
            (SRV_Channel::Aux_servo_function_t)AP::ilmor()->get_trim_fn()) + 1.0) * 0.5 * 254.0);
    }

    _output.is_new = true;

#if AP_ILMOR_DEBUG
    static uint32_t last_send_ms = 0;
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_send_ms > 1000)
    {
        last_send_ms = now_ms;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Ilmor: RPM command %d, Trim command %u",
                      _output.motor_rpm, (unsigned)_output.motor_trim);
    }
#endif
}

void AP_Ilmor_Driver::loop()
{
    int16_t motor_rpm_cmd = 0;
    uint8_t trim_cmd = 255;

    while (true)
    {
        hal.scheduler->delay_microseconds(20000); // 50Hz

        const uint32_t now_ms = AP_HAL::millis();

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
        
        if (_current_trim_position > AP::ilmor()->get_max_run_trim())
        {
            // If the trim is above the maximum allowed trim, stop the motor
            if (motor_rpm_cmd > 0)
            {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Ilmor: Trim above max, stopping motor");
            }
            motor_rpm_cmd = 0;
        }

        ilmor_unmanned_throttle_control_t throttle_msg;
        // Set the magic number to allow unmanned control
        throttle_msg.unmanned_control_key = 
            ilmor_unmanned_throttle_control_unmanned_control_key_encode(0x4D);
        // Set the desired motor speed
        throttle_msg.unmanned_p_rpm_demand = 
            ilmor_unmanned_throttle_control_unmanned_p_rpm_demand_encode(motor_rpm_cmd);

        send_unmanned_throttle_control(throttle_msg);

#if AP_ILMOR_DEBUG
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Ilmor: Trim command %u, Current trim %u",
                      (unsigned)trim_cmd, (unsigned)_current_trim_position);
#endif

        ilmor_r3_status_frame_2_t r3_status_frame_2_msg;
        // Set the trim to physical button control (255)
        // this is the default state in case the AP loses control, the buttons will be active
        r3_status_frame_2_msg.trim_demand = ilmor_r3_status_frame_2_trim_demand_encode(255);

        // Operator must use buttons to lower the trim out of the upper limit,
        // and then release the buttons to allow the AP to take control of the trim
        if (_trim_command_from_buttons == 0 && trim_cmd < 255 && !_trim_locked_out) {
            // control the trim based on the commanded trim
            uint8_t deadband = 0;
            // don't use deadband for the full down position
            if (trim_cmd != 0) {
                deadband = TRIM_DEADBAND;
            }
            if (trim_cmd > _current_trim_position + deadband) {
                // Trim up
                r3_status_frame_2_msg.trim_demand = ilmor_r3_status_frame_2_trim_demand_encode(1);
            }
            else if (trim_cmd < _current_trim_position - deadband) {
                // Trim down
                r3_status_frame_2_msg.trim_demand = ilmor_r3_status_frame_2_trim_demand_encode(2);
            }
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
    uint8_t data[ILMOR_UNMANNED_THROTTLE_CONTROL_LENGTH];
    ilmor_unmanned_throttle_control_pack(data, &msg, sizeof(data));
    return AP_J1939_CAN::get_instance(AP::ilmor()->get_can_port())->
        enqueue_message(ILMOR_UNMANNED_THROTTLE_CONTROL_FRAME_ID, data, sizeof(data));
}


bool AP_Ilmor_Driver::send_r3_status_frame_2(const struct ilmor_r3_status_frame_2_t &msg)
{
    // Trim demand values: 0 = stop, 1 = up, 2 = down, 255 = trim control is given to the physical buttons
    uint8_t data[ILMOR_R3_STATUS_FRAME_2_LENGTH];
    ilmor_r3_status_frame_2_pack(data, &msg, sizeof(data));
    return AP_J1939_CAN::get_instance(AP::ilmor()->get_can_port())->
        enqueue_message(ILMOR_R3_STATUS_FRAME_2_FRAME_ID, data, sizeof(data));
}

void AP_Ilmor_Driver::handle_unmanned_throttle_control(const struct ilmor_unmanned_throttle_control_t &msg)
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Ilmor: Unmanned Throttle Control msg received from another CAN device");
}

void AP_Ilmor_Driver::handle_r3_status_frame_2(const struct ilmor_r3_status_frame_2_t &msg)
{
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Ilmor: R3 Status Frame 2 msg received from another CAN device");
}

void AP_Ilmor_Driver::handle_icu_status_frame_1(const struct ilmor_icu_status_frame_1_t &msg)
{
    _current_trim_position = msg.trim_position_adjusted;
}

void AP_Ilmor_Driver::handle_icu_status_frame_7(const struct ilmor_icu_status_frame_7_t &msg)
{
    _trim_command_from_buttons = msg.trim_demand_request_from_buttons;
    if (_trim_command_from_buttons != 0) {
        // Operator is using buttons to adjust trim
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Ilmor: Trim buttons pressed");
        if (_trim_command_from_buttons == 1 && _current_trim_position == 255) {
            if (!_trim_locked_out) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Ilmor: Trim locked up by buttons");
            }
            _trim_locked_out = true;
        }
        else if (_trim_command_from_buttons == 2 && _trim_locked_out) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Ilmor: Trim lock released");
            _trim_locked_out = false;
        }
    }
}

void AP_Ilmor_Driver::handle_inverter_status_frame_1(const struct ilmor_inverter_status_frame_1_t &msg)
{
    update_rpm(0, int32_t(msg.e_rpm / 5)); // Divide by 5 to get Prop RPM
    // Hack the motor current into the next ESC telemetry slot
    const TelemetryData t = {
        .current = float(msg.motor_current) / 10.0f,
    };
    update_telem_data(1, t,
                      AP_ESC_Telem_Backend::TelemetryType::CURRENT);
}

void AP_Ilmor_Driver::handle_inverter_status_frame_2(const struct ilmor_inverter_status_frame_2_t &msg)
{
    const TelemetryData t = {
        .consumption_mah = float((msg.ah_consumed) / 10.0f),
    };
    // Hack the current into the next+1 ESC telemetry slot
    update_telem_data(2, t,
                      AP_ESC_Telem_Backend::TelemetryType::CURRENT);

    // Put the current consumption into our ESC telemetry slot
    // Note that this is not available via Mavlink, hence the use of the hack above
    update_telem_data(0, t,
                      AP_ESC_Telem_Backend::TelemetryType::CONSUMPTION);
}

void AP_Ilmor_Driver::handle_inverter_status_frame_3(const struct ilmor_inverter_status_frame_3_t &msg)
{
    const TelemetryData t = {
        .voltage = float(msg.wh_consumed) / 10000.0f,
    };
    // Hack the Wh consumed into the next ESC telemetry slot
    update_telem_data(2, t,
                      AP_ESC_Telem_Backend::TelemetryType::VOLTAGE);
}

void AP_Ilmor_Driver::handle_inverter_status_frame_4(const struct ilmor_inverter_status_frame_4_t &msg)
{
    const TelemetryData t = {
        .temperature_cdeg = int16_t(msg.motor_temperature * 10),
        .current = float(msg.battery_current) / 10.0f,
    };
    update_telem_data(0, t,
                      AP_ESC_Telem_Backend::TelemetryType::CURRENT |
                          AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE);
    // Hack the MOSFET temperature into the next ESC telemetry slot 
    // Note that only .temperature_cdeg is available via Mavlink
    const TelemetryData t2 = {
        .temperature_cdeg = int16_t(msg.mosfet_temperature * 10),
    };
    update_telem_data(1, t2,
                      AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE);

#if AP_ILMOR_DEBUG
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Ilmor: MOSFET Temp %d, Motor Temp %d, Battery Current %f",
                      (int)msg.mosfet_temperature * 100, (int)msg.motor_temperature * 100, float(msg.battery_current));
#endif
}

void AP_Ilmor_Driver::handle_inverter_status_frame_5(const struct ilmor_inverter_status_frame_5_t &msg)
{
    const TelemetryData t = {
        .voltage = float(msg.low_precision_battery_voltage) / 10.0f,
    };
    update_telem_data(0, t,
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
