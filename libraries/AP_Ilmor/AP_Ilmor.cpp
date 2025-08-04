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

    Notes:
    1) Currently this class is a singleton for simplicity.

    Configuring CAN Parameters in ArduRover: First configure the CAN port to use the J1939 CAN backend
    (See AP_J1939_CAN/AP_J1939_CAN.cpp for more information)

    Settable Parameters - Description, Default value:
    ILMOR_MIN_RPM - Minimum RPM in reverse, -500
    ILMOR_MAX_RPM - Maximum RPM in forward, 1600
    ILMOR_RUN_TRIM - Maximum trim that the motor will be allowed to run at, 127
    ILMOR_TRIM_FN - Servo function that sets the target motor trim, Gripper (28)
    (see https://ardupilot.org/rover/docs/parameters.html#servo1-function-servo-output-function)
    ILMOR_CAN_PORT - CAN port to use (0-indexed), -1 (disabled)

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
#include <AP_HAL/utility/sparse-endian.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

#define SEND_TIMEOUT_US 1000

#define AP_ILMOR_DEBUG 0
#define AP_ILMOR_COMMAND_RATE_HZ 20
#define AP_ILMOR_TRIM_DEADBAND 10
#define AP_ILMOR_MAX_TRIM 190        // highest setting that AP will be allowed to trim

#define AP_ILMOR_SOURCE_ADDRESS 0xF2
#define AP_ILMOR_ICU_SOURCE_ADDRESS 0xEF

// J1939 Message priorities
#define AP_ILMOR_UNMANNED_THROTTLE_CONTROL_PRIORITY 1
#define AP_ILMOR_R3_STATUS_FRAME_2_PRIORITY 3
#define AP_ILMOR_R3_STATUS_FRAME_1_PRIORITY 3

// table of user settable CAN bus parameters
const AP_Param::GroupInfo AP_Ilmor::var_info[] = {

    // @Param: MIN_RPM
    // @DisplayName: Ilmor Motor Minimum RPM
    // @Description: Ilmor Motor Minimum RPM, by default this is set to -500
    // @Values: -2000:2000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MN_R", 1, AP_Ilmor, _min_rpm, 200),

    // @Param: MAX_RPM
    // @DisplayName: Ilmor Motor Maximum RPM
    // @Description: Ilmor Motor Maximum RPM, by default this is set to 1600
    // @Values: -2000:2000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MX_R", 2, AP_Ilmor, _max_rpm, 2000),

    // @Param: TRIM_FN
    // @DisplayName: Ilmor Motor Trim Servo Channel
    // @Description: Ilmor Motor Trim Servo Channel, by default this is set to "gripper" (28)
    // @Values: 0:255
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("TR_FN", 3, AP_Ilmor, _trim_fn, (int16_t)SRV_Channel::k_rcin6),

    // @Param: RUN_TRIM
    // @DisplayName: Ilmor Motor Run Trim
    // @Description: Ilmor Motor Maximum Trim that the motor will be allowed to run at
    // @Values: 0:127
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RN_TM", 4, AP_Ilmor, _max_run_trim, 127),

    // @Param: TRIM_STP
    // @DisplayName: Soft-stop Trim position
    // @Description: Soft-stop Trim position
    // @Values: 0:65000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("TM_ST", 6, AP_Ilmor, _trim_stop, 135),

    // @Param: FW_UP
    // @DisplayName: Ilmor Firmware Update Server
    // @Description: Ilmor Firmware Update Server, 0 = off, 1 = on, 2 = request to turn on
    // @Values: 0:2
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("FW_UP", 7, AP_Ilmor, _fw_update, 0),

    // @Param: CLR_FLT
    // @DisplayName: Clear Faults Request
    // @Description: Request to clear faults, 0 = ready, 1 = clear
    // @Values: 0:1
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("CLR_F", 8, AP_Ilmor, _clear_faults_request, 0),

    AP_GROUPEND};

AP_Ilmor::AP_Ilmor()
    : CANSensor("Ilmor"),
    _trimState(TrimState::Start),
    _motor_state(MotorState::Ready),
    _comsState(ComsState::Unhealthy),
    _fw_server_state(FwServerState::WifiOff),
    _run_state(),
    _output()
{
    _num_active_faults = 0;
    _num_tp_packets = 0;
    _output.motor_rpm = 0;
    _output.motor_trim = AP_Ilmor::TRIM_CMD_STOP;

    AP_Param::setup_object_defaults(this, var_info);

    // register_driver(AP_CAN::Protocol::Ilmor);

}

AP_Ilmor *AP_Ilmor::get_ilmor(uint8_t driver_index)
{
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_driver_type(driver_index) != AP_CAN::Protocol::Ilmor) {
        return nullptr;
    }
    return static_cast<AP_Ilmor *>(AP::can().get_driver(driver_index));
}

void AP_Ilmor::init(uint8_t driver_index, bool enable_filters)
{
    CANSensor::init(driver_index, enable_filters);

    hal.util->snprintf(_thread_name, sizeof(_thread_name), "ilmor%d_tx", driver_index);
    hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Ilmor::run_io, void), _thread_name, 1024, AP_HAL::Scheduler::PRIORITY_TIMER, 0);
}

bool AP_Ilmor::pre_arm_check(char *failure_msg, uint8_t failure_msg_len)
{

    if (!healthy()) {
        strncpy(failure_msg, "not healthy", failure_msg_len);
        return false;
    }
    return true;
}

void AP_Ilmor::send_throttle_cmd()
{
    const uint8_t trim_upper_limit = _trim_stop.get() < 0 ? 255 : _trim_stop.get();

    if (AP_HAL::millis() - _run_state.last_send_throttle_ms >= 1000 / AP_ILMOR_COMMAND_RATE_HZ) {
        ilmor_unmanned_throttle_control_t throttle_msg = {
            .unmanned_control_key = 0x4d,
            .unmanned_p_rpm_demand = _output.motor_rpm,
            .trim_custom_upper_limit = trim_upper_limit,
        };
        
        if (!send_unmanned_throttle_control(throttle_msg)) {
            // GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Failed to send throttle control message");
        }

        _run_state.last_send_throttle_ms = AP_HAL::millis();
    }
}

void AP_Ilmor::send_trim_cmd()
{
    if (AP_HAL::millis() - _run_state.last_send_trim_ms >= 1000 / AP_ILMOR_COMMAND_RATE_HZ) {

        ilmor_r3_status_frame_2_t r3_status_frame_2_msg = {
            .trim_demand_request_from_r3 = static_cast<uint8_t>(_output.motor_trim),
        };

        if (!send_r3_status_frame_2(r3_status_frame_2_msg)) {
            // GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Failed to send trim control message");
        }

        _run_state.last_send_trim_ms = AP_HAL::millis();
    }
}

void AP_Ilmor::run_io()
{
    while (true) {
        tick();
        hal.scheduler->delay(2);
    }
}

// called periodically from the IO thread
void AP_Ilmor::tick()
{
    trim_state_machine();
    coms_state_machine();
    fw_server_state_machine();
    clear_faults_state_machine();

    if (AP_HAL::millis() - _last_print_faults_ms >= 1000) {
        report_faults();
        _last_print_faults_ms = AP_HAL::millis();
    }

}

// parse inbound frames
void AP_Ilmor::handle_frame(AP_HAL::CANFrame &frame)
{

    bool is_from_icu = false;

    switch (frame.id) {
        case ILMOR_INVERTER_STATUS_FRAME_1_FRAME_ID: {
            struct ilmor_inverter_status_frame_1_t msg;
            ilmor_inverter_status_frame_1_unpack(&msg, frame.data, frame.dlc);
            handle_inverter_status_frame_1(msg);
        } break;

        case ILMOR_INVERTER_STATUS_FRAME_2_FRAME_ID: {
            // Ah consumed
            struct ilmor_inverter_status_frame_2_t msg;
            ilmor_inverter_status_frame_2_unpack(&msg, frame.data, frame.dlc);
            handle_inverter_status_frame_2(msg);
            
        } break;

        case ILMOR_INVERTER_STATUS_FRAME_3_FRAME_ID: {
            // Wh consumed
            struct ilmor_inverter_status_frame_3_t msg;
            ilmor_inverter_status_frame_3_unpack(&msg, frame.data, frame.dlc);
            handle_inverter_status_frame_3(msg);
            
        } break;

        case ILMOR_INVERTER_STATUS_FRAME_4_FRAME_ID: {
            struct ilmor_inverter_status_frame_4_t msg;
            ilmor_inverter_status_frame_4_unpack(&msg, frame.data, frame.dlc);
            handle_inverter_status_frame_4(msg);
        } break;

        case ILMOR_INVERTER_STATUS_FRAME_5_FRAME_ID: {
            struct ilmor_inverter_status_frame_5_t msg;
            ilmor_inverter_status_frame_5_unpack(&msg, frame.data, frame.dlc);
            handle_inverter_status_frame_5(msg);
        } break;

        default: {
            const J1939::Id id(frame.id);
            const uint32_t pgn = id.pgn_raw();

            is_from_icu = (id.source_address() == AP_ILMOR_ICU_SOURCE_ADDRESS);
            if (!is_from_icu) {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Ilmor: pgn: %" PRIu32 " from unexpected sa: %" PRIu8, pgn, id.source_address());
                break;
            }

            switch (pgn) {
                case 0xff01: {
                    struct ilmor_icu_status_frame_1_t msg;
                    if (ilmor_icu_status_frame_1_unpack(&msg, frame.data, frame.dlc) == 0) {
                        handle_icu_status_frame_1(msg);
                    }
                } break;

                case 0xff02: {
                    struct ilmor_icu_status_frame_2_t msg;
                    if (ilmor_icu_status_frame_2_unpack(&msg, frame.data, frame.dlc) == 0) {
                        handle_icu_status_frame_2(msg);
                    }
                } break;

                case J1939_PGN_DM1: {
                    J1939::DiagnosticMessage1::DTC dtc = J1939::DiagnosticMessage1::DTC::from_data(&frame.data[2]);
                    _active_faults[0] = dtc;
                    _num_active_faults = 1;
                    report_faults();

                } break;

                case J1939_PGN_TP_CM: {
                    const J1939::PGN tp_pgn((frame.data[5] << 8) | frame.data[4]);
                    if (frame.data[0] == 0x20 && tp_pgn.type() == J1939::PGNType::DiagnosticMessage1) {
                        _num_tp_packets = frame.data[3];
                    }

                } break;

                case J1939_PGN_TP_DT: {
                    _num_tp_packets--;
                    J1939::DiagnosticMessage1::DTC dtc = J1939::DiagnosticMessage1::DTC::from_data(&frame.data[2]);
                    active_fault(dtc);
                    report_faults();

                } break;

                default:
                    break;
            }

        } break;
    }

    if (is_from_icu) {
        // If the message is from the ICU, we update the last received message time
        _run_state.last_received_msg_ms = AP_HAL::millis();
    }

}

bool AP_Ilmor::soft_stop_exceeded()
{
    const int16_t max_trim = _trim_stop.get();
    if ( max_trim > 0 && 10 < _current_trim_position && _current_trim_position < 254 && _current_trim_position > max_trim + 10 ) {
        return true;
    } else {
        return false;
    }
}

AP_Ilmor::TrimCmd AP_Ilmor::trim_demand()
{
    AP_Ilmor::TrimCmd retval;
    const float trim_servo = SRV_Channels::get_output_norm((SRV_Channel::Aux_servo_function_t)_trim_fn.get());
    if (trim_servo < -0.3f) {
        retval = AP_Ilmor::TRIM_CMD_DOWN;
    } else if (trim_servo > 0.3f) {
        retval = AP_Ilmor::TRIM_CMD_UP;
    } else {
        retval = AP_Ilmor::TRIM_CMD_BUTTONS;
    }
    return retval;
}

void AP_Ilmor::trim_state_machine()
{

    if (!hal.util->get_soft_armed()) {
        _output.motor_trim = AP_Ilmor::TRIM_CMD_STOP;
        return;
    }

    switch (_trimState) {
        case TrimState::Start:
            if (_trim_fn.get() > 0) {
                _trimState = TrimState::CheckSoftStop;
            } else {
                _output.motor_trim = TrimCmd::TRIM_CMD_BUTTONS;
            }
            break;

        case TrimState::CheckSoftStop:
        {
            if (soft_stop_exceeded()) {
                _trimState = TrimState::CmdDown;
                _last_trim_wait_ms = AP_HAL::millis();
            } else {
                _trimState = TrimState::Manual;
            }

        } break;
        
        case TrimState::Manual:
        {
            _output.motor_trim = trim_demand();
            _trimState = TrimState::CheckSoftStop;

        } break;

        case TrimState::CheckRelease:
        {
            if (soft_stop_exceeded()) {
                _trimState = TrimState::CmdDown;
                _last_trim_wait_ms = AP_HAL::millis();
            } else if (trim_demand() != TrimCmd::TRIM_CMD_UP) {
                _trimState = TrimState::CheckSoftStop;
            }

        } break;

        case TrimState::CmdDown:
        {
            _output.motor_trim = AP_Ilmor::TRIM_CMD_DOWN;
            const uint32_t now = AP_HAL::millis();
            if (now - _last_trim_wait_ms > 125) {
                _trimState = TrimState::CmdStop;
                _last_trim_wait_ms = now;
            }

        } break;

        case TrimState::CmdStop:
        {
            _output.motor_trim = AP_Ilmor::TRIM_CMD_BUTTONS;
            const uint32_t now = AP_HAL::millis();
            if (now - _last_trim_wait_ms > 1000) {
                _trimState = TrimState::CheckRelease;
            }
        }
    }



}

void AP_Ilmor::coms_state_machine()
{

    const uint32_t now_ms = AP_HAL::millis();
    switch (_comsState) {
        case ComsState::Running:
            if (healthy()) {
                send_throttle_cmd();
                send_trim_cmd();
            } else {
                _comsState = ComsState::Unhealthy;
            }
            break;

        case ComsState::Unhealthy:
            if (healthy()) {
                // start a 1 second timer to wait
                _comsState = ComsState::Waiting;
                _last_com_wait_ms = AP_HAL::millis();
            }
            break;
        
        case ComsState::Waiting:
            if (now_ms - _last_com_wait_ms > 3000) {
                _comsState = ComsState::Running;
            }
            break;
    }
    
}

void AP_Ilmor::fw_server_state_machine()
{
    switch (_fw_server_state) {
        case FwServerState::WifiOff:
            if (_fw_update.get() == 2) {
                // send request to turn on the firmware update server
                _fw_server_state = FwServerState::WifiOn;

                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Ilmor: Requesting firmware update server to turn on");

                ilmor_r3_status_frame_1_t msg = {
                    .server_mode = 2,
                };

                if (!send_r3_status_frame_1(msg)) {
                    // GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Failed to send trim control message");
                }

            }
            break;
        case FwServerState::WifiOn:
            if (_fw_update.get() != 2) {
                // send request to turn off firmware update server
                _fw_server_state = FwServerState::WifiOff;

                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Ilmor: Requesting firmware update server to turn off");

                ilmor_r3_status_frame_1_t msg = {
                    .server_mode = 1,
                };

                if (!send_r3_status_frame_1(msg)) {
                    // GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Failed to send trim control message");
                }
            }
            break;
    }

}

void AP_Ilmor::clear_faults_state_machine()
{
    switch (_clear_faults_state) {
        case ClearFaultsState::Ready:
            if (_clear_faults_request.get() == 1) {
                // send request to clear faults
                _clear_faults_state = ClearFaultsState::Cleared;

                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Ilmor: Requesting to clear faults");

                // Send a J1939 DM11 message to clear faults

                AP_HAL::CANFrame can_frame;
                can_frame.id = 0x00FED3F2;
                can_frame.id |= AP_HAL::CANFrame::FlagEFF;
                can_frame.dlc = 8; // J1939 frames are always 8 bytes
                can_frame.canfd = false; // J1939 does not support CAN FD (yet)
                memset(can_frame.data, 0xff, sizeof(can_frame.data));
                write_frame(can_frame, SEND_TIMEOUT_US);

                _num_active_faults = 0;

            }
            break;

        case ClearFaultsState::Cleared:
            if (_clear_faults_request.get() != 1) {
                _clear_faults_state = ClearFaultsState::Ready;
            }
            break;
    }
}

// update the output from the throttle servo channel
void AP_Ilmor::update()
{
    const uint32_t now_ms = AP_HAL::millis();

    // Check if the throttle is armed
    if (!hal.util->get_soft_armed()) {
        _output.motor_rpm = 0;
        return;
    }

    if (_max_run_trim.get() > 0 && _current_trim_position > _max_run_trim.get()) {
        // If the trim position is above the maximum run trim, we need to stop the motor
        _output.motor_rpm = 0;
        return;
    }

    const float throttle = constrain_float(SRV_Channels::get_output_norm(SRV_Channel::k_throttle), -1.0, 1.0);
    const int16_t command_rpm = throttle * _max_rpm.get();
    const int16_t min_rpm = abs(_min_rpm.get());

    switch (_motor_state) {

        case MotorState::Ready: {
            if (command_rpm > min_rpm) {
                _motor_state = MotorState::Forward;
                _last_motor_wait_ms = now_ms;
            } else if (command_rpm < -min_rpm) {
                _motor_state = MotorState::Reverse;
                _last_motor_wait_ms = now_ms;
            } else {
                _output.motor_rpm = 0;
            }

        } break;

        case MotorState::Stop: {
            _output.motor_rpm = 0;
            if (now_ms - _last_motor_wait_ms > 3000) {
                _motor_state = MotorState::Ready;
            }
        } break;

        case MotorState::Forward: {
            if (command_rpm > min_rpm) {
                _output.motor_rpm = command_rpm;

                if (now_ms - _last_motor_wait_ms > 5000) {
                    _last_motor_wait_ms = now_ms;
                    if (abs(_last_rpm) < min_rpm) {
                        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Ilmor: Zero RPM detected");
                        _motor_state = MotorState::Error;
                    }
                }

            } else {
                _motor_state = MotorState::Stop;
                _last_motor_wait_ms = now_ms;
            }
        
        } break;

        case MotorState::Reverse: {
            if (command_rpm < -min_rpm) {
                _output.motor_rpm = command_rpm;

                if (now_ms - _last_motor_wait_ms > 5000) {
                    _last_motor_wait_ms = now_ms;
                    if (abs(_last_rpm) < min_rpm) {
                        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Ilmor: Zero RPM detected");
                        _motor_state = MotorState::Error;
                    }
                }

            } else {
                _motor_state = MotorState::Stop;
                _last_motor_wait_ms = now_ms;
            }
        
        } break;

        case MotorState::Error: {
            _output.motor_rpm = 0;
            if (now_ms - _last_motor_wait_ms > 10000) {
                _motor_state = MotorState::Ready;
            }
        } break;
    }

}

void AP_Ilmor::active_fault(J1939::DiagnosticMessage1::DTC& dtc)
{
    // check if the fault is already active and update it if so
    for (int i = 0; i < _num_active_faults; i++) {
        if (_active_faults[i].spn() == dtc.spn()) {
            _active_faults[i].set_fmi(dtc.fmi());
            _active_faults[i].set_oc(dtc.oc());
            return;
        }
    }

    if (_num_active_faults < AP_ILMOR_MAX_FAULTS) {
        _active_faults[_num_active_faults++] = dtc;
    }

}

void AP_Ilmor::report_faults()
{
    // send a GCS_SEND_TEXT for each active fault
    for (int i = 0; i < _num_active_faults; i++) {
        const J1939::DiagnosticMessage1::DTC &dtc = _active_faults[i];
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Ilmor: SPN: %" PRIu32 " FMI: %" PRIu8 " oc: %" PRIu8, dtc.spn(), dtc.fmi(), dtc.oc());
    }
}

bool AP_Ilmor::healthy() const
{
    // Check if we have received any messages in the last second
    const uint32_t now_ms = AP_HAL::millis();
    if (_run_state.last_received_msg_ms == 0 || now_ms - _run_state.last_received_msg_ms > 1000) {
        return false;
    }
    return true;
}

bool AP_Ilmor::send_unmanned_throttle_control(const struct ilmor_unmanned_throttle_control_t &msg)
{
    // Prepare the data
    uint8_t data[ILMOR_UNMANNED_THROTTLE_CONTROL_LENGTH];
    ilmor_unmanned_throttle_control_pack(data, &msg, sizeof(data));

    // Even though the frame ID contains the priority and source address,
    // we set them explicitly here for clarity
    J1939::J1939Frame frame;
    frame.priority = AP_ILMOR_UNMANNED_THROTTLE_CONTROL_PRIORITY;
    frame.pgn = J1939::extract_j1939_pgn(ILMOR_UNMANNED_THROTTLE_CONTROL_FRAME_ID);
    frame.source_address = AP_ILMOR_SOURCE_ADDRESS;
    memcpy(frame.data, data, sizeof(data));

    AP_HAL::CANFrame can_frame = J1939::pack_j1939_frame(frame);
    return write_frame(can_frame, SEND_TIMEOUT_US);
}

bool AP_Ilmor::send_r3_status_frame_1(const struct ilmor_r3_status_frame_1_t &msg)
{
    uint8_t data[ILMOR_R3_STATUS_FRAME_1_LENGTH];
    ilmor_r3_status_frame_1_pack(data, &msg, sizeof(data));

    J1939::J1939Frame frame;
    frame.priority = AP_ILMOR_R3_STATUS_FRAME_1_PRIORITY;
    frame.pgn = J1939::extract_j1939_pgn(ILMOR_R3_STATUS_FRAME_1_FRAME_ID);
    frame.source_address = AP_ILMOR_SOURCE_ADDRESS;
    memcpy(frame.data, data, sizeof(data));

    AP_HAL::CANFrame can_frame = J1939::pack_j1939_frame(frame);
    return write_frame(can_frame, SEND_TIMEOUT_US);
}

bool AP_Ilmor::send_r3_status_frame_2(const struct ilmor_r3_status_frame_2_t &msg)
{
    uint8_t data[ILMOR_R3_STATUS_FRAME_2_LENGTH];
    ilmor_r3_status_frame_2_pack(data, &msg, sizeof(data));

    J1939::J1939Frame frame;
    frame.priority = AP_ILMOR_R3_STATUS_FRAME_2_PRIORITY;
    frame.pgn = J1939::extract_j1939_pgn(ILMOR_R3_STATUS_FRAME_2_FRAME_ID);
    frame.source_address = AP_ILMOR_SOURCE_ADDRESS;
    memcpy(frame.data, data, sizeof(data));

    AP_HAL::CANFrame can_frame = J1939::pack_j1939_frame(frame);
    return write_frame(can_frame, SEND_TIMEOUT_US);
}

void AP_Ilmor::handle_unmanned_throttle_control(const struct ilmor_unmanned_throttle_control_t &msg)
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Ilmor: Unmanned Throttle Control msg received from another CAN device");
}

void AP_Ilmor::handle_r3_status_frame_2(const struct ilmor_r3_status_frame_2_t &msg)
{
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Ilmor: R3 Status Frame 2 msg received from another CAN device");
}

void AP_Ilmor::handle_icu_status_frame_1(const struct ilmor_icu_status_frame_1_t &msg)
{
    _current_trim_position = msg.trim_position_adjusted;
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Ilmor: Trim position %d", msg.trim_position);
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Ilmor: Trim adj %d", msg.trim_position_adjusted);

    // Populate esc2_rpm with the trim position
    update_rpm(1, msg.trim_position_adjusted);
}

void AP_Ilmor::handle_icu_status_frame_2(const struct ilmor_icu_status_frame_2_t &msg)
{
    static uint16_t counter = 0;
    if (counter++ % 100 == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Ilmor: t: %" PRIi32 " s:%d v%d.%d.%d", msg.throttle_demand, msg.shift_position,
            msg.software_version_major, msg.software_version_minor, msg.software_version_patch);
    }
    
    
}

void AP_Ilmor::handle_icu_status_frame_7(const struct ilmor_icu_status_frame_7_t &msg)
{
    
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Ilmor: trim demand %d", msg.trim_demand_request_from_buttons);
    // _trim_command_from_buttons = msg.trim_demand_request_from_buttons;
    // if (_trim_command_from_buttons != 0)
    // {
    //     // Operator is using buttons to adjust trim
    //     GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Ilmor: Trim buttons pressed");
    //     if (_trim_command_from_buttons == 1 && _current_trim_position >= AP_ILMOR_MAX_TRIM)
    //     {
    //         if (!_trim_locked_out)
    //         {
    //             GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Ilmor: Trim locked up by buttons");
    //         }
    //         _trim_locked_out = true;
    //     }
    //     else if (_trim_command_from_buttons == 2 && _trim_locked_out)
    //     {
    //         GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Ilmor: Trim lock released");
    //         _trim_locked_out = false;
    //     }
    // }
}

void AP_Ilmor::handle_inverter_status_frame_1(const struct ilmor_inverter_status_frame_1_t &msg)
{
    // Divide by 5 to get Prop RPM
    _last_rpm = msg.e_rpm / 5;
    update_rpm(0, int32_t(_last_rpm));
    // Hack the motor current into the next ESC telemetry slot
    const TelemetryData t = {
        .current = float(msg.motor_current) / 10.0f,
    };
    update_telem_data(1, t,
                      AP_ESC_Telem_Backend::TelemetryType::CURRENT);
}

void AP_Ilmor::handle_inverter_status_frame_2(const struct ilmor_inverter_status_frame_2_t &msg)
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

void AP_Ilmor::handle_inverter_status_frame_3(const struct ilmor_inverter_status_frame_3_t &msg)
{
    const TelemetryData t = {
        .voltage = float(msg.wh_consumed) / 10000.0f,
    };
    // Hack the Wh consumed into the next ESC telemetry slot
    update_telem_data(2, t,
                      AP_ESC_Telem_Backend::TelemetryType::VOLTAGE);
}

void AP_Ilmor::handle_inverter_status_frame_4(const struct ilmor_inverter_status_frame_4_t &msg)
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

void AP_Ilmor::handle_inverter_status_frame_5(const struct ilmor_inverter_status_frame_5_t &msg)
{
    const TelemetryData t = {
        .voltage = float(msg.low_precision_battery_voltage) / 10.0f,
    };
    update_telem_data(0, t,
                      AP_ESC_Telem_Backend::TelemetryType::VOLTAGE);
}


#endif // HAL_ILMOR_ENABLED
