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

#include "AP_Torqeedo_TQBus.h"

#if HAL_TORQEEDO_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>
#include <AP_HAL/utility/sparse-endian.h>

// #define DEBUG_TORQEEDO 1

#define TORQEEDO_SERIAL_BAUD        19200   // communication is always at 19200
#define TORQEEDO_PARITY             0       // communication is always no parity
#define TORQEEDO_STOP_BITS          1       // communication is always 1 stop bit
#define TORQEEDO_PACKET_HEADER      0xAC    // communication packet header
#define TORQEEDO_PACKET_FOOTER      0xAD    // communication packet footer
#define TORQEEDO_PACKET_ESCAPE      0xAE    // escape character for handling occurrences of header, footer and this escape bytes in original message
#define TORQEEDO_PACKET_ESCAPE_MASK 0x80    // byte after ESCAPE character should be XOR'd with this value

#define TORQEEDO_MESSAGE_LEN_MAX    35  // messages are no more than 35 bytes

#define MOTOR_STATUS_RUNNING (1 << 3)

enum class MsgAddress : uint8_t {
    BUS_MASTER = 0x00,
    REMOTE1 = 0x14,
    DISPLAY = 0x20,
    MOTOR = 0x30,
    BATTERY = 0x80
};

enum class RemoteMsgId : uint8_t {
    INFO = 0x00,
    REMOTE = 0x01,
    SETUP = 0x02
};

// Display specific message ids
enum class DisplayMsgId : uint8_t {
    INFO = 0x00,
    SYSTEM_STATE = 0x41,
    SYSTEM_SETUP = 0x42
};

enum class MotorMsgId : uint8_t {
    INFO = 0x00,
    STATUS = 0x01,
    DRIVE = 0x82,
    PARAM = 0x03,
};

extern const AP_HAL::HAL& hal;

#define SOFTWARE_FLOWCONTROL 1

#ifdef SOFTWARE_FLOWCONTROL
static uint32_t calc_transmit_time_us(uint8_t num_bytes)
{
    // baud rate of 19200 bits/sec
    // total number of bits = num_bytes * (1 start_bit + 8 data_bits + num_parity_bits + 1 stop_bit)
    // convert from seconds to micros by multiplying by 1,000,000
    // plus additional 300us safety margin
    uint8_t bits_per_data_byte = 10 + TORQEEDO_PARITY;
    const uint32_t delay_us = 1e6 * num_bytes * bits_per_data_byte / TORQEEDO_SERIAL_BAUD + 300;
    return delay_us;
}
#endif


static void configure_uart(AP_HAL::UARTDriver* uart)
{
    uart->end();
    uart->configure_parity(0); // no parity
    uart->set_stop_bits(1);


#ifdef SOFTWARE_FLOWCONTROL
    uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    uart->set_unbuffered_writes(true);
    uart->set_CTS_pin(false);
#else
    uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_RTS_DE);
#endif // SOFTWARE_FLOWCONTROL

    uart->set_options(uart->get_options()
                     | AP_HAL::UARTDriver::OPTION_NODMA_RX
                    // | AP_HAL::UARTDriver::OPTION_NOFIFO
                );

    uart->begin(TORQEEDO_SERIAL_BAUD, 16, 16);
    uart->discard_input();
}

static bool send_message(uint8_t* pkt_body, uint8_t len, AP_HAL::UARTDriver* uart)
{
#ifdef SOFTWARE_FLOWCONTROL
    uart->set_CTS_pin(true);
#endif // SOFTWARE_FLOWCONTROL

    uint8_t num_bytes_written = 0;
    if (uart->write(TORQEEDO_PACKET_HEADER) != 1) {
        return false;
    }
    num_bytes_written++;

    const uint8_t crc = crc8_maxim(pkt_body, len);

    for (int i=0;i<len;i++) {
        if (pkt_body[i] == TORQEEDO_PACKET_HEADER || pkt_body[i] == TORQEEDO_PACKET_FOOTER || pkt_body[i] == TORQEEDO_PACKET_ESCAPE) {
            // escape the byte
            if (uart->write(TORQEEDO_PACKET_ESCAPE) != 1) {
                return false; // write failed
            }
            
            // XOR with escape mask
            if (uart->write(pkt_body[i] ^ TORQEEDO_PACKET_ESCAPE_MASK) != 1) {
                return false; // write failed
            }

            num_bytes_written += 2; // one for escape byte, one for the escaped byte
        } else {
            // write the byte as is
            if (uart->write(pkt_body[i]) != 1) {
                return false; // write failed
            }
            num_bytes_written++;
        }
    }

    if (uart->write(crc) != 1) {
        return false; // write failed
    }

    if (uart->write(TORQEEDO_PACKET_FOOTER) != 1) {
        return false; // write failed
    }

    // add padding byte
    if (uart->write(0xFF) != 1) {
        return false; // write failed
    }

    num_bytes_written += 3; // one for CRC byte, one for footer byte, one for padding byte

    uart->flush();

#ifdef SOFTWARE_FLOWCONTROL
    // wait for the transmission to complete
    hal.scheduler->delay_microseconds(calc_transmit_time_us(num_bytes_written));
    uart->set_CTS_pin(false);
#endif // SOFTWARE_FLOWCONTROL

    return true;
}

class TQBusRxFrame
{
public:
    TQBusRxFrame();

    bool push_byte(uint8_t b);
    void reset() {
        _len = 0;
        _state = WAITING_FOR_HEADER;
    }

    const uint8_t* get_buffer() const { return _buff; }
    uint8_t get_length() const { return _len; }

private:
    uint8_t _buff[TORQEEDO_MESSAGE_LEN_MAX];
    uint8_t _len;

    enum {
        WAITING_FOR_HEADER = 0,
        WAITING_FOR_FOOTER,
        WAITING_FOR_UNESCAPED_BYTE,
        PACKET_RECEIVED,
    } _state;

};

TQBusRxFrame::TQBusRxFrame()
  : _len(0), _state(WAITING_FOR_HEADER)
{}

bool TQBusRxFrame::push_byte(uint8_t b)
{
    switch (_state) {
        case WAITING_FOR_HEADER:
            if (b == TORQEEDO_PACKET_HEADER) {
                _len = 0;
                _state = WAITING_FOR_FOOTER;
            }
            break;

        case WAITING_FOR_FOOTER:
            if (b == TORQEEDO_PACKET_FOOTER) {
                // check CRC
                const uint8_t calculated_crc = crc8_maxim(_buff, _len-1);
                if (calculated_crc == _buff[_len-1]) {
                    // valid packet received
                    _len -= 1; // remove CRC byte from length
                    _state = PACKET_RECEIVED; // set state to indicate packet is received
                    return true;
                } else {
                    // invalid packet, reset state
                    _len = 0;
                    _state = WAITING_FOR_HEADER;
                }

            } else if (b == TORQEEDO_PACKET_ESCAPE) {
                _state = WAITING_FOR_UNESCAPED_BYTE;

            } else {
                // store the byte
                if (_len < TORQEEDO_MESSAGE_LEN_MAX) {
                    _buff[_len++] = b;
                } else {
                    // buffer overflow, reset state
                    _len = 0;
                    _state = WAITING_FOR_HEADER;
                }
            }
            break;

        case WAITING_FOR_UNESCAPED_BYTE:
            if (_len < TORQEEDO_MESSAGE_LEN_MAX) {
                _buff[_len++] = b ^ TORQEEDO_PACKET_ESCAPE_MASK;
                _state = WAITING_FOR_FOOTER;
            } else {
                // buffer overflow, reset state
                _len = 0;
                _state = WAITING_FOR_HEADER;
            }
            break;

        case PACKET_RECEIVED:
            // packet already received, ignore any further bytes until next header
            break;
            
    }


    return false;
}



AP_Torqeedo_TQBus::AP_Torqeedo_TQBus(AP_Torqeedo_Params &params, uint8_t instance)
  : AP_Torqeedo_Backend(params, instance),
  _uart(nullptr),
  _state(DriverState::Init)
{}

// initialise driver
void AP_Torqeedo_TQBus::init()
{
    // create background thread to process serial input and output
    char thread_name[15];
    hal.util->snprintf(thread_name, sizeof(thread_name), "torqeedo%u", (unsigned)_instance);
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Torqeedo_TQBus::thread_main, void), thread_name, 1024, AP_HAL::Scheduler::PRIORITY_IO, 0)) {
        return;
    }
}

void AP_Torqeedo_TQBus::send_mavlink_status(mavlink_channel_t ch)
{
    mavlink_msg_torqeedo_telemetry_send_struct(ch, &_torqeedo_telemetry);
}

// returns true if communicating with the motor
bool AP_Torqeedo_TQBus::healthy()
{
    // if the last received time is more than 2 seconds ago, we consider the connection unhealthy
    if (AP_HAL::millis() - _last_rx_ms > 5000) {
        return false;
    } else {
        return true;
    }
    
}

void AP_Torqeedo_TQBus::filter_desired_speed()
{
    #define SLEW_FILTER_CUTOFF_FREQ 1.0f // cutoff frequency for the lowpass filter in Hz

    const uint32_t now_ms = AP_HAL::millis();
    const float dt = (now_ms - _last_set_rpm_ms) / 1000.0f; // time since last state change in seconds
    _last_set_rpm_ms = now_ms; // update last set RPM time

    float rpm_cmd = SRV_Channels::get_output_norm((SRV_Channel::Aux_servo_function_t)_params.servo_fn.get()) * 1000.0f;
    _filtered_desired_speed += calc_lowpass_alpha_dt(dt, SLEW_FILTER_CUTOFF_FREQ) * (rpm_cmd - _filtered_desired_speed);
}

// consume incoming messages from motor, reply with latest motor speed
// runs in background thread
void AP_Torqeedo_TQBus::thread_main()
{

    const AP_SerialManager &serial_manager = AP::serialmanager();
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Torqeedo, _instance);
    if (_uart == nullptr) {
        return;
    }
    configure_uart(_uart);

    TQBusRxFrame rx_frame;

    while (true) {

        _torqeedo_telemetry.other = (0x0F & static_cast<uint8_t>(_state)) |
                                    (healthy() ? 0x10 : 0x00)
                                    ;


#ifdef DEBUG_TORQEEDO
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TQBus: state %d",
                      static_cast<uint8_t>(_state));
#endif // DEBUG_TORQEEDO

        hal.scheduler->delay(2);

        // read bytes from UART
        uint8_t b;
        while (_uart->read(b)) {
            if (rx_frame.push_byte(b)) {
                // complete frame received
                process_rx_frame(rx_frame.get_buffer(), rx_frame.get_length());
                rx_frame.reset();
                _last_rx_ms = AP_HAL::millis(); // update last received time
            }
        }

        filter_desired_speed();

        const uint32_t now_ms = AP_HAL::millis();
        if (now_ms - _last_master_error_code_set_ms > 60000) {
            // reset master error code after 60 seconds
            _torqeedo_telemetry.master_error_code = 0;
        }

        switch (_comsState) {
            case ComsState::Healthy: {
                if (!healthy()) {
                    _comsState = ComsState::Unhealthy;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Torqeedo: lost comms");
                }

            } break;

            case ComsState::Unhealthy: {
                if (healthy()) {
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Torqeedo: comms restored");
                    _comsState = ComsState::Healthy;
                } else if (now_ms - _last_rx_ms > 20000) {

                    switch (_state) {
                        case DriverState::Init:
                        case DriverState::Stop:
                        case DriverState::Ready:
                        case DriverState::Forward:
                        case DriverState::Reverse:
                            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Torqeedo: comms lost, resetting driver state");
                            _state = DriverState::PowerOn;
                            _last_state_change_ms = now_ms;
                            break;

                        default:
                            break;
                    }
                }
            } break;
        }

        #define MIN_THROTTLE 30 // minimum throttle to consider the motor running

        switch (_state) {
            case DriverState::Init: {

                if (!healthy() && now_ms - _last_state_change_ms > 20000) {
                    _state = DriverState::PowerOn;
                    _last_state_change_ms = now_ms;
                }

                if (hal.util->get_soft_armed()) {
                    _state = DriverState::Stop;
                    _last_state_change_ms = now_ms;
                }

            } break;

            case DriverState::PowerOn: {
                _motor_speed_desired = 0;
                _uart->set_RTS_pin(true);

                if (now_ms - _last_state_change_ms > 6000) {
                    _last_state_change_ms = now_ms;
                    _state = DriverState::PowerOff; // go to PowerOff state after 3 seconds
                }

            } break;

            case DriverState::PowerOff: {
                _uart->set_RTS_pin(false);

                if (now_ms - _last_state_change_ms > 3000) {
                    _state = DriverState::Init; // go back to Init state
                    _last_state_change_ms = now_ms;
                }
            } break;

            case DriverState::Stop: {
                _motor_speed_desired = 0;
                if (now_ms - _last_state_change_ms > 3000) {
                    _last_state_change_ms = now_ms;
                    _state = DriverState::Ready;
                }

            } break;

            case DriverState::Ready: {
                if (_filtered_desired_speed > MIN_THROTTLE) {
                    _state = DriverState::Forward;
                    _last_state_change_ms = now_ms;
                } else if (_filtered_desired_speed < -MIN_THROTTLE) {
                    _state = DriverState::Reverse;
                    _last_state_change_ms = now_ms;
                }
                
            } break;

            case DriverState::Forward: {
                if (_filtered_desired_speed > MIN_THROTTLE) {
                    _motor_speed_desired = _filtered_desired_speed;

                    if (now_ms - _last_state_change_ms > 10000) {
                        _last_state_change_ms = now_ms;
                        if (_motor_rpm < 20 || (_torqeedo_telemetry.motor_status & MOTOR_STATUS_RUNNING) == 0) {
                            set_master_error_code(240);
                        }
                    }

                } else {
                    _state = DriverState::Stop;
                    _last_state_change_ms = now_ms;
                }

            } break;

            case DriverState::Reverse: {
                if (_filtered_desired_speed < -MIN_THROTTLE) {
                    _motor_speed_desired = _filtered_desired_speed;

                    if (now_ms - _last_state_change_ms > 10000) {
                        _last_state_change_ms = now_ms;
                        if (_motor_rpm > -20 || (_torqeedo_telemetry.motor_status & MOTOR_STATUS_RUNNING) == 0) {
                            set_master_error_code(240);
                        }
                    }

                } else {
                    _state = DriverState::Stop;
                    _last_state_change_ms = now_ms;
                }

            } break;

            case DriverState::Error: {
                _motor_speed_desired = 0;
                if (now_ms - _last_state_change_ms > 10000) {
                    _last_state_change_ms = now_ms;
                    _state = DriverState::PowerOn;
                }
            } break;
        }

        
    }
    
}

void AP_Torqeedo_TQBus::set_master_error_code(uint8_t error_code)
{
    if (error_code != 0 && error_code != _torqeedo_telemetry.master_error_code) {
        _state = DriverState::Error; // reset driver state to Error
        const uint32_t now = AP_HAL::millis(); // update last state change time
        _last_master_error_code_set_ms = now;
        _last_state_change_ms = now;
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Torqeedo error: E%03" PRIu8, error_code);
    }

    _torqeedo_telemetry.master_error_code = error_code;
}

void AP_Torqeedo_TQBus::process_rx_frame(const uint8_t* frame, uint8_t len)
{
#ifdef DEBUG_TORQEEDO
    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "TQBus: received frame: 0x%02X 0x%02X", frame[0], frame[1]);
#endif

    switch (frame[0]) {

        case static_cast<uint8_t>(MsgAddress::BUS_MASTER):
            handle_bus_master_msg(frame, len);
            break;

        case static_cast<uint8_t>(MsgAddress::REMOTE1):
            handle_remote_msg(frame, len);
            break;

        case static_cast<uint8_t>(MsgAddress::DISPLAY):
            handle_display_msg(frame, len);
            break;

        case static_cast<uint8_t>(MsgAddress::MOTOR):
            handle_motor_msg(frame, len);
            break;
        
        default:
            // unknown frame, ignore
            break;
    }

    
}

void AP_Torqeedo_TQBus::handle_bus_master_msg(const uint8_t* frame, uint8_t len)
{
    
    switch (_expectedReply) {
        case ExpectedReply::None:
            // no expected reply, ignore
            break;

        case ExpectedReply::MotorStatus:
        {
            // handle motor status message
            _torqeedo_telemetry.motor_status = frame[2];
            _torqeedo_telemetry.motor_errors = UINT16_VALUE(frame[3], frame[4]);

        } break;

        case ExpectedReply::MotorParam:
            if (len >= 14) {
                // handle motor parameter message
                _motor_rpm = (int16_t) ((frame[2] << 8) | frame[3]);
                const uint16_t power = (uint16_t) ((frame[4] << 8) | frame[5]);
                const uint16_t voltage = (uint16_t) ((frame[6] << 8) | frame[7]);
                const uint16_t current = (uint16_t) ((frame[8] << 8) | frame[9]);
                const int16_t pcb_temp = (int16_t) ((frame[10] << 8) | frame[11]);
                const int16_t motor_temp = (int16_t) ((frame[12] << 8) | frame[13]);

                const int16_t reported_temp = MAX(pcb_temp, motor_temp);

                TelemetryData telem_data;
                telem_data.temperature_cdeg = reported_temp;
                telem_data.voltage = 0.01f * voltage;
                telem_data.current = 0.1f * current;
                telem_data.motor_temp_cdeg = motor_temp;
                update_telem_data(_instance, telem_data, 
                    TelemetryType::TEMPERATURE |
                    TelemetryType::MOTOR_TEMPERATURE |
                    TelemetryType::VOLTAGE |
                    TelemetryType::CURRENT);
                update_rpm(_instance, _motor_rpm);

                static uint16_t counter = 0;
                if (counter++ % 10 == 0) {
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Torqeedo: RPM %d Pwr %dW %dV %dA PCB %d M %d",
                                    _motor_rpm, power, voltage, current, pcb_temp, motor_temp);
                }

            } break;
    }

    _expectedReply = ExpectedReply::None; // reset expected reply
}

void AP_Torqeedo_TQBus::handle_remote_msg(const uint8_t* frame, uint8_t len)
{
    switch (frame[1]) {
        case static_cast<uint8_t>(RemoteMsgId::INFO):
            {
                uint8_t remote_msg_reply[] = {
                    static_cast<uint8_t>(MsgAddress::BUS_MASTER),
                    0x00, // message ID in replys are always 0x00
                    0x00,
                };

                send_message(remote_msg_reply, sizeof(remote_msg_reply), _uart);

            }
            break;

        case static_cast<uint8_t>(RemoteMsgId::REMOTE):
            {
                uint8_t remote_msg_reply[] = {
                    static_cast<uint8_t>(MsgAddress::BUS_MASTER),
                    0x00, // message ID in replys are always 0x00
                    0x05, // flags, 0x05 means pin is present and motor speed is valid
                    0x00, // status
                    HIGHBYTE(_motor_speed_desired),
                    LOWBYTE(_motor_speed_desired),
                };

                // if (_state == DriverState::INITIALIZING) {
                //     remote_msg_reply[2] = 0x01;
                //     remote_msg_reply[3] = 0x02;
                //     remote_msg_reply[4] = 0x4c;
                //     remote_msg_reply[5] = 0x47;
                //     _state = DriverState::READY;
                // }

                send_message(remote_msg_reply, sizeof(remote_msg_reply), _uart);
                
            }
            break;

        case static_cast<uint8_t>(RemoteMsgId::SETUP):
            {
                uint16_t capacity_ah = 60;
                uint8_t charge_pct = 98;
                uint8_t tech = 1;
                 uint8_t remote_msg_reply[] = {
                    static_cast<uint8_t>(MsgAddress::BUS_MASTER),
                    0x00, // message ID in replys are always 0x00
                    0x01, // flags, 0x01 means pexit setup and do not save
                    HIGHBYTE(capacity_ah), // battery capacity in Ampere.hours
                    LOWBYTE(capacity_ah), // battery capacity in Ampere.hours
                    charge_pct, // battery charge percentage
                    tech, // battery technology, 0 = lead acid, 1 = Lithium-Ion
                };

                send_message(remote_msg_reply, sizeof(remote_msg_reply), _uart);

            }
            break;

        default:
            // unknown remote message ID, ignore
            break;
    }
}

void AP_Torqeedo_TQBus::handle_display_msg(const uint8_t* frame, uint8_t len)
{
    switch (frame[1]) {
        case static_cast<uint8_t>(DisplayMsgId::INFO):
            // {
            //     uint8_t display_msg_reply[] = {
            //         static_cast<uint8_t>(MsgAddress::BUS_MASTER),
            //         0x00, // message ID in replys are always 0x00
            //         0x00,
            //     };

            //     send_message(display_msg_reply, sizeof(display_msg_reply), _uart);
            // }
            break;

        case static_cast<uint8_t>(DisplayMsgId::SYSTEM_STATE):
            {

                // uint8_t flags0 = frame[2];
                // uint8_t flags1 = frame[3];
                // uint8_t master_state = frame[4];
                const uint8_t master_error_code = frame[5];
                // const float motor_voltage = 0.01f * UINT16_VALUE(frame[6], frame[7]); // convert to Volts
                // const float motor_current = 0.1f * UINT16_VALUE(frame[8], frame[9]); // convert to Amps

                // _motor_rpm = (int16_t) ((frame[12] << 8) | frame[13]);
                // const uint8_t motor_pcb_temp = frame[14];
                // const uint8_t motor_stator_temp = frame[15];

                // uint8_t display_msg_reply[] = {
                //     static_cast<uint8_t>(MsgAddress::BUS_MASTER),
                //     0x00, // message ID in replys are always 0x00
                // };
                // send_message(display_msg_reply, sizeof(display_msg_reply), _uart);

                

                if (master_error_code != 0) {
                    set_master_error_code(master_error_code);
                }

            }
            break;

        case static_cast<uint8_t>(DisplayMsgId::SYSTEM_SETUP):
            {
                // uint8_t display_msg_reply[] = {
                //     static_cast<uint8_t>(MsgAddress::BUS_MASTER),
                //     0x00, // message ID in replys are always 0x00
                // };

                // send_message(display_msg_reply, sizeof(display_msg_reply), _uart);
            }

        default:
            // unknown display message ID, ignore
            break;
    }
}

void AP_Torqeedo_TQBus::handle_motor_msg(const uint8_t* frame, uint8_t len)
{
    switch (frame[1]) {
        case static_cast<uint8_t>(MotorMsgId::INFO):
            // handle motor info message
            break;

        case static_cast<uint8_t>(MotorMsgId::STATUS):
            {
                

                _expectedReply = ExpectedReply::MotorStatus;

            } break;

        case static_cast<uint8_t>(MotorMsgId::DRIVE):
            // handle motor drive message
            break;

        case static_cast<uint8_t>(MotorMsgId::PARAM):
            {
                _expectedReply = ExpectedReply::MotorParam;
            } break;

        default:
            // unknown motor message ID, ignore
            break;
    }
}


// get latest battery status info.  returns true on success and populates arguments
bool AP_Torqeedo_TQBus::get_batt_info(float &voltage, float &current_amps, float &temp_C, uint8_t &pct_remaining) const
{
    return false;
}

// get battery capacity.  returns true on success and populates argument
bool AP_Torqeedo_TQBus::get_batt_capacity_Ah(uint16_t &amp_hours) const
{
   return false;
}


#endif // HAL_TORQEEDO_ENABLED
