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

#include <AP_Common/async.h>

#define TORQEEDO_SERIAL_BAUD        19200   // communication is always at 19200
#define TORQEEDO_PARITY             0       // communication is always no parity
#define TORQEEDO_STOP_BITS          1       // communication is always 1 stop bit
#define TORQEEDO_PACKET_HEADER      0xAC    // communication packet header
#define TORQEEDO_PACKET_FOOTER      0xAD    // communication packet footer
#define TORQEEDO_PACKET_ESCAPE      0xAE    // escape character for handling occurrences of header, footer and this escape bytes in original message
#define TORQEEDO_PACKET_ESCAPE_MASK 0x80    // byte after ESCAPE character should be XOR'd with this value

#define TORQEEDO_MESSAGE_LEN_MAX    35  // messages are no more than 35 bytes

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
  _state(DriverState::INITIALIZING)
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

// returns true if communicating with the motor
bool AP_Torqeedo_TQBus::healthy()
{
    // if the last received time is more than 2 seconds ago, we consider the connection unhealthy
    if (AP_HAL::millis() - _last_rx_ms > 2000) {
        if (_state == DriverState::RUNNING) {
            _state = DriverState::INITIALIZING; // reset driver state to INITIALIZING
            _last_state_change_ms = AP_HAL::millis(); // update last state change time
        }
        return false;
    } else {
        return true;
    }
    
}

void AP_Torqeedo_TQBus::reset()
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Torqeedo: resetting driver");
    _state = DriverState::INITIALIZING;
    _last_state_change_ms = AP_HAL::millis();
    _motor_speed_desired = 0;
    _motor_rpm = 0;
    _master_error_code = 0;
    _last_rx_ms = 0;
    _last_set_rpm_ms = 0;

    if (_uart != nullptr) {
        _uart->discard_input();
    }

    // set throttle to zero and wait 5 seconds
    _motor_speed_desired = 0;
        // use serial port's RTS pin to turn on battery
    _uart->set_RTS_pin(true);
    hal.scheduler->delay(5000);
    _uart->set_RTS_pin(false);
                
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

        switch (_state) {
            case DriverState::INITIALIZING:
                if (AP_HAL::millis() - _last_state_change_ms > 5000) {
                    _last_state_change_ms = AP_HAL::millis();
                    reset();
                    _state = DriverState::READY;
                }
                break;

            case DriverState::REVERSE_WAIT:
                if (AP_HAL::millis() - _last_state_change_ms > 3000) {
                    _last_state_change_ms = AP_HAL::millis();
                    _state = DriverState::READY;
                }
                break;

            case DriverState::READY:
                if (hal.util->get_soft_armed()) {
                    _last_state_change_ms = AP_HAL::millis();
                    _state = DriverState::RUNNING;
                }
                break;

            case DriverState::RUNNING:
            {
                #define SLEW_FILTER_CUTOFF_FREQ 1.0f // cutoff frequency for the lowpass filter in Hz

                const uint32_t now_ms = AP_HAL::millis();
                const float dt = (now_ms - _last_set_rpm_ms) / 1000.0f; // time since last state change in seconds
                _last_set_rpm_ms = now_ms; // update last set RPM time

                float rpm_cmd = SRV_Channels::get_output_norm((SRV_Channel::Aux_servo_function_t)_params.servo_fn.get()) * 1000.0f;
                rpm_cmd = _motor_speed_desired + calc_lowpass_alpha_dt(dt, SLEW_FILTER_CUTOFF_FREQ) * (rpm_cmd - _motor_speed_desired);

                // if the signs rpm_cmd and _motor_speed_desired signs are different, we need to wait for the direction change delay before setting the new RPM
                if ((_motor_speed_desired < 0 && rpm_cmd > 0) || (_motor_speed_desired > 0 && rpm_cmd < 0)) {
                    _motor_speed_desired = 0; // set motor speed to zero to stop the motor
                    _state = DriverState::REVERSE_WAIT;
                    _last_state_change_ms = now_ms; // update last state change time
                } else {
                    _motor_speed_desired = constrain_int16( static_cast<int16_t>(rpm_cmd), -1000, 1000);
                }

                if (now_ms - _last_state_change_ms > 5000) {
                    _last_state_change_ms = now_ms;

                    // if the motor's rpm is close to zero and if the abs value of the desired RPM is > 100, reset the driver state to INITIALIZING
                    if (abs(_motor_rpm) < 100 && abs(_motor_speed_desired) > 100) {
                        _state = DriverState::INITIALIZING;
                        _last_state_change_ms = now_ms; // update last state change time
                        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Torqeedo: no RPM detected, resetting driver");
                    }
                }
            } break;
        }
    }
    
}

void AP_Torqeedo_TQBus::set_master_error_code(uint8_t error_code)
{

    if (error_code != 0 && error_code != _master_error_code) {
        _state = DriverState::INITIALIZING; // reset driver state to INITIALIZING
        _last_state_change_ms = AP_HAL::millis(); // update last state change time
        _master_error_code = error_code;
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Torqeedo error: E%02" PRIu8, _master_error_code);
    }
    
}

// #define DEBUG_TORQEEDO 1

void AP_Torqeedo_TQBus::process_rx_frame(const uint8_t* frame, uint8_t len)
{
#ifdef DEBUG_TORQEEDO
    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "TQBus: received frame: 0x%02X 0x%02X", frame[0], frame[1]);
#endif

    switch (frame[0]) {
        case static_cast<uint8_t>(MsgAddress::REMOTE1):
            handle_remote_msg(frame, len);
            break;

        case static_cast<uint8_t>(MsgAddress::DISPLAY):
            handle_display_msg(frame, len);
            break;
        
        default:
            // unknown frame, ignore
            break;
    }

    
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

                #ifdef DEBUG_TORQEEDO
                GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "TQBus: remote speed set to %d", _motor_speed_desired);
                #endif
                
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
                float motor_voltage = 0.01f * UINT16_VALUE(frame[6], frame[7]); // convert to Volts
                float motor_current = 0.1f * UINT16_VALUE(frame[8], frame[9]); // convert to Amps
                _motor_rpm = (int16_t)UINT16_VALUE(frame[12], frame[13]); // RPM value
                uint8_t motor_stator_temp = frame[15];

                // uint8_t display_msg_reply[] = {
                //     static_cast<uint8_t>(MsgAddress::BUS_MASTER),
                //     0x00, // message ID in replys are always 0x00
                // };
                // send_message(display_msg_reply, sizeof(display_msg_reply), _uart);

                TelemetryData telem_data = {
                    .temperature_cdeg = static_cast<int16_t>(10 * motor_stator_temp), // convert to centi-degrees C
                    .voltage = motor_voltage,
                    .current = motor_current,
                };
                update_telem_data(_instance, telem_data, TelemetryType::VOLTAGE | TelemetryType::CURRENT | TelemetryType::TEMPERATURE);
                update_rpm(_instance, (float)_motor_rpm);

                telem_data.temperature_cdeg = master_error_code;
                update_telem_data(_instance + 1, telem_data, TelemetryType::TEMPERATURE);
                set_master_error_code(master_error_code);

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
