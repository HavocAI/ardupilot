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

#include <AP_Common/async.h>

#define TORQEEDO_SERIAL_BAUD        19200   // communication is always at 19200
#define TORQEEDO_PARITY             0       // communication is always no parity
#define TORQEEDO_STOP_BITS          1       // communication is always 1 stop bit
#define TORQEEDO_PACKET_HEADER      0xAC    // communication packet header
#define TORQEEDO_PACKET_FOOTER      0xAD    // communication packet footer
#define TORQEEDO_PACKET_ESCAPE      0xAE    // escape character for handling occurrences of header, footer and this escape bytes in original message
#define TORQEEDO_PACKET_ESCAPE_MASK 0x80    // byte after ESCAPE character should be XOR'd with this value

#define TORQEEDO_MESSAGE_LEN_MAX    35  // messages are no more than 35 bytes

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

static bool send_message(uint8_t* pkt_body, uint8_t len, uint8_t& num_bytes_written, AP_HAL::UARTDriver* uart)
{
    num_bytes_written = 0;
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
            
            pkt_body[i] ^= TORQEEDO_PACKET_ESCAPE_MASK; // XOR with escape mask

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

    uart->flush();
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
  _uart(nullptr)
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
    return false;
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

        // wait for data to be available
        if (_uart->available() == 0) {
            hal.scheduler->delay(10); // no data, wait a bit
            continue;
        }

        // read bytes from UART
        uint8_t b;
        while (_uart->read(b)) {
            if (rx_frame.push_byte(b)) {
                // complete frame received
                process_rx_frame(rx_frame.get_buffer(), rx_frame.get_length());
                rx_frame.reset();
            }
        }
    }
    
}

void AP_Torqeedo_TQBus::process_rx_frame(const uint8_t* frame, uint8_t len)
{

    
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
