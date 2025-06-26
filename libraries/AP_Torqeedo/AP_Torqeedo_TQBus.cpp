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

#define TORQEEDO_SERIAL_BAUD        19200   // communication is always at 19200
#define TORQEEDO_PACKET_HEADER      0xAC    // communication packet header
#define TORQEEDO_PACKET_FOOTER      0xAD    // communication packet footer
#define TORQEEDO_PACKET_ESCAPE      0xAE    // escape character for handling occurrences of header, footer and this escape bytes in original message
#define TORQEEDO_PACKET_ESCAPE_MASK 0x80    // byte after ESCAPE character should be XOR'd with this value
#define TORQEEDO_LOG_TRQD_INTERVAL_MS                   5000// log TRQD message at this interval in milliseconds
#define TORQEEDO_SEND_MOTOR_SPEED_INTERVAL_MS           100 // motor speed sent at 10hz if connected to motor
#define TORQEEDO_SEND_MOTOR_STATUS_REQUEST_INTERVAL_MS  400 // motor status requested every 0.4sec if connected to motor
#define TORQEEDO_SEND_MOTOR_PARAM_REQUEST_INTERVAL_MS   400 // motor param requested every 0.4sec if connected to motor
#define TORQEEDO_SEND_BATTERY_STATUS_REQUEST_INTERVAL_MS 400 // battery status requested every 0.4sec
#define TORQEEDO_BATT_TIMEOUT_MS    5000    // battery info timeouts after 5 seconds
#define TORQEEDO_REPLY_TIMEOUT_MS   25      // stop waiting for replies after 25ms
#define TORQEEDO_ERROR_REPORT_INTERVAL_MAX_MS   10000   // errors reported to user at no less than once every 10 seconds
#define TORQEEDO_MIN_RESET_INTERVAL_MS 5000    // minimum time between hard resets/wake
#define TORQEEDO_RESET_THROTTLE_HOLDDOWN_MS 6000 // throttle hold down time after reset/wake

extern const AP_HAL::HAL& hal;

class TQBusTransaction
{
public:
    typedef struct Buffer {
        uint8_t data[MODBUS_MAX_MSG_LEN];
        uint8_t len;
    } Buffer;

    TQBusTransaction() {};
    TQBusTransaction(AP_HAL::UARTDriver *uart);
    // CLASS_NO_COPY(TQBusTransaction);

    void set_tx_data(uint8_t* data, uint8_t len);
    
    /**
     * run the transaction state machine
     * @return true if the transaction is finished
     */
    bool run();

    bool is_finished() const;
    bool is_timeout() const;

protected:
    virtual bool parse_response(uint8_t byte) = 0;
    Buffer buffer;

private:
    AP_HAL::UARTDriver *uart;
    uint8_t read_len;
    uint32_t start_wait_us;
    uint32_t last_received_ms;

    /**
     * Wait for a specified number of microseconds.
     * Keep calling this function until the wait is finished.
     * @param us The number of microseconds to wait.
     * @return true if the wait is finished, false otherwise.
     */
    bool wait_us(uint32_t us);
    

    enum ModbusState {
        Init = 0,
        Send,
        WaitingToFinshSend,
        WaitingForResponse,
        Finished,
        Timeout,
    };

    ModbusState state;

};

bool TQBusTransaction::run()
{
    switch (state) {
        case Init:
            uart->discard_input();
            read_len = 0;
            // send the write buffer
            state = Send;
        FALLTHROUGH;

        case Send:
#ifdef DEBUG
            debug_print_buf(buffer.data, buffer.len, "=> ");
#endif // DEBUG

#ifdef SOFTWARE_FLOWCONTROL
            uart->set_CTS_pin(true);
#endif
            {
                size_t i = 0;
                size_t bytes_written;
                while (i < buffer.len) {
                    // write the data to the UART
                    bytes_written = uart->write(&buffer.data[i], buffer.len - i);
                    i += bytes_written;
                }
            }
            uart->flush();
            last_received_ms = AP_HAL::millis();
            state = WaitingToFinshSend;
#ifdef IO_ASYNC_WAIT
            start_wait_us = AP_HAL::micros();
#endif
        FALLTHROUGH;

        case WaitingToFinshSend:
#ifdef SOFTWARE_FLOWCONTROL
            if (!wait_us(calc_transmit_time_us(buffer.len))) {
                return false;
            }
            uart->set_CTS_pin(false);
#endif
            state = WaitingForResponse;
        FALLTHROUGH;

        case WaitingForResponse: 
        {
            uint8_t b;
            uart->wait_timeout(1, 100);
            if (uart->read(&b, 1) == 1) {
                last_received_ms = AP_HAL::millis();
                if (parse_response(b)) {
                    state = Finished;
                }
            } else if (AP_HAL::millis() - last_received_ms > 100) {
                // timeout waiting for response
                state = Timeout;
            }

        } break;

        case Finished:
            // transaction is finished
            break;
        case Timeout:
            // transaction timed out
            break;
    }

    return is_finished();
    
}

bool TQBusTransaction::is_finished() const
{
    return state == Finished || state == Timeout;
}

bool TQBusTransaction::is_timeout() const
{
    return state == Timeout;
}



// initialise driver
void AP_Torqeedo_TQBus::init()
{
    

    // create background thread to process serial input and output
    char thread_name[15];
    hal.util->snprintf(thread_name, sizeof(thread_name), "torqeedo%u", (unsigned)_instance);
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Torqeedo_TQBus::thread_main, void), thread_name, 2048, AP_HAL::Scheduler::PRIORITY_RCOUT, 1)) {
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
