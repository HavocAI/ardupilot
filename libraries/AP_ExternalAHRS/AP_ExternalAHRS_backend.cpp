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
  parent class for ExternalAHRS backends
 */

#include "AP_ExternalAHRS_backend.h"
#include <AP_AHRS/AP_AHRS.h>

#if HAL_EXTERNAL_AHRS_ENABLED

#include <GCS_MAVLink/GCS.h>

#if EXTERNAL_AHRS_LOGGING_ENABLED
#include <AP_Filesystem/AP_Filesystem.h>
#endif

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_backend::AP_ExternalAHRS_backend(AP_ExternalAHRS *_frontend,
                                                 AP_ExternalAHRS::state_t &_state) :
    state(_state),
    frontend(*_frontend)
{}


uint16_t AP_ExternalAHRS_backend::get_rate(void) const
{
    return frontend.get_IMU_rate();
}

bool AP_ExternalAHRS_backend::option_is_set(AP_ExternalAHRS::OPTIONS option) const
{
    return frontend.option_is_set(option);
}

bool AP_ExternalAHRS_backend::in_fly_forward(void) const
{
    return AP::ahrs().get_fly_forward();
}

#if EXTERNAL_AHRS_LOGGING_ENABLED
void AP_ExternalAHRS_backend::log_data(const void* data, uint16_t len)
{
    logging.buf.write(static_cast<const uint8_t*>(data), len);
    if (!log_thread_created) {
        log_thread_created = true;
        hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_backend::logging_start, void), "eahrs_log", 4096, AP_HAL::Scheduler::PRIORITY_IO, 0);
    }
}

void AP_ExternalAHRS_backend::logging_start()
{
    bool is_dirty = false;
    uint32_t last_flush_ms = 0;
    const char* log_filename = "eahrs.log";
    // AP::FS().unlink(log_filename);
    logging.fd = AP::FS().open(log_filename, O_WRONLY|O_CREAT|O_TRUNC, 0644);

    while (true) {
        hal.scheduler->delay(10);

        if (logging.fd >= 0 && logging.buf.available()) {
            uint32_t n;
            const uint8_t* ptr = logging.buf.readptr(n);
            AP::FS().write(logging.fd, ptr, n);
            logging.buf.advance(n);
            is_dirty = true;
        }

        if (is_dirty && AP_HAL::millis() - last_flush_ms > 1000) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EAHRS log flushed");
            AP::FS().fsync(logging.fd);
            last_flush_ms = AP_HAL::millis();
            is_dirty = false;
        }
    }
}

#endif

#endif  // HAL_EXTERNAL_AHRS_ENABLED

