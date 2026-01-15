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
* AP_BattMonitor_SSM.h
*
*      Author: Andrew Gregg
*/

#pragma once

#include "AP_BattMonitor_config.h"

#if AP_BATTERY_SSM_ENABLED
#include "AP_BattMonitor_Backend.h"
#include <AP_BattMonitor/ssmbattery.h>
#include <AP_CANManager/AP_CANSensor.h>
#include <AP_J1939_CAN/AP_J1939_CAN.h>

class AP_BattMonitor_SSM : public AP_BattMonitor_Backend, public CANSensor
{
public:
    AP_BattMonitor_SSM(AP_BattMonitor &mon, 
                       AP_BattMonitor::BattMonitor_State &mon_state,
                       AP_BattMonitor_Params &params);

    CLASS_NO_COPY(AP_BattMonitor_SSM);

    bool has_consumed_energy() const override { return true; }
    bool has_current() const override { return true; }
    // bool has_cell_voltages() const override { return true; }
    bool has_temperature() const override { return true; }
    bool capacity_remaining_pct(uint8_t &percentage) const override;

    void init() override;
    void read() override;
    
    static const struct AP_Param::GroupInfo var_info[];

    struct ssm_fault_state_t {
        ssmbattery_hardware_and_battery_failure_information_t hwinfo;
        ssmbattery_fault_t faults[2]; // NOTE: Page 1 and Page 2 are stored as their own message, this wastes 2 bytes to store the implicit page number
    };

    ssm_fault_state_t get_ssm_fault_info();
private:

    void tick(void);
    void init_can();

    // handler for incoming frames
    void handle_frame(AP_HAL::CANFrame &frame) override;

    bool send_status_request();

    void send_query_frame();

    void handle_cell_voltage_information(const struct ssmbattery_cell_voltage_information_t &msg);
    void handle_cell_temperature_information(const struct ssmbattery_cell_temperature_information_t &msg);
    void handle_total_information_0(const struct ssmbattery_total_information_0_t &msg);
    void handle_total_information_1(const struct ssmbattery_total_information_1_t &msg);
    void handle_cell_voltage_statistical_information(const struct ssmbattery_cell_voltage_statistical_information_t &msg);
    void handle_unit_temperature_statistical_information(const struct ssmbattery_unit_temperature_statistical_information_t &msg);
    void handle_status_information_0(const struct ssmbattery_status_information_0_t &msg);
    void handle_status_information_1(const struct ssmbattery_status_information_1_t &msg);
    void handle_status_information_2(const struct ssmbattery_status_information_2_t &msg);
    void handle_hardware_and_battery_failure_information(const struct ssmbattery_hardware_and_battery_failure_information_t &msg);
    void handle_charging_information(const struct ssmbattery_charging_information_t &msg);
    void handle_limiting(const struct ssmbattery_limiting_t &msg);
    void handle_fault(const struct ssmbattery_fault_t &msg);

    void split_id(uint32_t can_id, uint32_t& base_id, uint32_t& board_number);

    AP_BattMonitor::BattMonitor_State _interim_state;
    ssm_fault_state_t _fault_state, _interim_fault_state;
    uint8_t _capacity_remaining_pct;

    // Parameters
    AP_Int8 _can_port;
    AP_Int8 _board_number;
    AP_Int8 _num_cells;

    // J1939 CAN backend
    AP_J1939_CAN* j1939;

    HAL_Semaphore _sem_battmon;
    uint32_t _last_query_time;
};

#endif // AP_BATTERY_SSM_ENABLED
