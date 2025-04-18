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

#include "AP_MarineICE_Params.h"

const AP_Param::GroupInfo AP_MarineICE_Params::var_info[] = {

    // @Param: TYPE
    // @DisplayName: Type
    // @Description: Driver type to use
    // @Values: 0:Disabled, 1:Simulated, 2:NMEA2000-Dometic-Suzuki
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_MarineICE_Params, type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: AUTO_START
    // @DisplayName: Auto Start
    // @Description: Whether to automatically start the engine
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    // @RebootRequired: False
    AP_GROUPINFO("AUTO_START", 2, AP_MarineICE_Params, auto_start, 0),

    // @Param: AUTO_TRIM
    // @DisplayName: Auto Trim
    // @Description: Whether to automatically trim the engine
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    // @RebootRequired: False
    AP_GROUPINFO("AUTO_TRIM", 3, AP_MarineICE_Params, auto_trim, 0),

    // @Param: SAFE_TRIM
    // @DisplayName: Safe Trim
    // @Description: Maximum trim level to allow the engine to run (pct)
    // @Units: degrees
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: False
    AP_GROUPINFO("SAFE_TRIM", 4, AP_MarineICE_Params, safe_trim, 50),

    // @Param: START_TIME
    // @DisplayName: Start Time
    // @Description: Max time (sec) that the starter will run during each try
    // @Units: seconds
    // @Range: 0 255
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: False
    AP_GROUPINFO("START_TIME", 5, AP_MarineICE_Params, start_time, 5),

    // @Param: START_DELAY
    // @DisplayName: Start Delay
    // @Description: Delay (sec) between starting the engine
    // @Units: seconds
    // @Range: 0 255
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: False
    AP_GROUPINFO("START_DELAY", 6, AP_MarineICE_Params, start_delay, 15),

    // @Param: START_RETRY
    // @DisplayName: Start Retries
    // @Description: Number of retries to start the engine
    // @Units: retries
    // @Range: 0 255
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: False
    AP_GROUPINFO("START_RETRY", 7, AP_MarineICE_Params, start_retries, 2),

    // @Param: RPM_THRES
    // @DisplayName: RPM Threshold
    // @Description: Threshold for RPM to consider the engine started
    // @Units: RPM
    // @Range: 0 6000
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: False
    AP_GROUPINFO("RPM_THRES", 8, AP_MarineICE_Params, rpm_thres, 500),

    // @Param: THR_SLEW
    // @DisplayName: Throttle Slew Rate
    // @Description: Throttle slew rate (%/sec)
    // @Units: %/sec
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: False
    AP_GROUPINFO("THR_SLEW", 9, AP_MarineICE_Params, thr_slewrate, 25),

    // @Param: THR_MAX
    // @DisplayName: Max Throttle
    // @Description: Maximum throttle level (%)
    // @Units: %
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: False
    AP_GROUPINFO("THR_MAX", 10, AP_MarineICE_Params, thr_max, 100),

    // @Param: THR_DB
    // @DisplayName: Throttle Deadband
    // @Description: Zero deadband for throttle input (%)
    // @Units: %
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: False
    AP_GROUPINFO("THR_DB", 11, AP_MarineICE_Params, thr_deadband, 5),

    // @Param: RPM_MAX
    // @DisplayName: Max RPM
    // @Description: Maximum Normal RPM (RPM)
    // @Units: RPM
    // @Range: 0 6000
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: False
    AP_GROUPINFO("RPM_MAX", 12, AP_MarineICE_Params, rpm_max, 6000),

    // @Param: TEMP_MAX
    // @DisplayName: Max Temperature
    // @Description: Maximum normal temperature (deg C)
    // @Units: degrees
    // @Range: 0.0 300.0
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: False
    AP_GROUPINFO("TEMP_MAX", 13, AP_MarineICE_Params, temp_max, 150.0f),

    // @Param: DP_CHK_EN
    // @DisplayName: Water Depth Check Enabled
    // @Description: Whether to check water depth (0:disabled, 1:enabled)
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    // @RebootRequired: False
    AP_GROUPINFO("DP_CHK_EN", 14, AP_MarineICE_Params, water_depth_check_enabled, 0),

    // @Param: MIN_DPTH
    // @DisplayName: Minimum Water Depth
    // @Description: Minimum water depth (m) to start the engine
    // @Units: meters
    // @Range: 0.0 100.0
    // @Increment: 0.1
    // @User: Standard
    // @RebootRequired: False
    AP_GROUPINFO("MIN_DPTH", 15, AP_MarineICE_Params, water_depth_thres, 1.0f),

    // @Param: CAN_PORT
    // @DisplayName: CAN Port
    // @Description: CAN port to use for the CAN backend (0:CAN1, 1:CAN2, -1:disabled)
    // @Units: port number
    // @Range: -1 255
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: False
    AP_GROUPINFO("CAN_PORT", 16, AP_MarineICE_Params, can_port, 0),

    // @Param: LC_BANK
    // @DisplayName: Load Center Bank Instance
    // @Description: Bank instance of the Load Center Binary Device (Maretron)
    // @Units: instance number
    // @Range: 0 255
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: False
    AP_GROUPINFO("LC_BANK", 17, AP_MarineICE_Params, lc_bank_instance, 42),

    // @Param: LC_IGN
    // @DisplayName: Load Center Ignition Channel
    // @Description: Load Center Ignition channel
    // @Units: channel number
    // @Range: 0 28
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: False
    AP_GROUPINFO("LC_IGN", 18, AP_MarineICE_Params, lc_ignition, 3),

    // @Param: LC_START
    // @DisplayName: Load Center Starter Channel
    // @Description: Load Center Starter channel
    // @Units: channel number
    // @Range: 0 28
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: False
    AP_GROUPINFO("LC_START", 19, AP_MarineICE_Params, lc_starter, 2),

    // @Param: LC_TRIMUP
    // @DisplayName: Load Center Trim Up Channel
    // @Description: Load Center Trim Up channel
    // @Units: channel number
    // @Range: 0 28
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: False
    AP_GROUPINFO("LC_TRIMUP", 20, AP_MarineICE_Params, lc_trim_up, 12),

    // @Param: LC_TRIMDN
    // @DisplayName: Load Center Trim Down Channel
    // @Description: Load Center Trim Down channel
    // @Units: channel number
    // @Range: 0 28
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: False
    AP_GROUPINFO("LC_TRIMDN", 21, AP_MarineICE_Params, lc_trim_down, 11),

    AP_GROUPEND
};

AP_MarineICE_Params::AP_MarineICE_Params(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}
