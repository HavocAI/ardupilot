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

#include "State_Run_Forward.h"
#include <GCS_MAVLink/GCS.h>

void State_Run_Forward::enter(AP_MarineICE& ctx) {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[MarineICE] RUN_FORWARD: Entering forward mode...");
}

void State_Run_Forward::run(AP_MarineICE& ctx) {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[MarineICE] RUN_FORWARD: Running...");
    // Add logic for forward operation
}

void State_Run_Forward::exit(AP_MarineICE& ctx) {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[MarineICE] RUN_FORWARD: Exiting forward mode...");
}
