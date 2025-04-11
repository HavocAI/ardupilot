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

#include "State_Run_Reverse.h"
#include "../AP_MarineICE.h"

void State_Run_Reverse::enter(AP_MarineICE& ctx) {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[MarineICE] RUN_REVERSE: Entering reverse mode...");
    ctx.set_gear(AP_MarineICE::GearPosition::REVERSE);
}

void State_Run_Reverse::run(AP_MarineICE& ctx) {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[MarineICE] RUN_REVERSE: Running...");
    // Add logic for reverse operation
}

void State_Run_Reverse::exit(AP_MarineICE& ctx) {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[MarineICE] RUN_REVERSE: Exiting reverse mode...");
}
