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
  rover simulator class
*/

#include "SIM_Rover.h"

#include <string.h>
#include <stdio.h>

#include <AP_Math/AP_Math.h>

namespace SITL {

SimRover::SimRover(const char *frame_str) :
    Aircraft(frame_str)
{
    skid_steering = strstr(frame_str, "skid") != nullptr;

    if (skid_steering) {
        printf("SKID Steering Rover Simulation Started\n");
        // these are taken from a 6V wild thumper with skid steering,
        // with a sabertooth controller
        max_accel = 14;
        max_speed = 4;
        return;
    }

    printf("Custom Dynamics Rover Simulation Started\n");

    lock_step_scheduled = true;
}

/*
  return turning circle (diameter) in meters for steering angle proportion in degrees
*/
float SimRover::turn_circle(float steering) const
{
    if (fabsf(steering) < 1.0e-6) {
        return 0;
    }
    return turning_circle * sinf(radians(max_wheel_turn)) / sinf(radians(steering*max_wheel_turn));
}

/*
   return yaw rate in degrees/second given steering_angle and speed
*/
float SimRover::calc_yaw_rate(float steering, float speed)
{
    if (skid_steering) {
        return constrain_float(steering * skid_turn_rate, -MAX_YAW_RATE, MAX_YAW_RATE);
    }
    if (vectored_thrust) {
        return constrain_float(steering * vectored_turn_rate_max, -MAX_YAW_RATE, MAX_YAW_RATE);
    }
    if (fabsf(steering) < 1.0e-6 or fabsf(speed) < 1.0e-6) {
        return 0;
    }
    float d = turn_circle(steering);
    float c = M_PI * d;
    float t = c / speed;
    float rate = constrain_float(360.0f / t, -MAX_YAW_RATE, MAX_YAW_RATE);
    return rate;
}

/*
  return lateral acceleration in m/s/s
*/
float SimRover::calc_lat_accel(float steering_angle, float speed)
{
    float yaw_rate = calc_yaw_rate(steering_angle, speed);
    float accel = radians(yaw_rate) * speed;
    return accel;
}

/*
  update the rover simulation by one time step
 */
void SimRover::update(const struct sitl_input &input)
{
    // how much time has passed?
    float delta_time = frame_time_us * 1.0e-6f;

    // update gyro and accel_body according to frame type
    if (omni3) {
        update_omni3(input, delta_time);
    } else {
        // Use a custom havoc vehicle dynamics model
        if (havoc_vehicle_sim_dynamics_type == 2){
            update_rampage_gen2(input, delta_time);
        }
        else{
            update_ackermann_or_skid(input, delta_time);
        }
    }

    // common to all rovers

    // now in earth frame
    Vector3f accel_earth = dcm * accel_body;
    accel_earth += Vector3f(0, 0, GRAVITY_MSS);

    // we are on the ground, so our vertical accel is zero
    accel_earth.z = 0;

    // work out acceleration as seen by the accelerometers. It sees the kinematic
    // acceleration (ie. real movement), plus gravity
    accel_body = dcm.transposed() * (accel_earth + Vector3f(0, 0, -GRAVITY_MSS));

    // new velocity vector
    velocity_ef += accel_earth * delta_time;

    // new position vector
    position += (velocity_ef * delta_time).todouble();

    update_external_payload(input);

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}

void SimRover::update_rampage_gen2(const struct sitl_input &input, float delta_time)
{
    // speed in m/s in body frame
    Vector3f velocity_body = dcm.transposed() * velocity_ef;

    // Current velocities for dynamics calculations
    float v_x = velocity_body.x;  // forward velocity
    float v_y = velocity_body.y;  // starboard velocity  
    float v_yaw_deg_s = degrees(gyro.z); // current yaw rate in deg/s

    // Check for NaN and set to 0 if needed
    if (isnan(v_x) || !isfinite(v_x)) {
        v_x = 0.0f;
    }
    if (isnan(v_y) || !isfinite(v_y)) {
        v_y = 0.0f;
    }
    if (isnan(v_yaw_deg_s) || !isfinite(v_yaw_deg_s)) {
        v_yaw_deg_s = 0.0f;
    }

    float servo1_output = (input.servos[0] - 1500.0f)/1000.0f; // steering
    float servo3_output = (input.servos[2] - 1500.0f)/1000.0f;
    float servo6_output = (input.servos[5] - 1000.0f)/1000.0f;

    // Custom dynamics equations  
    float v_yaw_deg_s_tp1 = 0.8788459312 * v_yaw_deg_s - 3.9296105163 * servo3_output - 49.3558109408 * servo1_output*servo3_output;
    float forward_speed_tp1 = 0.9600673480 * v_x + 0.8431726738 * servo3_output;
    float starboard_speed_tp1 = 0.9979987959 * v_y + 0.2795478102 * servo1_output*servo3_output;

    // float v_yaw_deg_s_tp1 = 0.4916564184 * v_yaw_deg_s - 0.0377095101 * v_x - 0.0173249737 * v_y - 0.5474767805 * servo1_output - 1.7479285367 * servo3_output - 1.4322567509 * servo6_output - 0.0030811745 * v_yaw_deg_s * fabsf(v_yaw_deg_s) + 0.7417252685 * v_x * fabsf(v_x) + 0.0307017345 * v_y * fabsf(v_y) + 2.7803675978 * servo1_output * fabsf(servo1_output) + 39.6328582426 * servo3_output * fabsf(servo3_output) - 286.4513501710 * servo6_output * fabsf(servo6_output) - 0.0171158803 * v_yaw_deg_s * fabsf(v_x) + 0.0043611697 * v_yaw_deg_s * fabsf(v_y) + 0.1988914066 * v_yaw_deg_s * fabsf(servo1_output) + 98.3312836846 * v_yaw_deg_s * fabsf(servo6_output) + 0.0320896254 * v_x * fabsf(v_y) + 0.3093921495 * v_x * fabsf(servo1_output) + 0.1161545418 * v_x * fabsf(servo3_output) - 7.5419020177 * v_x * fabsf(servo6_output) - 0.4583748473 * v_y * fabsf(servo1_output) + 0.0440219117 * v_y * fabsf(servo3_output) - 3.4649947405 * v_y * fabsf(servo6_output) - 14.9576128441 * servo1_output * fabsf(servo3_output) - 109.4953560999 * servo1_output * fabsf(servo6_output) - 349.5857073488 * servo3_output * fabsf(servo6_output) - 0.0001233248 * powf(v_yaw_deg_s, 2) - 0.7460009872 * powf(v_x, 2) - 0.0640987752 * powf(v_y, 2) - 21.7940955738 * powf(servo3_output, 2) + 286.4513501731 * powf(servo6_output, 2) + 0.0000544364 * powf(v_yaw_deg_s, 3) + 0.0002939413 * powf(v_x, 3) - 0.0110899243 * powf(v_y, 3) + 5.4802165574 * powf(servo1_output, 3) - 32.4357758347 * powf(servo3_output, 3) - 57290.2700370548 * powf(servo6_output, 3);
    // float forward_speed_tp1 = 0.0012184941 * v_yaw_deg_s + 0.4712879431 * v_x - 0.0130216917 * v_y - 0.0455316502 * servo1_output - 0.4411568342 * servo3_output - 0.6502429234 * servo6_output - 0.0000672716 * v_yaw_deg_s * fabsf(v_yaw_deg_s) + 0.0088666014 * v_x * fabsf(v_x) - 0.0077023937 * v_y * fabsf(v_y) + 0.7213930293 * servo1_output * fabsf(servo1_output) + 20.9705308953 * servo3_output * fabsf(servo3_output) - 130.0485846752 * servo6_output * fabsf(servo6_output) + 0.0006194596 * v_yaw_deg_s * fabsf(v_x) - 0.0026734055 * v_yaw_deg_s * fabsf(v_y) - 0.0010742781 * v_yaw_deg_s * fabsf(servo1_output) - 0.0257308567 * v_yaw_deg_s * fabsf(servo3_output) + 0.2436988178 * v_yaw_deg_s * fabsf(servo6_output) - 0.0002040007 * v_x * fabsf(v_y) - 0.0153490950 * v_x * fabsf(servo3_output) + 94.2575886249 * v_x * fabsf(servo6_output) + 0.1005677203 * v_y * fabsf(servo1_output) + 0.0805001337 * v_y * fabsf(servo3_output) - 2.6043383301 * v_y * fabsf(servo6_output) - 0.6090922161 * servo1_output * fabsf(servo3_output) - 9.1063300438 * servo1_output * fabsf(servo6_output) - 88.2313668470 * servo3_output * fabsf(servo6_output) + 0.0001715758 * powf(v_yaw_deg_s, 2) - 0.0206917556 * powf(v_x, 2) - 0.0074700379 * powf(v_y, 2) - 0.1307424288 * powf(servo1_output, 2) + 0.3859444409 * powf(servo3_output, 2) + 130.0485846752 * powf(servo6_output, 2) + 0.0000087160 * powf(v_yaw_deg_s, 3) + 0.0009890791 * powf(v_x, 3) + 0.0009996821 * powf(v_y, 3) - 1.1264180334 * powf(servo1_output, 3) - 36.5005860301 * powf(servo3_output, 3) - 26009.7169368370 * powf(servo6_output, 3);
    // float starboard_speed_tp1 = 0.0012186617 * v_yaw_deg_s - 0.0010720336 * v_x + 0.4820522014 * v_y - 0.0510560946 * servo1_output + 0.0238960831 * servo3_output - 0.0000476661 * v_yaw_deg_s * fabsf(v_yaw_deg_s) - 0.0067437096 * v_x * fabsf(v_x) - 0.0051463783 * v_y * fabsf(v_y) + 0.4357296219 * servo1_output * fabsf(servo1_output) - 2.4866660468 * servo3_output * fabsf(servo3_output) - 0.0003776649 * v_yaw_deg_s * fabsf(v_x) + 0.0014462809 * v_yaw_deg_s * fabsf(v_y) - 0.0026763063 * v_yaw_deg_s * fabsf(servo1_output) + 0.0016902258 * v_yaw_deg_s * fabsf(servo3_output) + 0.2437323431 * v_yaw_deg_s * fabsf(servo6_output) - 0.0011703117 * v_x * fabsf(v_y) - 0.0669306517 * v_x * fabsf(servo3_output) - 0.2144067133 * v_x * fabsf(servo6_output) + 0.0561839699 * v_y * fabsf(servo1_output) + 0.1390013109 * v_y * fabsf(servo3_output) + 96.4104402734 * v_y * fabsf(servo6_output) + 0.0473917826 * servo1_output * fabsf(servo3_output) - 10.2112189198 * servo1_output * fabsf(servo6_output) + 4.7792166226 * servo3_output * fabsf(servo6_output) + 0.0101858154 * powf(v_x, 2) + 0.0003884389 * powf(v_y, 2) - 0.0344011563 * powf(servo1_output, 2) + 2.2106219374 * powf(servo3_output, 2) - 0.0000016791 * powf(v_yaw_deg_s, 3) - 0.0001713849 * powf(v_x, 3) + 0.0004106642 * powf(v_y, 3) - 0.4277868461 * powf(servo1_output, 3) + 2.0056869696 * powf(servo3_output, 3);

    printf("\n\n_____ Inputs: raw_servo1: %u, raw_servo3: %u, raw_servo6: %u\n", input.servos[0], input.servos[2], input.servos[5]);
    printf("_____ Input speeds: vx: %f, vy: %f, vyaw: %f\n", v_x, v_y, v_yaw_deg_s);
    printf("__ Delta time: %f\n", delta_time);

    printf("Inputs: Servo1: %f, Servo3: %f, Servo6: %f\n", servo1_output, servo3_output, servo6_output);
    printf("Yaw spd: %f, Forward spd: %f, Starboard spd: %f\n", v_yaw_deg_s_tp1, forward_speed_tp1, starboard_speed_tp1);

    fflush(stdout);

    gyro = Vector3f(0, 0, radians(v_yaw_deg_s_tp1));


    // update attitude
    dcm.rotate(gyro * delta_time);
    dcm.normalize();

    // These equations were trained on a timestep of 0.25s, so we need to scale this timestep to match
    float accel_x = (forward_speed_tp1 - v_x) / (delta_time/0.25f); // + sin(radians(yaw_rate)) * v_y;
    float accel_y = (starboard_speed_tp1 - v_y) / (delta_time/0.25f) + radians(v_yaw_deg_s) * v_x; // cos(radians) ~= radians
    accel_body = Vector3f(accel_x, accel_y, 0);
}

/*
  update the ackermann or skid rover simulation by one time step
 */
void SimRover::update_ackermann_or_skid(const struct sitl_input &input, float delta_time)
{


    // speed in m/s in body frame
    Vector3f velocity_body = dcm.transposed() * velocity_ef;

    // Current velocities for dynamics calculations
    float v_x = velocity_body.x;  // forward velocity
    float v_y = velocity_body.y;  // starboard velocity  
    float v_yaw_deg_s = degrees(gyro.z); // current yaw rate in deg/s

    // Check for NaN and set to 0 if needed
    if (isnan(v_x) || !isfinite(v_x)) {
        v_x = 0.0f;
    }
    if (isnan(v_y) || !isfinite(v_y)) {
        v_y = 0.0f;
    }
    if (isnan(v_yaw_deg_s) || !isfinite(v_yaw_deg_s)) {
        v_yaw_deg_s = 0.0f;
    }

    float servo1_output = (input.servos[0] - 1500.0f)/1000.0f; // steering
    float servo3_output = (input.servos[2] - 1500.0f)/1000.0f;
    float servo6_output = (input.servos[5] - 1000.0f)/1000.0f;

    // Custom dynamics equations  
    float v_yaw_deg_s_tp1 = 0.8788459312 * v_yaw_deg_s - 3.9296105163 * servo3_output - 49.3558109408 * servo1_output*servo3_output;
    float forward_speed_tp1 = 0.9600673480 * v_x + 0.8431726738 * servo3_output;
    float starboard_speed_tp1 = 0.9979987959 * v_y + 0.2795478102 * servo1_output*servo3_output;

    // float v_yaw_deg_s_tp1 = 0.4916564184 * v_yaw_deg_s - 0.0377095101 * v_x - 0.0173249737 * v_y - 0.5474767805 * servo1_output - 1.7479285367 * servo3_output - 1.4322567509 * servo6_output - 0.0030811745 * v_yaw_deg_s * fabsf(v_yaw_deg_s) + 0.7417252685 * v_x * fabsf(v_x) + 0.0307017345 * v_y * fabsf(v_y) + 2.7803675978 * servo1_output * fabsf(servo1_output) + 39.6328582426 * servo3_output * fabsf(servo3_output) - 286.4513501710 * servo6_output * fabsf(servo6_output) - 0.0171158803 * v_yaw_deg_s * fabsf(v_x) + 0.0043611697 * v_yaw_deg_s * fabsf(v_y) + 0.1988914066 * v_yaw_deg_s * fabsf(servo1_output) + 98.3312836846 * v_yaw_deg_s * fabsf(servo6_output) + 0.0320896254 * v_x * fabsf(v_y) + 0.3093921495 * v_x * fabsf(servo1_output) + 0.1161545418 * v_x * fabsf(servo3_output) - 7.5419020177 * v_x * fabsf(servo6_output) - 0.4583748473 * v_y * fabsf(servo1_output) + 0.0440219117 * v_y * fabsf(servo3_output) - 3.4649947405 * v_y * fabsf(servo6_output) - 14.9576128441 * servo1_output * fabsf(servo3_output) - 109.4953560999 * servo1_output * fabsf(servo6_output) - 349.5857073488 * servo3_output * fabsf(servo6_output) - 0.0001233248 * powf(v_yaw_deg_s, 2) - 0.7460009872 * powf(v_x, 2) - 0.0640987752 * powf(v_y, 2) - 21.7940955738 * powf(servo3_output, 2) + 286.4513501731 * powf(servo6_output, 2) + 0.0000544364 * powf(v_yaw_deg_s, 3) + 0.0002939413 * powf(v_x, 3) - 0.0110899243 * powf(v_y, 3) + 5.4802165574 * powf(servo1_output, 3) - 32.4357758347 * powf(servo3_output, 3) - 57290.2700370548 * powf(servo6_output, 3);
    // float forward_speed_tp1 = 0.0012184941 * v_yaw_deg_s + 0.4712879431 * v_x - 0.0130216917 * v_y - 0.0455316502 * servo1_output - 0.4411568342 * servo3_output - 0.6502429234 * servo6_output - 0.0000672716 * v_yaw_deg_s * fabsf(v_yaw_deg_s) + 0.0088666014 * v_x * fabsf(v_x) - 0.0077023937 * v_y * fabsf(v_y) + 0.7213930293 * servo1_output * fabsf(servo1_output) + 20.9705308953 * servo3_output * fabsf(servo3_output) - 130.0485846752 * servo6_output * fabsf(servo6_output) + 0.0006194596 * v_yaw_deg_s * fabsf(v_x) - 0.0026734055 * v_yaw_deg_s * fabsf(v_y) - 0.0010742781 * v_yaw_deg_s * fabsf(servo1_output) - 0.0257308567 * v_yaw_deg_s * fabsf(servo3_output) + 0.2436988178 * v_yaw_deg_s * fabsf(servo6_output) - 0.0002040007 * v_x * fabsf(v_y) - 0.0153490950 * v_x * fabsf(servo3_output) + 94.2575886249 * v_x * fabsf(servo6_output) + 0.1005677203 * v_y * fabsf(servo1_output) + 0.0805001337 * v_y * fabsf(servo3_output) - 2.6043383301 * v_y * fabsf(servo6_output) - 0.6090922161 * servo1_output * fabsf(servo3_output) - 9.1063300438 * servo1_output * fabsf(servo6_output) - 88.2313668470 * servo3_output * fabsf(servo6_output) + 0.0001715758 * powf(v_yaw_deg_s, 2) - 0.0206917556 * powf(v_x, 2) - 0.0074700379 * powf(v_y, 2) - 0.1307424288 * powf(servo1_output, 2) + 0.3859444409 * powf(servo3_output, 2) + 130.0485846752 * powf(servo6_output, 2) + 0.0000087160 * powf(v_yaw_deg_s, 3) + 0.0009890791 * powf(v_x, 3) + 0.0009996821 * powf(v_y, 3) - 1.1264180334 * powf(servo1_output, 3) - 36.5005860301 * powf(servo3_output, 3) - 26009.7169368370 * powf(servo6_output, 3);
    // float starboard_speed_tp1 = 0.0012186617 * v_yaw_deg_s - 0.0010720336 * v_x + 0.4820522014 * v_y - 0.0510560946 * servo1_output + 0.0238960831 * servo3_output - 0.0000476661 * v_yaw_deg_s * fabsf(v_yaw_deg_s) - 0.0067437096 * v_x * fabsf(v_x) - 0.0051463783 * v_y * fabsf(v_y) + 0.4357296219 * servo1_output * fabsf(servo1_output) - 2.4866660468 * servo3_output * fabsf(servo3_output) - 0.0003776649 * v_yaw_deg_s * fabsf(v_x) + 0.0014462809 * v_yaw_deg_s * fabsf(v_y) - 0.0026763063 * v_yaw_deg_s * fabsf(servo1_output) + 0.0016902258 * v_yaw_deg_s * fabsf(servo3_output) + 0.2437323431 * v_yaw_deg_s * fabsf(servo6_output) - 0.0011703117 * v_x * fabsf(v_y) - 0.0669306517 * v_x * fabsf(servo3_output) - 0.2144067133 * v_x * fabsf(servo6_output) + 0.0561839699 * v_y * fabsf(servo1_output) + 0.1390013109 * v_y * fabsf(servo3_output) + 96.4104402734 * v_y * fabsf(servo6_output) + 0.0473917826 * servo1_output * fabsf(servo3_output) - 10.2112189198 * servo1_output * fabsf(servo6_output) + 4.7792166226 * servo3_output * fabsf(servo6_output) + 0.0101858154 * powf(v_x, 2) + 0.0003884389 * powf(v_y, 2) - 0.0344011563 * powf(servo1_output, 2) + 2.2106219374 * powf(servo3_output, 2) - 0.0000016791 * powf(v_yaw_deg_s, 3) - 0.0001713849 * powf(v_x, 3) + 0.0004106642 * powf(v_y, 3) - 0.4277868461 * powf(servo1_output, 3) + 2.0056869696 * powf(servo3_output, 3);

    printf("\n\n_____ Inputs: raw_servo1: %u, raw_servo3: %u, raw_servo6: %u\n", input.servos[0], input.servos[2], input.servos[5]);
    printf("_____ Input speeds: vx: %f, vy: %f, vyaw: %f\n", v_x, v_y, v_yaw_deg_s);
    printf("__ Delta time: %f\n", delta_time);

    printf("Inputs: Servo1: %f, Servo3: %f, Servo6: %f\n", servo1_output, servo3_output, servo6_output);
    printf("Yaw spd: %f, Forward spd: %f, Starboard spd: %f\n", v_yaw_deg_s_tp1, forward_speed_tp1, starboard_speed_tp1);

    fflush(stdout);

    gyro = Vector3f(0, 0, radians(v_yaw_deg_s_tp1));


    // update attitude
    dcm.rotate(gyro * delta_time);
    dcm.normalize();

    // These equations were trained on a timestep of 0.25s, so we need to scale this timestep to match
    float accel_x = (forward_speed_tp1 - v_x) / (delta_time/0.25f); // + sin(radians(yaw_rate)) * v_y;
    float accel_y = (starboard_speed_tp1 - v_y) / (delta_time/0.25f) + radians(v_yaw_deg_s) * v_x; // cos(radians) ~= radians
    accel_body = Vector3f(accel_x, accel_y, 0);
}

/*
  update the omni3 rover simulation by one time step
 */
void SimRover::update_omni3(const struct sitl_input &input, float delta_time)
{
    // in omni3 mode the first three servos are motor speeds
    float motor1 = 2*((input.servos[0]-1000)/1000.0f - 0.5f);
    float motor2 = 2*((input.servos[1]-1000)/1000.0f - 0.5f);
    float motor3 = 2*((input.servos[2]-1000)/1000.0f - 0.5f);

    // use forward kinematics to calculate body frame velocity
    Vector3f wheel_ang_vel(
      motor1 * omni3_wheel_max_ang_vel,
      motor2 * omni3_wheel_max_ang_vel,
      motor3 * omni3_wheel_max_ang_vel
    );

    // derivation of forward kinematics for an Omni3Mecanum rover
    // A. Gfrerrer. "Geometry and kinematics of the Mecanum wheel",
    // Computer Aided Geometric Design 25 (2008) 784–791.
    // Retrieved from https://www.geometrie.tugraz.at/gfrerrer/publications/MecanumWheel.pdf.
    //
    // the frame is equilateral triangle
    //
    // d[i] = 0.18 m is distance from frame centre to each wheel 
    // r_w = 0.04725 m is the wheel radius.
    // delta = radians(-45) is angle of the roller to the direction of forward rotation
    // alpha[i] is the angle the wheel axis is rotated about the body z-axis
    // c[i] = cos(alpha[i] + delta)
    // s[i] = sin(alpha[i] + delta)
    //
    // wheel  d[i]  alpha[i]  a_x[i]   a_y[i]      c[i]      s[i]
    //     1  0.18   1.04719    0.09  0.15588  0.965925  0.258819
    //     2  0.18   3.14159   -0.18  0.0     -0.707106  0.707106
    //     3  0.18   5.23598    0.09 -0.15588 -0.258819 -0.965925
    //
    //  k = 1/(r_w * sin(delta)) = -29.930445 is a scale factor
    //
    // inverse kinematic matrix
    // M[i, 0] = k * c[i]
    // M[i, 1] = k * s[i]
    // M[i, 2] = k * (a_x[i] s[i] - a_y[i] c[i])
    //
    // forward kinematics matrix: Minv = M^-1
    constexpr Matrix3f Minv(
      -0.0215149, 0.01575, 0.0057649,
      -0.0057649, -0.01575, 0.0215149,
      0.0875, 0.0875, 0.0875);

    // twist - this is the target linear and angular velocity
    Vector3f twist = Minv * wheel_ang_vel;

    // speed in m/s in body frame
    Vector3f velocity_body = dcm.transposed() * velocity_ef;

    // linear acceleration in m/s/s - very crude model
    float accel_x = omni3_max_accel * (twist.x - velocity_body.x) / omni3_max_speed;
    float accel_y = omni3_max_accel * (twist.y - velocity_body.y) / omni3_max_speed;

    gyro = Vector3f(0, 0, twist.z);

    // update attitude
    dcm.rotate(gyro * delta_time);
    dcm.normalize();

    // accel in body frame due to motors (excluding gravity)
    accel_body = Vector3f(accel_x, accel_y, 0);
}

} // namespace SITL
