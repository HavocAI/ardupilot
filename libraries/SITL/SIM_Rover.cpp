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

#include <stdio.h>
#include <string.h>

#include <AP_Math/AP_Math.h>

namespace SITL {

SimRover::SimRover(const char *frame_str) : Aircraft(frame_str) {
  skid_steering = strstr(frame_str, "skid") != nullptr;

  if (skid_steering) {
    printf("SKID Steering Rover Simulation Started\n");
    // these are taken from a 6V wild thumper with skid steering,
    // with a sabertooth controller
    max_accel = 14;
    max_speed = 4;
    return;
  }

  vectored_thrust = strstr(frame_str, "vector") != nullptr;
  if (vectored_thrust) {
    printf("Vectored Thrust (Rampage gen2) Rover Simulation Started\n");
  }

  omni3 = strstr(frame_str, "omni3mecanum") != nullptr;
  if (omni3) {
    printf("Omni3 Mecanum Rover Simulation Started\n");
  }

  lock_step_scheduled = true;
}

/*
  return turning circle (diameter) in meters for steering angle proportion in
  degrees
*/
float SimRover::turn_circle(float steering) const {
  if (fabsf(steering) < 1.0e-6) {
    return 0;
  }
  return turning_circle * sinf(radians(max_wheel_turn)) /
         sinf(radians(steering * max_wheel_turn));
}

/*
   return yaw rate in degrees/second given steering_angle and speed
*/
float SimRover::calc_yaw_rate(float steering, float speed) {
  if (skid_steering) {
    return constrain_float(steering * skid_turn_rate, -MAX_YAW_RATE,
                           MAX_YAW_RATE);
  }
  if (vectored_thrust) {
    return constrain_float(steering * vectored_turn_rate_max, -MAX_YAW_RATE,
                           MAX_YAW_RATE);
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
float SimRover::calc_lat_accel(float steering_angle, float speed) {
  float yaw_rate = calc_yaw_rate(steering_angle, speed);
  float accel = radians(yaw_rate) * speed;
  return accel;
}

/*
  update the rover simulation by one time step
 */
void SimRover::update(const struct sitl_input &input) {
  // how much time has passed?
  float delta_time = frame_time_us * 1.0e-6f;

  // update gyro and accel_body according to frame type
  if (omni3) {
    update_omni3(input, delta_time);
  } else {
    update_ackermann_or_skid(input, delta_time);
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

/*
  update the ackermann or skid rover simulation by one time step
 */
void SimRover::update_ackermann_or_skid(const struct sitl_input &input,
                                        float delta_time) {

  // speed in m/s in body frame
  Vector3f velocity_body = dcm.transposed() * velocity_ef;

  // Current velocities for dynamics calculations
  float v_x = velocity_body.x;         // forward velocity
  float v_y = velocity_body.y;         // starboard velocity
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

  float servo1_output = -(input.servos[0] - 1500.0f) / 1000.0f; // steering
  float servo3_output = (input.servos[2] - 1500.0f) / 1000.0f;  // throttle

  // THRUST
  double advance_coefficient = 0.95;
  double propeller_rpm = servo3_output * 5000;
  double va =
      v_x *
      advance_coefficient; // Rotational speed (n) in revolutions per second
  double rps = propeller_rpm / 60.0;
  if (fabsf(rps) < 1e-2)
    rps = 0.0;
  // Propeller diameter (d) in meters
  double d = 0.3; // advance ratio = j
  double j = va / (rps * d);
  double Kt = -0.091 * std::pow(j, 2) - 0.289 * j + 0.367; // thrust coefficient
  double rho = 1025; // saltwater density in kg/m^3
  double thrust = Kt * rho * std::pow(rps, 2) * std::pow(d, 4);

  double thruster_angle = servo1_output * 30; // +/- 30 deg max output
  double thrust_fwd = thrust * cos(radians(thruster_angle));
  double thrust_side = thrust * sin(radians(thruster_angle));
  double thrust_yaw = thrust_side * 1; // 1 meter between CG and motor pivot

  // DRAG
  // 3x1 vector of drag coefficients, 3x3 matrix of cross-coupling drag
  // coefficients
  Vector3f drag_coeffs(1.101, 4.673, 2.0001); // linear drag coefficients
  Matrix3f drag_coupling; // cross-coupling drag coefficients
  drag_coupling.zero();
  drag_coupling[0][0] = 2.012f;
  drag_coupling[1][1] = 93.462f;
  drag_coupling[2][2] = 50.0001f;
  drag_coupling[0][1] = 28.241f;
  drag_coupling[1][0] = 131.587f;
  double f_drag_x = drag_coeffs[0] * v_x +
                    drag_coupling[0][0] * v_x * fabsf(v_x) +
                    drag_coupling[0][1] * v_x * fabsf(v_y) +
                    drag_coupling[0][2] * v_x * fabsf(v_yaw_deg_s);
  double f_drag_y = drag_coeffs[1] * v_y +
                    drag_coupling[1][0] * v_y * fabsf(v_x) +
                    drag_coupling[1][1] * v_y * fabsf(v_y) +
                    drag_coupling[1][2] * v_y * fabsf(v_yaw_deg_s);
  double f_drag_yaw = drag_coeffs[2] * v_yaw_deg_s +
                      drag_coupling[2][0] * v_yaw_deg_s * fabsf(v_x) +
                      drag_coupling[2][1] * v_yaw_deg_s * fabsf(v_y) +
                      drag_coupling[2][2] * v_yaw_deg_s * fabsf(v_yaw_deg_s);

  // INERTIA
  float vehicle_mass = 257.208;
  float Lzz = 39.284389916;

  // Accelerations
  double a_x = (thrust_fwd - f_drag_x) / vehicle_mass;
  double a_y = (thrust_side - f_drag_y) / vehicle_mass;
  double a_yaw = (thrust_yaw - f_drag_yaw) / Lzz;

  // Custom dynamics equations
  // float v_yaw_deg_s_tp1 = 0.8788459312 * v_yaw_deg_s - 3.9296105163 *
  // servo3_output - 49.3558109408 * servo1_output*servo3_output; float
  // forward_speed_tp1 = 0.9600673480 * v_x + 0.8431726738 * servo3_output;
  // float starboard_speed_tp1 = 0.9579987959 * v_y + 0.2795478102 *
  // servo1_output*servo3_output;

  // Including x|x| drag terms
  double v_yaw_deg_s_tp1 = v_yaw_deg_s + a_yaw * (delta_time);
  //   float forward_speed_tp1 = forward_speed + a_x * (delta_time);
  //   float starboard_speed_tp1 = starboard_speed + a_y * (delta_time);

  // printf("Inputs: Servo1: %f, Servo3: %f, Servo6: %f\n", servo1_output,
  // servo3_output, servo6_output);
  printf("Yaw spd: %f, Forward spd: %f, Starboard spd: %f\n", v_x, v_y,
         v_yaw_deg_s);

  // fflush(stdout);

  gyro = Vector3f(0, 0, radians(v_yaw_deg_s_tp1));

  // update attitude
  dcm.rotate(gyro * delta_time);
  dcm.normalize();

  // These equations were trained on a timestep of 0.25s, so we need to scale
  // this timestep to match also: The rotation of the coordinate frame is
  // already learned in these dynamics, so no radians(yaw) * v_x
  //   float accel_x = (forward_speed_tp1 - v_x) /
  //                   (delta_time / 0.25f); // + sin(radians(yaw_rate)) * v_y;
  //   float accel_y =
  //       (starboard_speed_tp1 - v_y) /
  //       (delta_time /
  //        0.25f); // + radians(v_yaw_deg_s) * v_x; // cos(radians) ~= radians
  accel_body = Vector3f(a_x, a_y, 0);
}

/*
  update the omni3 rover simulation by one time step
 */
void SimRover::update_omni3(const struct sitl_input &input, float delta_time) {
  // in omni3 mode the first three servos are motor speeds
  float motor1 = 2 * ((input.servos[0] - 1000) / 1000.0f - 0.5f);
  float motor2 = 2 * ((input.servos[1] - 1000) / 1000.0f - 0.5f);
  float motor3 = 2 * ((input.servos[2] - 1000) / 1000.0f - 0.5f);

  // use forward kinematics to calculate body frame velocity
  Vector3f wheel_ang_vel(motor1 * omni3_wheel_max_ang_vel,
                         motor2 * omni3_wheel_max_ang_vel,
                         motor3 * omni3_wheel_max_ang_vel);

  // derivation of forward kinematics for an Omni3Mecanum rover
  // A. Gfrerrer. "Geometry and kinematics of the Mecanum wheel",
  // Computer Aided Geometric Design 25 (2008) 784–791.
  // Retrieved from
  // https://www.geometrie.tugraz.at/gfrerrer/publications/MecanumWheel.pdf.
  //
  // the frame is equilateral triangle
  //
  // d[i] = 0.18 m is distance from frame centre to each wheel
  // r_w = 0.04725 m is the wheel radius.
  // delta = radians(-45) is angle of the roller to the direction of forward
  // rotation alpha[i] is the angle the wheel axis is rotated about the body
  // z-axis c[i] = cos(alpha[i] + delta) s[i] = sin(alpha[i] + delta)
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
  constexpr Matrix3f Minv(-0.0215149, 0.01575, 0.0057649, -0.0057649, -0.01575,
                          0.0215149, 0.0875, 0.0875, 0.0875);

  // twist - this is the target linear and angular velocity
  Vector3f twist = Minv * wheel_ang_vel;

  // speed in m/s in body frame
  Vector3f velocity_body = dcm.transposed() * velocity_ef;

  // linear acceleration in m/s/s - very crude model
  float accel_x =
      omni3_max_accel * (twist.x - velocity_body.x) / omni3_max_speed;
  float accel_y =
      omni3_max_accel * (twist.y - velocity_body.y) / omni3_max_speed;

  gyro = Vector3f(0, 0, twist.z);

  // update attitude
  dcm.rotate(gyro * delta_time);
  dcm.normalize();

  // accel in body frame due to motors (excluding gravity)
  accel_body = Vector3f(accel_x, accel_y, 0);
}

} // namespace SITL
