#include "Rover.h"

void ModeAcro::update()
{
    // get speed forward
    float speed, desired_steering, throttle_out;
    if (!attitude_control.get_forward_speed(speed)) {
        float desired_throttle;
        // convert pilot stick input into desired steering and throttle
        get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);

        // if vehicle is balance bot, calculate actual throttle required for balancing
        if (rover.is_balancebot()) {
            rover.balancebot_pitch_control(desired_throttle);
        }

        throttle_out = desired_throttle * 0.01f;

    } else {
        float desired_speed;
        // convert pilot stick input into desired steering and speed
        get_pilot_desired_steering_and_speed(desired_steering, desired_speed);
        // Limit to SPEED_MAX parameter
        if (is_positive(g2.speed_max)) {
            if (fabsf(desired_speed) > g2.speed_max) {
                desired_speed = copysignf(g2.speed_max, desired_speed);
            }
        }
        throttle_out = attitude_control.get_throttle_out_speed(desired_speed, g2.motors.limit.throttle_lower, g2.motors.limit.throttle_upper, g.speed_cruise, g.throttle_cruise * 0.01f, rover.G_Dt);
    }

    float steering_out;

    // handle sailboats
    if (!is_zero(desired_steering)) {
        // steering input return control to user
        g2.sailboat.clear_tack();
    }
    if (g2.sailboat.tacking()) {
        // call heading controller during tacking

        steering_out = attitude_control.get_steering_out_heading(g2.sailboat.get_tack_heading_rad(),
                                                                 g2.wp_nav.get_pivot_rate(),
                                                                 g2.motors.limit.steer_left,
                                                                 g2.motors.limit.steer_right,
                                                                 rover.G_Dt);
    } else {
        // convert pilot steering input to desired turn rate in radians/sec
        const float target_turn_rate = (desired_steering / 4500.0f) * radians(g2.acro_turn_rate);

        // run steering turn rate controller and throttle controller
        steering_out = attitude_control.get_steering_out_rate(target_turn_rate,
                                                              g2.motors.limit.steer_left,
                                                              g2.motors.limit.steer_right,
                                                              rover.G_Dt);
    }

    throttle_out = constrain_float(throttle_out, -1.0f, 1.0f);
    steering_out = constrain_float(steering_out, -1.0f, 1.0f);


    const float throttle_sign = (throttle_out > 0) ? 1.0f : -1.0f;

    const float mixed_throttle = throttle_sign * sqrt(steering_out * steering_out + throttle_out * throttle_out);
    const float mixed_steering = throttle_sign * atan2(steering_out, throttle_out);

    
    g2.motors.set_steering(mixed_steering * 4500.0f, false);
    g2.motors.set_throttle(mixed_throttle * 100.0f);
}

bool ModeAcro::requires_velocity() const
{
    return !g2.motors.have_skid_steering();
}

// sailboats in acro mode support user manually initiating tacking from transmitter
void ModeAcro::handle_tack_request()
{
    g2.sailboat.handle_tack_request_acro();
}
