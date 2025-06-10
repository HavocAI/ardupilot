#include "Rover.h"

bool ModeLoiter::_enter()
{
    // set _destination to reasonable stopping point
    if (!g2.wp_nav.get_stopping_location(_destination)) {
        return false;
    }

    // initialise desired speed to current speed
    if (!attitude_control.get_forward_speed(_desired_speed)) {
        _desired_speed = 0.0f;
    }

    // initialise heading to current heading
    _desired_yaw_cd = ahrs.yaw_sensor;

    return true;
}

void ModeLoiter::original_update()
{
    // get distance (in meters) to destination
    _distance_to_destination = rover.current_loc.get_distance(_destination);

    const float loiter_radius = g2.sailboat.tack_enabled() ? g2.sailboat.get_loiter_radius() : g2.loit_radius;

    // if within loiter radius slew desired speed towards zero and use existing desired heading
    if (_distance_to_destination <= loiter_radius) {
        // sailboats should not stop unless motoring
        const float desired_speed_within_radius = g2.sailboat.tack_enabled() ? 0.1f : 0.0f;
        _desired_speed = attitude_control.get_desired_speed_accel_limited(desired_speed_within_radius, rover.G_Dt);

        // if we have a sail but not trying to use it then point into the wind
        if (!g2.sailboat.tack_enabled() && g2.sailboat.sail_enabled()) {
            _desired_yaw_cd = degrees(g2.windvane.get_true_wind_direction_rad()) * 100.0f;
        }
    } else {
        // P controller with hard-coded gain to convert distance to desired speed
        _desired_speed = MIN((_distance_to_destination - loiter_radius) * g2.loiter_speed_gain, g2.wp_nav.get_default_speed());

        // calculate bearing to destination
        _desired_yaw_cd = rover.current_loc.get_bearing_to(_destination);
        float yaw_error_cd = wrap_180_cd(_desired_yaw_cd - ahrs.yaw_sensor);
        // if destination is behind vehicle, reverse towards it
        if ((fabsf(yaw_error_cd) > 9000 && g2.loit_type == 0) || g2.loit_type == 2) {
            _desired_yaw_cd = wrap_180_cd(_desired_yaw_cd + 18000);
            yaw_error_cd = wrap_180_cd(_desired_yaw_cd - ahrs.yaw_sensor);
            _desired_speed = -_desired_speed;
        }

        // reduce desired speed if yaw_error is large
        // 45deg of error reduces speed to 75%, 90deg of error reduces speed to 50%
        float yaw_error_ratio = 1.0f - constrain_float(fabsf(yaw_error_cd / 9000.0f), 0.0f, 1.0f) * 0.5f;
        _desired_speed *= yaw_error_ratio;
    }

    // 0 turn rate is no limit
    float turn_rate = 0.0;

    // make sure sailboats don't try and sail directly into the wind
    if (g2.sailboat.use_indirect_route(_desired_yaw_cd)) {
        _desired_yaw_cd = g2.sailboat.calc_heading(_desired_yaw_cd);
        if (g2.sailboat.tacking()) {
            // use pivot turn rate for tacks
            turn_rate = g2.wp_nav.get_pivot_rate();
        }
    }

    // run steering and throttle controllers
    calc_steering_to_heading(_desired_yaw_cd, turn_rate);
    calc_throttle(_desired_speed, true);

}

static float logistic(float x)
{
    // logistic function to smoothly transition from 0 to 1
    return 1.0f / (1.0f + expf(-x));
}

void ModeLoiter::new_update()
{
    Vector3f wind;
    if (!ahrs.wind_estimate(wind)) {
        original_update();
        return;
    }

    Vector2f distance_to_destination = _destination.get_distance_NE(rover.current_loc);
    _distance_to_destination = distance_to_destination.length();
    Vector2f body_distance = AP::ahrs().earth_to_body2D(distance_to_destination);

    float into_wind_angle_rad = atan2f(-wind.y, -wind.x);

    // modify the target angle to 'lean' towards the destination
    into_wind_angle_rad += constrain_float(body_distance.y * g2.loiter_angle_gain, -M_PI_4, M_PI_4);

    const float into_wind_angle_cd = degrees(into_wind_angle_rad) * 100.0f;
    const float into_wind_speed = MIN(body_distance.x * g2.loiter_speed_gain, g2.wp_nav.get_default_speed());

    // linearly interpolate between the wind direction and the destination bearing based on distance to destination
    const float angle_lerp = logistic(g2.loit_radius - _distance_to_destination);
    _desired_yaw_cd = angle_lerp * into_wind_angle_cd + (1.0f - angle_lerp) * rover.current_loc.get_bearing_to(_destination);
    _desired_speed = angle_lerp * into_wind_speed + (1.0f - angle_lerp) * g2.wp_nav.get_default_speed();


    // run steering and throttle controllers
    calc_steering_to_heading(_desired_yaw_cd);
    calc_throttle(_desired_speed, true);
}

void ModeLoiter::update()
{

    if (g2.loit_type <= 3) {
        // use original loiter code
        original_update();
    } else {
        
        // if (rover.is_boat()) {
            new_update();
        // }
        
    }
}

// get desired location
bool ModeLoiter::get_desired_location(Location& destination) const
{
    destination = _destination;
    return true;
}
