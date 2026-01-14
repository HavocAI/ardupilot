#![no_std]


use core::mem::MaybeUninit;

mod physics;
mod kalman_filter;
mod rampage;

type Float = f32;

use panic_abort as _;

static mut INSTANCE: MaybeUninit<kalman_filter::MotorboatDynamicsKalmanFilter> = MaybeUninit::uninit();

#[unsafe(no_mangle)]
pub extern "C" fn boatekf_init(dt: Float) {
    let model = rampage::create_motorboat_model();
    let ekf = kalman_filter::MotorboatDynamicsKalmanFilter::new(dt, model);
    unsafe {
        let instance = &mut * &raw mut INSTANCE;
        instance.write(ekf);
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn boatekf_update_origin() {
    unsafe {
        let instance = &mut * &raw mut INSTANCE;
        let ekf = instance.assume_init_mut();
        ekf.update_origin();
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn boatekf_predict(rudder: Float, throttle: Float) {
    unsafe {
        let instance = &mut * &raw mut INSTANCE;
        let ekf = instance.assume_init_mut();
        ekf.predict(rudder, throttle);
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn boatekf_update_gps(north: Float, east: Float, var: Float) {
    unsafe {
        let instance = &mut * &raw mut INSTANCE;
        let ekf = instance.assume_init_mut();
        ekf.update_gps(north, east, var);
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn boatekf_update_compass(heading: Float, var: Float) {
    unsafe {
        let instance = &mut * &raw mut INSTANCE;
        let ekf = instance.assume_init_mut();
        ekf.update_compass(heading, var);
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn boatekf_get_position(north: *mut Float, east: *mut Float) {
    unsafe {
        let instance = &*&raw const INSTANCE;
        let ekf = instance.assume_init_ref();
        let pos = ekf.pos();
        *north = pos[0];
        *east = pos[1];
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn boatekf_get_position_variance() -> Float {
    unsafe {
        let instance = &*&raw const INSTANCE;
        let ekf = instance.assume_init_ref();
        ekf.pos_variance()
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn boatekf_get_heading() -> Float {
    unsafe {
        let instance = &*&raw const INSTANCE;
        let ekf = instance.assume_init_ref();
        ekf.theta()
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn boatekf_get_theta_variance() -> Float {
    unsafe {
        let instance = &*&raw const INSTANCE;
        let ekf = instance.assume_init_ref();
        ekf.theta_variance()
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn boatekf_get_velocity(north: *mut Float, east: *mut Float) {
    unsafe {
        let instance = &*&raw const INSTANCE;
        let ekf = instance.assume_init_ref();
        let vel = ekf.velocity();
        *north = vel[0];
        *east = vel[1];
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn boatekf_get_speed_variance() -> Float {
    unsafe {
        let instance = &*&raw const INSTANCE;
        let ekf = instance.assume_init_ref();
        ekf.speed_variance()
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn boatekf_get_wind(north: *mut Float, east: *mut Float) {
    unsafe {
        let instance = &*&raw const INSTANCE;
        let ekf = instance.assume_init_ref();
        let wind = ekf.wind_velocity();
        *north = wind[0];
        *east = wind[1];
    }
}

