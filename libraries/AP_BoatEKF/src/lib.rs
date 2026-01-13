#![no_std]


use core::mem::MaybeUninit;

mod physics;
mod kalman_filter;
mod rampage;

type Float = f32;

use panic_abort as _;

static INSTANCE: MaybeUninit<kalman_filter::MotorboatDynamicsKalmanFilter> = MaybeUninit::uninit();

unsafe fn convert_mut<T>(ptr: *const T) -> *mut T {
    ptr as *mut T
}

#[unsafe(no_mangle)]
pub extern "C" fn boatekf_init(dt: Float) {
    let model = rampage::create_motorboat_model();
    let ekf = kalman_filter::MotorboatDynamicsKalmanFilter::new(dt, model);
    unsafe {
        // let instance = (&INSTANCE) as *const _ as *mut MaybeUninit<kalman_filter::MotorboatDynamicsKalmanFilter>;
        // instance.write(ekf);
        
        convert_mut(INSTANCE.as_ptr()).write(ekf);
        // INSTANCE.as_mut_ptr().write(ekf);
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn boat_ekf() -> *mut kalman_filter::MotorboatDynamicsKalmanFilter {
    unsafe { convert_mut(INSTANCE.as_ptr()) }
}

#[unsafe(no_mangle)]
pub extern "C" fn boatekf_predict(rudder: Float, throttle: Float) {
    unsafe {
        let ekf = &mut *convert_mut(INSTANCE.as_ptr());
        ekf.predict(rudder, throttle);
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn boatekf_update_gps(north: Float, east: Float, var: Float) {
    unsafe {
        let ekf = &mut *convert_mut(INSTANCE.as_ptr());
        ekf.update_gps(north, east, var);
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn boatekf_update_compass(heading: Float, var: Float) {
    unsafe {
        let ekf = &mut *convert_mut(INSTANCE.as_ptr());
        ekf.update_compass(heading, var);
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn boatekf_get_position(north: *mut Float, east: *mut Float) {
    unsafe {
        let ekf = &*INSTANCE.as_ptr();
        let pos = ekf.pos();
        *north = pos[0];
        *east = pos[1];
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn boatekf_get_velocity(north: *mut Float, east: *mut Float) {
    unsafe {
        let ekf = &*INSTANCE.as_ptr();
        let vel = ekf.velocity();
        *north = vel[0];
        *east = vel[1];
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn boatekf_get_wind(north: *mut Float, east: *mut Float) {
    unsafe {
        let ekf = &*INSTANCE.as_ptr();
        let wind = ekf.wind_velocity();
        *north = wind[0];
        *east = wind[1];
    }
}

