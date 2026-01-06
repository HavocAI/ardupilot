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
pub extern "C" fn call_from_rust() {}

pub fn add(left: u64, right: u64) -> u64 {
    left + right
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        assert_eq!(result, 4);
    }
}
