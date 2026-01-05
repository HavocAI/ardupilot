use std::time::Duration;


pub struct LsmMode {

}

impl LsmMode {
    pub fn new() -> Self {
        LsmMode {}
    }

    pub fn step(&mut self, dt: Duration) {

        

    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lsm_mode() {
        for i in 0..100 {
            let x = i as f64 / (100.0) * 2.0 * std::f64::consts::PI;
            let y = x.sin();

            // steer_mag values -1 ... +1
            let steer_mag = 1.0;

            let throttle_mag = 1.0;

            let steer_out = steer_mag * y.signum();
            let throttle = throttle_mag * y;

        }
    }
}