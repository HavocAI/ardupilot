use autodiff_emb::*;
use nalgebra::{SVector, SVectorView, Vector2, matrix};

type CovarianceMatrix = nalgebra::SMatrix<
    Float,
    { physics::motorboat_model::NUM_STATES },
    { physics::motorboat_model::NUM_STATES },
>;

use crate::{
    Float,
    physics::{
        self,
        motorboat_model::{STATE_ORIENTATION, STATE_POSITION_X, STATE_POSITION_Y},
    },
};


pub struct MotorboatDynamicsKalmanFilter {
    model: physics::motorboat_model::MotorboatModel,
    state_estimate: [Float; physics::motorboat_model::NUM_STATES],
    covariance_estimate: CovarianceMatrix,
    process_noise: CovarianceMatrix,
    dt: Float,
}


impl MotorboatDynamicsKalmanFilter {
    pub fn new(dt: Float, model: physics::motorboat_model::MotorboatModel) -> Self {
        Self {
            model,
            state_estimate: [0.0; physics::motorboat_model::NUM_STATES],
            covariance_estimate: CovarianceMatrix::identity(),
            process_noise: CovarianceMatrix::identity() * 0.1,
            dt,
        }
    }

    pub fn create() -> Self {
        let mut model = crate::rampage::create_motorboat_model();
        // model.mass *= 1.3;
        // model.moment_of_inertia *= 0.8;
        // model.water_drag_coefficient *= 1.3;
        // model.wind_drag_coefficient *= 0.8;
        // model.angular_drag_coefficient *= 1.2;
        // model.motor_scaler *= 2.3;

        Self::new(0.1, model)
    }

    
    pub fn pos(&self) -> [Float; 2] {
        [self.state_estimate[0], self.state_estimate[1]]
    }

    
    pub fn theta(&self) -> Float {
        self.state_estimate[2]
    }

    
    pub fn angular_velocity(&self) -> Float {
        self.state_estimate[3]
    }

    
    pub fn speed(&self) -> Float {
        let speed = Vector2::new(self.state_estimate[4], self.state_estimate[5]).norm();
        if speed.is_nan() { 0.0 } else { speed }
    }

    pub fn velocity(&self) -> [Float; 2] {
        [self.state_estimate[4], self.state_estimate[5]]
    }

    
    pub fn wind_velocity(&self) -> [Float; 2] {
        [self.state_estimate[6], self.state_estimate[7]]
    }

    
    pub fn predict(&mut self, rudder: Float, throttle: Float) {
        let f = |x: nalgebra::SVectorView<'_, Dual32, { physics::motorboat_model::NUM_STATES }>| {
            physics::motorboat_model::state_transition(&self.model, self.dt, rudder, throttle, x)
        };

        // let (x, jac) = num_dual::jacobian(f, self.state_estimate.into());

        let x: nalgebra::SVector<Float, { physics::motorboat_model::NUM_STATES }> =
            self.state_estimate.into();

        let jac = autodiff_emb::jacobian(f, x.as_view());

        let x_hat = physics::motorboat_model::state_transition(
            &self.model,
            self.dt,
            rudder,
            throttle,
            x.as_view(),
        );

        self.state_estimate = x_hat.into();

        self.covariance_estimate =
            jac * self.covariance_estimate * jac.transpose() + self.process_noise;
    }

    #[allow(non_snake_case)]
    
    pub fn update_gps(
        &mut self,
        gps_position_x: Float,
        gps_position_y: Float,
        gps_position_variance: Float,
    ) {
        let z_data = [gps_position_x, gps_position_y];
        let z = SVectorView::from_slice(&z_data);
        let R = nalgebra::Matrix2::<Float>::identity() * gps_position_variance.abs();

        let h = SVectorView::from_slice(&self.state_estimate[STATE_POSITION_X..=STATE_POSITION_Y]);

        let y = z - h;

        let H = matrix![
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        ];

        let S = H * self.covariance_estimate * H.transpose() + R;

        let K = self.covariance_estimate * H.transpose() * S.try_inverse().unwrap();

        let x = SVectorView::from_slice(&self.state_estimate);
        self.state_estimate = (x + K * y).into();
        self.covariance_estimate =
            (CovarianceMatrix::identity() - K * H) * self.covariance_estimate;
    }

    #[allow(non_snake_case)]
    
    pub fn update_compass(&mut self, compass_heading: Float, compass_heading_variance: Float) {
        let z_data = [compass_heading];
        let z = SVectorView::from_slice(&z_data);
        let R = nalgebra::Matrix1::<Float>::identity() * compass_heading_variance.abs();

        let h = SVectorView::from_slice(core::slice::from_ref(
            &self.state_estimate[STATE_ORIENTATION],
        ));

        let y = z - h;

        let H = matrix![
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        ];

        let S = H * self.covariance_estimate * H.transpose() + R;

        let K = self.covariance_estimate * H.transpose() * S.try_inverse().unwrap();

        let x = SVectorView::from_slice(&self.state_estimate);
        self.state_estimate = (x + K * y).into();
        self.covariance_estimate =
            (CovarianceMatrix::identity() - K * H) * self.covariance_estimate;
    }
}

#[cfg(test)]
mod tests {
    use crate::physics::motorboat_model::{STATE_POSITION_X, STATE_POSITION_Y, STATE_VELOCITY_X};

    use super::*;
    use assert_approx_eq::assert_approx_eq;

    #[test]
    fn test_kalman_predict() {
        let model = crate::rampage::create_motorboat_model();
        let mut kf = MotorboatDynamicsKalmanFilter::new(0.1, model);

        // kf.state_estimate[STATE_VELOCITY_X] = 3.0;

        let mut position = Vector2::new(0.0, 0.0);
        let velocity = Vector2::new(1.0, 0.0);

        for _ in 0..30 {
            for i in 0..10 {
                kf.predict(0.0, 0.0);

                println!("state estimate {} {:?}", i, kf.state_estimate);
            }

            position += velocity * 1.0;
            kf.update_gps(position.x, position.y, 1.0);

            println!("expected position: {:?}", position);
            println!("state estimate: {:?}", kf.state_estimate);
        }

        assert_approx_eq!(kf.state_estimate[STATE_POSITION_X], 3.0, 1e-2);
        assert_approx_eq!(kf.state_estimate[STATE_POSITION_Y], 0.0);
    }

    #[test]
    fn test_kalman_update_compass() {
        let model = crate::rampage::create_motorboat_model();
        let mut kf = MotorboatDynamicsKalmanFilter::new(0.1, model);

        kf.predict(0.0, 0.0);
        kf.predict(0.0, 0.0);
        kf.predict(0.0, 0.0);

        kf.update_compass(0.1, 0.01);

        // assert that the state estimate is finite numbers
        for x in kf.state_estimate.iter() {
            assert!(x.is_finite());
        }

        kf.predict(0.0, 0.0);

        // assert that the state estimate is finite numbers
        for x in kf.state_estimate.iter() {
            assert!(x.is_finite());
        }
    }
}
