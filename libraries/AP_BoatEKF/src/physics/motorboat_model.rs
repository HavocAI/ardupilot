use nalgebra::{SVector, Vector2, VectorView};

use crate::physics::{
    AIR_DENSITY, WATER_DENSITY,
    rigidbody::{RigidBody2D, polar_to_vector2},
};

/// position x, position y,
/// orientation theta,
/// angular velocity a,
/// velocity x, velocity y,
/// wind vx, wind vy
pub const NUM_STATES: usize = 8;

pub const STATE_POSITION_X: usize = 0;
pub const STATE_POSITION_Y: usize = 1;
pub const STATE_ORIENTATION: usize = 2;
pub const STATE_ANGULAR_VELOCITY: usize = 3;
pub const STATE_VELOCITY_X: usize = 4;
pub const STATE_VELOCITY_Y: usize = 5;
pub const STATE_WIND_VELOCITY_X: usize = 6;
pub const STATE_WIND_VELOCITY_Y: usize = 7;

pub struct MotorboatModel {
    pub mass: f32,
    pub moment_of_inertia: f32,
    pub motor_mount_point: Vector2<f32>,
    pub center_of_water_drag: Vector2<f32>,
    pub wind_center_force: Vector2<f32>,
    pub water_drag_coefficient: f32,
    pub wind_drag_coefficient: f32,
    pub angular_drag_coefficient: f32,
    pub motor_scaler: f32,
}

impl MotorboatModel {}

pub fn state_transition<D: nalgebra::RealField + Copy + From<f32>>(
    model: &MotorboatModel,
    dt: f32,
    rudder: f32,
    throttle: f32,
    x: VectorView<'_, D, nalgebra::Const<NUM_STATES>>,
) -> SVector<D, NUM_STATES> {
    let dt = D::from(dt);
    let rudder = D::from(rudder);
    let throttle = D::from(throttle);
    let position = Vector2::new(x[0].clone(), x[1].clone());
    let orientation = x[2].clone();
    let angular_velocity = x[3].clone();
    let velocity = Vector2::new(x[4].clone(), x[5].clone());
    let wind_velocity = Vector2::new(x[6].clone(), x[7].clone());

    let mut rigid_body = RigidBody2D {
        position,
        orientation,
        velocity,
        angular_velocity,
        mass: model.mass.into(),
        moment_of_inertia: model.moment_of_inertia.into(),
    };

    let motor_force = polar_to_vector2(throttle * model.motor_scaler.into(), orientation - rudder);
    let motor_mount_point = rigid_body.body_to_world(Vector2::new(
        model.motor_mount_point.x.into(),
        model.motor_mount_point.y.into(),
    ));

    rigid_body.apply_force_at_point(motor_force, motor_mount_point, dt);

    let water_dynamic_pressure = velocity.norm_squared()
        * D::from(0.5)
        * D::from(WATER_DENSITY)
        * D::from(model.water_drag_coefficient);

    if water_dynamic_pressure > D::zero() {
        let drag_force = -velocity.normalize() * water_dynamic_pressure;
        let center_of_drag = rigid_body.body_to_world(Vector2::new(
            model.center_of_water_drag.x.into(),
            model.center_of_water_drag.y.into(),
        ));
        rigid_body.apply_force_at_point(drag_force, center_of_drag, dt);
    }

    let relative_wind_velocity = wind_velocity - velocity;
    let wind_dynamic_pressure = relative_wind_velocity.norm_squared()
        * D::from(0.5)
        * D::from(AIR_DENSITY)
        * D::from(model.wind_drag_coefficient);

    if wind_dynamic_pressure > D::zero() {
        let wind_force = relative_wind_velocity.normalize() * wind_dynamic_pressure;
        let center_of_drag = rigid_body.body_to_world(Vector2::new(
            model.wind_center_force.x.into(),
            model.wind_center_force.y.into(),
        ));
        rigid_body.apply_force_at_point(wind_force, center_of_drag, dt);
    }

    let angular_drag_torque = -D::one().copysign(rigid_body.angular_velocity)
        * (rigid_body.angular_velocity
            * rigid_body.angular_velocity
            * D::from(model.angular_drag_coefficient));
    rigid_body.apply_torque(angular_drag_torque, dt);

    rigid_body.update(dt);
    SVector::from([
        rigid_body.position[0],
        rigid_body.position[1],
        rigid_body.orientation,
        rigid_body.angular_velocity,
        rigid_body.velocity[0],
        rigid_body.velocity[1],
        wind_velocity[0],
        wind_velocity[1],
    ])
}
