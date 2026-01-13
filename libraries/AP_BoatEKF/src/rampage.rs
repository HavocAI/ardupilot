/// Rampage is 16 feet long, converted to meters.
pub const LENGTH: f32 = 16.0 * 0.3048;

/// Rampage is about 6.5 feet wide, converted to meters.
pub const WIDTH: f32 = 6.5 * 0.3048;

/// Rampage is about 600lbs, converted to kg.
pub const MASS: f32 = 800.0 * 0.453592;

// about a 10KW motor w/80% efficient
pub const MOTOR_SCALER: f32 = 100.0 * 0.8;

/// Rampage moment of inertia, approximated as a rectangle.
pub const MOMENT_OF_INERTIA: f32 = MASS * (LENGTH * WIDTH * WIDTH * WIDTH) / 12.0;

pub const MOTOR_MOUNT_X: f32 = -3.0 * 0.3048;
pub const MOTOR_MOUNT_Y: f32 = 0.0;

pub const CENTER_OF_WATER_DRAG_X: f32 = -1.0 * 0.3048; // 1 feet from center of mass
pub const CENTER_OF_WATER_DRAG_Y: f32 = 0.0;

pub const WIND_CENTER_FORCE_X: f32 = 2.0 * 0.3048;
pub const WIND_CENTER_FORCE_Y: f32 = 0.0;

const WIND_DRAG_COEFFICIENT: f32 = 5.5;
const DRAG_COEFFICIENT: f32 = 0.42;
const ANGULAR_DRAG_COEFFICIENT: f32 = 400.0; // angular drag coefficient

pub const fn create_motorboat_model() -> crate::physics::motorboat_model::MotorboatModel {
    crate::physics::motorboat_model::MotorboatModel {
        mass: MASS,
        moment_of_inertia: MOMENT_OF_INERTIA,
        motor_mount_point: nalgebra::Vector2::new(MOTOR_MOUNT_X, MOTOR_MOUNT_Y),
        center_of_water_drag: nalgebra::Vector2::new(
            CENTER_OF_WATER_DRAG_X,
            CENTER_OF_WATER_DRAG_Y,
        ),
        wind_center_force: nalgebra::Vector2::new(WIND_CENTER_FORCE_X, WIND_CENTER_FORCE_Y),
        water_drag_coefficient: DRAG_COEFFICIENT,
        wind_drag_coefficient: WIND_DRAG_COEFFICIENT,
        angular_drag_coefficient: ANGULAR_DRAG_COEFFICIENT,
        motor_scaler: MOTOR_SCALER,
    }
}
