use nalgebra::{Rotation2, Vector2};

pub struct RigidBody2D<T> {
    pub position: Vector2<T>,
    pub orientation: T, // angle in radians
    pub velocity: Vector2<T>,
    pub angular_velocity: T,
    pub mass: T,
    pub moment_of_inertia: T,
}

impl<T> RigidBody2D<T>
where
    T: nalgebra::Field + Copy + core::fmt::Debug + 'static,
{
    pub fn apply_force(&mut self, force: Vector2<T>, dt: T) {
        let acceleration = force / self.mass;
        self.velocity += acceleration * dt;
    }

    pub fn apply_force_at_point(&mut self, force: Vector2<T>, point: Vector2<T>, dt: T) {
        self.apply_force(force, dt);
        let r = point - self.position;
        let torque = r.perp(&force);
        self.apply_torque(torque, dt);
    }

    pub fn apply_torque(&mut self, torque: T, dt: T) {
        let angular_acceleration = torque / self.moment_of_inertia;
        self.angular_velocity += angular_acceleration * dt;
    }

    pub fn update(&mut self, dt: T) {
        self.position += self.velocity * dt;
        self.orientation += self.angular_velocity * dt;
    }

    pub fn body_to_world(&self, local_point: Vector2<T>) -> Vector2<T>
    where
        T: nalgebra::SimdRealField,
    {
        let rotation = Rotation2::new(self.orientation);
        self.position + rotation * local_point
    }
}


pub fn polar_to_vector2<T>(magnitude: T, angle_rad: T) -> Vector2<T>
where T: nalgebra::ComplexField + Copy,
{
    Vector2::new(magnitude * angle_rad.cos(), magnitude * angle_rad.sin())
}
