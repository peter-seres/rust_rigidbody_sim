extern crate nalgebra as na;
use crate::quaternion;
use std::fmt;


pub struct RigidBody
{
    // Parameters:
    pub m: f32,
    pub inertia: na::Matrix3<f32>,
    inertia_inverse: na::Matrix3<f32>,

    // States:
    pub position: na::Vector3<f32>,
    pub velocity: na::Vector3<f32>,
    pub orientation: na::Vector4<f32>,
    pub angular_velocity : na::Vector3<f32>,
}

impl RigidBody 
{
    pub fn build_default() -> RigidBody 
    {
        let mass : f32 = 10.5;
        let inertia : na::Matrix3<f32> = na::Matrix3::identity();
        let inertia_inv = inertia.try_inverse();
        let j_inv: na::Matrix3<f32>;

        // Check if inverse exists: (set identity if not)
        match inertia_inv {
            Some(res) => { 
                j_inv = res;
            }
            None => {
                j_inv = na::Matrix3::identity();
                warn!("The following inertia matrix has no inverse: {}. Used Identity matrix for this simulation.", inertia);
            }
        }

        // Initial states are all zero:
        let rb = RigidBody{
            m: mass, 
            inertia: inertia, 
            inertia_inverse: j_inv,
            position: na::Vector3::zeros(), 
            velocity: na::Vector3::zeros(), 
            orientation: na::Vector4::new(1.0, 0.0, 0.0, 0.0),
            angular_velocity: na::Vector3::zeros()
        };

        return rb
    }

    // TODO: implement Runge-Kutta 4 integration!
    pub fn step(&mut self, forces: na::Vector3<f32>, moments: na::Vector3<f32>, dt_ms: f32) {
        let dt = dt_ms / 1000.0;

        // Linear kinematics:
        self.position += self.velocity * dt;

        // Linear Dynamics: 
        self.velocity += quaternion::rotate_vec(self.orientation, forces) * dt / self.m; 

        // Rotational Kinematics:
        self.orientation += 0.5 * quaternion::as_matrix(self.orientation) * quaternion::as_quat(self.angular_velocity) * dt;
        self.orientation.normalize_mut();

        // Rotational Dynamics:
        let angular_momentum = self.inertia * self.angular_velocity;
        let angular_acceleration =self.inertia_inverse * (moments - self.angular_velocity.cross(&angular_momentum));
        self.angular_velocity += angular_acceleration * dt;
    } 
}

impl fmt::Display for RigidBody 
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result 
    {
        write!(f, "RigidBody with mass: {} and inertia: {}", self.m, self.inertia)
    }
}