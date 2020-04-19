extern crate nalgebra as na;
use crate::quaternion;
use std::fmt;

// This module implements a 6DOF rigidbody struct with properties: mass and inertia matrix. The step method applies 3D force and moment vectors to the system.

struct StateDot {
    pos_dot: na::Vector3<f32>,
    vel_dot: na::Vector3<f32>,
    quat_dot: na::Vector4<f32>,
    omega_dot: na::Vector3<f32>,
}

// Todo: restructure dynamics call, to allow Runge Kutta integration:

// struct State {
//     pos: na::Vector3<f32>,
//     vel: na::Vector3<f32>,
//     quat: na::Vector4<f32>,
//     omega: na::Vector3<f32>,
// }

// struct Input {
//     forces: na::Vector3<f32>,
//     moments: na::Vector3<f32>,
// }


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
    pub fn new() -> RigidBody 
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
                println!("The following inertia matrix has no inverse: {}. Used Identity matrix for this simulation.", inertia);
            }
        }

        // Initial states are all zero:
        RigidBody{
            m: mass, 
            inertia, 
            inertia_inverse: j_inv,
            position: na::Vector3::zeros(), 
            velocity: na::Vector3::zeros(), 
            orientation: na::Vector4::new(1.0, 0.0, 0.0, 0.0),
            angular_velocity: na::Vector3::zeros()
        }
    }

    fn dynamics(&self, forces: na::Vector3<f32>, moments: na::Vector3<f32>) -> StateDot {

        // Translational kinematics:
        let pos_dot = self.velocity;

        // Translational Dynamics: 
        let vel_dot = quaternion::rotate_vec(self.orientation, forces) / self.m;

        // Rotational Kinematics:
        let quat_dot = quaternion::q_dot(self.orientation, self.angular_velocity);

        // Rotational Dynamics:
        let angular_momentum = self.inertia * self.angular_velocity;
        let omega_dot = self.inertia_inverse * (moments - self.angular_velocity.cross(&angular_momentum));

        StateDot{
            pos_dot,
            vel_dot,
            quat_dot,
            omega_dot
        }
    }

    fn euler_forward(&mut self, state_dot: StateDot, dt: f32) {
        self.position += state_dot.pos_dot * dt;
        self.velocity += state_dot.vel_dot * dt;
        self.orientation += state_dot.quat_dot * dt;
        self.angular_velocity += state_dot.omega_dot * dt;
    }

    pub fn step(&mut self, forces: na::Vector3<f32>, moments: na::Vector3<f32>, dt_ms: f32) {
        let dt = dt_ms / 1000.0;

        let state_dot = self.dynamics(forces, moments);

        self.euler_forward(state_dot, dt);
    } 
}

impl fmt::Display for RigidBody 
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result 
    {
        write!(f, "RigidBody with mass: {} and inertia: {}", self.m, self.inertia)
    }
}