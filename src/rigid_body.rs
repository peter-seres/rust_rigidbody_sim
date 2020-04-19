extern crate nalgebra as na;
use crate::quaternion;
use std::fmt;
use std::ops;

// This module implements a 6DOF rigidbody struct with properties: mass and inertia matrix. The step method applies 3D force and moment vectors to the system.
#[derive(Copy, Clone)]
struct State {
    position: na::Vector3<f32>,
    velocity: na::Vector3<f32>,
    orientation: na::Vector4<f32>,
    angular_velocity : na::Vector3<f32>,
}

impl State {
    fn new () -> State {
        State{
            position: na::Vector3::zeros(), 
            velocity: na::Vector3::zeros(), 
            orientation: na::Vector4::new(1.0, 0.0, 0.0, 0.0),
            angular_velocity: na::Vector3::zeros()
        }
    }
}

impl ops::Mul<f32> for State {
    type Output = State;

    fn mul(self, _rhs: f32) -> State {
        State {
            position: self.position * _rhs,
            velocity: self.velocity * _rhs,
            orientation: self.orientation * _rhs,
            angular_velocity: self.angular_velocity * _rhs,
        }
    }
}

impl ops::Mul<State> for f32 {
    type Output = State;

    fn mul(self, _rhs: State) -> State {
        State {
            position: _rhs.position * self,
            velocity: _rhs.velocity * self,
            orientation: _rhs.orientation * self,
            angular_velocity: _rhs.angular_velocity * self,
        }
    }
}

impl ops::Add<State> for State {
    type Output = State;

    fn add(self, _rhs: State) -> State {
        State {
            position: self.position + _rhs.position,
            velocity: self.velocity + _rhs.velocity,
            orientation: self.orientation + _rhs.orientation,
            angular_velocity: self.angular_velocity + _rhs.angular_velocity,
        }
    }
}

#[derive(Copy, Clone)]
struct FMInput {
    forces: na::Vector3<f32>,
    moments: na::Vector3<f32>,
}

pub struct RigidBody
{
    // Parameters:
    pub m: f32,
    pub inertia: na::Matrix3<f32>,
    inertia_inverse: na::Matrix3<f32>,

    // States:
    state: State,
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

        let state = State::new();

        // Initial states are all zero:
        RigidBody{
            m: mass, 
            inertia, 
            inertia_inverse: j_inv,
            state,
        }
    }
    pub fn get_position(&self) -> na::Vector3<f32> {
        self.state.position
    }

    pub fn get_orientation(&self) -> na::Vector4<f32> {
        self.state.orientation
    }

    fn dynamics(&self, x: State, u: FMInput) -> State {

        // Translational kinematics:
        let pos_dot = x.velocity;

        // Translational Dynamics: 
        let vel_dot = quaternion::rotate_vec(x.orientation, u.forces) / self.m;

        // Rotational Kinematics:
        let quat_dot = quaternion::q_dot(x.orientation, x.angular_velocity);

        // Rotational Dynamics:
        let angular_momentum = self.inertia * x.angular_velocity;
        let omega_dot = self.inertia_inverse * (u.moments - x.angular_velocity.cross(&angular_momentum));

        State{
            position: pos_dot,
            velocity: vel_dot,
            orientation: quat_dot,
            angular_velocity: omega_dot
        }
    }

    fn rk4(&mut self, input: FMInput, dt: f32) -> State {

        let d1  = dt * self.dynamics(self.state, input);

        let d2_start = self.state + 0.5 * d1; 
        let d2  = dt * self.dynamics(d2_start, input);

        let d3_start = self.state + 0.5 * d2; 
        let d3  = dt * self.dynamics(d3_start, input);

        let d4_start = self.state + d3;
        let d4  = dt * self.dynamics(d4_start, input);

        let increment = (1.0 / 6.0) * (d1 + 2.0 * d2 + 2.0 * d3 + d4);
    
        self.state + increment
    }

    // Step the system forward using RK4
    pub fn step(&mut self, forces: na::Vector3<f32>, moments: na::Vector3<f32>, dt_ms: f32) {
        let dt = dt_ms / 1000.0;

        let input = FMInput{forces, moments};
        self.state = self.rk4(input, dt);
    } 
}


// Print statements:
impl fmt::Display for RigidBody 
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result 
    {
        write!(f, "RigidBody with mass: {} and inertia: {}", self.m, self.inertia)
    }
}

impl fmt::Display for State 
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result 
    {
        write!(f, "State with position: {} velocity {}, quaternion: {} and omega {}.", 
        self.position, self.velocity, self.orientation, self.angular_velocity)
    }
}