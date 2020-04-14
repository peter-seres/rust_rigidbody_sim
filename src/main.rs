extern crate nalgebra as na;
extern crate kiss3d;
use std::fmt;
use std::time;

// 2. xbox joystick input handling
// 3. plotting / logging

type FP = f32;

fn get_q_as_matrix(q: &na::Vector4<f32>) -> na::Matrix4<f32> {

    let Q = na::Matrix4::new(q[0], -q[1], -q[2], -q[3],
                             q[1],  q[0], -q[3],  q[2],
                             q[2],  q[3],  q[0], -q[1],
                             q[3], -q[2],  q[1],  q[0]);

    return Q;
}

fn get_quaternion_derivative(q: &na::Vector4<f32>, omega: &na::Vector3<f32>) -> na::Vector4<f32> {
    let q_pqr = na::Vector4::new(0.0, omega[0], omega[1], omega[2]);
    let Q = get_q_as_matrix(q);
    let q_dot = 0.5 * Q * q_pqr;
    return q_dot;
}

fn get_quaternion_vector(q: na::UnitQuaternion<f32>) -> na::Vector4<f32>{
    let q_vec = q.as_vector();
    let q_vec_flipped = na::Vector4::new(q_vec[3], q[0], q[1], q[2]);
    return q_vec_flipped;
}

fn build_quaternion_from_vector(q_vec: na::Vector4<f32>) -> na::UnitQuaternion<f32> {
    let q = na::Quaternion::from(q_vec);
    let q_unit = na::UnitQuaternion::from_quaternion(q);
    return q_unit;
}


struct RigidBody
{
    // Parameters:
    m: f32,
    inertia: na::Matrix3<f32>,
    inertia_inverse: na::Matrix3<f32>,

    // States:
    position: na::Vector3<f32>,
    velocity: na::Vector3<f32>,
    orientation: na::UnitQuaternion<f32>,
    angular_velocity : na::Vector3<f32>, 
}

impl RigidBody 
{
    fn build_default() -> RigidBody 
    {
        let mass : f32 = 10.5;
        let inertia : na::Matrix3<f32> = na::Matrix3::identity();
        let inertia_inv = inertia.try_inverse();
        let j_inv: na::Matrix3<f32>;

        match inertia_inv {
            Some(res) => {
                j_inv = res;
            }
            None => {
                j_inv = na::Matrix3::identity();
            }
        }

        // Initial states are all zero:
        let rb = RigidBody{
            m: mass, 
            inertia: inertia, 
            inertia_inverse: j_inv,
            position: na::Vector3::zeros(), 
            velocity: na::Vector3::zeros(), 
            orientation: na::UnitQuaternion::identity(),
            angular_velocity: na::Vector3::zeros()
        };

        return rb
    }

    fn step(&mut self, F: na::Vector3<f32>, M: na::Vector3<f32>, dt_ms: f32) {
        let dt = dt_ms * 0.001;

        // Linear kinematics:
        self.position = self.position + self.velocity * dt;

        // Linear Dynamics: 
        self.velocity = self.orientation * F / self.m; 

        // Rotational Kinematics:
        let q = get_quaternion_vector(self.orientation);
        let q_dot = get_quaternion_derivative(&q, &self.angular_velocity);
        let new_q = q + q_dot * dt;
        self.orientation = build_quaternion_from_vector(new_q);

        // Rotational Dynamics:
        let angular_momentum = self.inertia * self.angular_velocity;
        self.angular_velocity = self.inertia_inverse * (M - self.angular_velocity.cross(&angular_momentum));
    } 
}

impl fmt::Display for RigidBody 
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result 
    {
        write!(f, "RigidBody with mass: {} and inertia: {}", self.m, self.inertia)
    }
}

struct TimeStep 
{
    last_time: time::Instant,
    delta_time: f32,
    last_render_time: time::Instant,
    render_dt: f32,
}

impl TimeStep {

    const MS_PER_FRAME: f32 = 16.7;

    fn new() -> TimeStep {
        TimeStep {
            last_time: time::Instant::now(),
            delta_time: 0.0,
            last_render_time: time::Instant::now(),
            render_dt: 0.0,
        }
    }

    fn delta(&mut self) -> f32 {
        let current_time = time::Instant::now();
        let delta = current_time.duration_since(self.last_time).as_millis() as f32;
        self.last_time = current_time;
        self.delta_time = delta;

        return delta;
    }

    fn should_render(&mut self) -> bool {
        let current_time = time::Instant::now();
        let delta_render = current_time.duration_since(self.last_render_time).as_millis() as f32;

        if delta_render >= TimeStep::MS_PER_FRAME {
            self.last_render_time = current_time;
            self.render_dt = delta_render;
            return true;
        } else {
            return false;
        }
    }
}


fn main() 
{
    // GUI Setup:
    let mut window  = kiss3d::window::Window::new("RigidBody Rust Demo by PS");
    let mut cube    = window.add_cube(1.0, 1.0, 1.0);
    window.set_light(kiss3d::light::Light::StickToCamera);
    cube.set_color(0.4, 0.4, 0.83);

    // Rigidbody:
    let _rb = RigidBody::build_default();

    // Time management:
    let mut time_manager = TimeStep::new();
    loop
    {
        let _dt = time_manager.delta();

        // Render only every MS_PER_FRAME millisecond: 
        if time_manager.should_render() {
            window.render();
        }
    }
}
