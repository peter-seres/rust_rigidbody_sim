extern crate nalgebra as na;
extern crate kiss3d;
#[macro_use] extern crate log;
use gilrs::{Gilrs, Axis, Event, Gamepad, EventType};
use std::fmt;
use std::time;
use std::thread;
use std::path::Path;


// Todo list:
// 1. refactor and organize
// 2. plotting / logging
// 3. add runge kutta dynamics


// For quaternion rotations:
fn as_matrix(q: na::Vector4<f32>) -> na::Matrix4<f32> {
    let mat = na::Matrix4::new(q[0], -q[1], -q[2], -q[3],
                               q[1],  q[0], -q[3],  q[2],
                               q[2],  q[3],  q[0], -q[1],
                               q[3], -q[2],  q[1],  q[0]);
    return mat;
}

// For inverse quaternion rotations:
fn as_inverse_matrix(q: na::Vector4<f32>) -> na::Matrix4<f32> {
    let mat = na::Matrix4::new(q[0],   q[1],  q[2],  q[3],
                               -q[1],  q[0],  q[3], -q[2],
                               -q[2], -q[3],  q[0],  q[1],
                               -q[3],  q[2],  -q[1],  q[0]);
    return mat;
}

// Prepend 0 to Vector3:
fn as_quat(v: na::Vector3<f32>) -> na::Vector4<f32> {
    let v_ext = na::Vector4::new(0.0, v[0], v[1], v[2]);
    return v_ext;
}

// Rotate vector p using quaternion:  p' = q p q.inv()
fn rotate_vec(q: na::Vector4<f32>, p: na::Vector3<f32>) -> na::Vector3<f32> {
    let p_extended = as_quat(p);
    let p_rot_extended = as_matrix(q) * (as_matrix(p_extended) * as_inverse_matrix(q));
    let p_rot = na::Vector3::new(p_rot_extended[1], p_rot_extended[2], p_rot_extended[3]);
    return p_rot;
}

// Rotate quaternion q_2 by q_1:
fn _rotate_quat(q_1: na::Vector4<f32>, q_2: na::Vector4<f32>) -> na::Vector4<f32> {
    let q = as_matrix(q_1) * q_2;
    return q;
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
    orientation: na::Vector4<f32>,
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
    fn step(&mut self, forces: na::Vector3<f32>, moments: na::Vector3<f32>, dt_ms: f32) {
        let dt = dt_ms / 1000.0;

        // Linear kinematics:
        self.position += self.velocity * dt;

        // Linear Dynamics: 
        self.velocity += rotate_vec(self.orientation, forces) * dt / self.m; 

        // Rotational Kinematics:
        self.orientation += 0.5 * as_matrix(self.orientation) * as_quat(self.angular_velocity) * dt;
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

// Time management: keeps track of loop times
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


const PI: f32 = 3.14159265359;

fn main() 
{
    // GUI Setup:
    let mut window  = kiss3d::window::Window::new("RigidBody Rust Demo by PS");
    window.set_light(kiss3d::light::Light::StickToCamera);
    let mut is_running = true;

    // Drone object setup:
    let obj_path = Path::new("assets/aera_low_poly.obj");
    let mtl_path = Path::new("assets/");
    let mut cube = window.add_obj(&obj_path, &mtl_path, na::Vector3::new(0.6, 0.6, 0.6));
    cube.set_color(0.6, 0.6, 0.93);
    let mut cube_trans: na::Translation3<f32>;
    let mut cube_rot: na::UnitQuaternion<f32>;

    // Ground plane setup:
    let mut quad = window.add_quad(13.0, 13.0, 10, 10);
    quad.set_color(0.3, 0.3, 0.3);
    quad.set_local_translation(na::Translation3::from(na::Vector3::new(0.0, -1.0, 0.0)));
    quad.set_local_rotation(na::UnitQuaternion::from_euler_angles(PI/2.0, 0., 0.));

    // Transformation matrices from NED to OpenGL coorinate system:
    let translation_transform: na::Matrix3<f32> = na::Matrix3::new(1., 0., 0., 0., 0., -1., 0., 1., 0.);
    let rotation_transform: na::Matrix4<f32> = na::Matrix4::new(0., 1., 0., 0., 0., 0., 0., -1., 0., 0., 1., 0., 1., 0., 0., 0.);

    // Rigidbody:
    let mut rb = RigidBody::build_default();

    // Game pad management:
    let mut gilrs = Gilrs::new().unwrap();
    let mut _gp: Option<Gamepad> = None;

    for (_id, gamepad) in gilrs.gamepads() {
        println!(" >>> Available input device: {}", gamepad.name());
        _gp = Some(gilrs.gamepad(gamepad.id()));
    }

    // Time management:
    let mut time_manager = TimeStep::new();

    let mut _throttle = 0.0;
    let mut roll = 0.0;
    let mut pitch = 0.0;
    let mut yaw = 0.0;

    while is_running
    {

        // Gamepad input:
        while let Some(Event { id: _, event, time: _ }) = gilrs.next_event() {
            match event {
                EventType::AxisChanged(Axis::LeftStickX, val, _) => {
                    println!("Left X {}", val);
                    yaw = val; },
                EventType::AxisChanged(Axis::LeftStickY, val, _) => {
                    println!("Right Y {}", val);
                    pitch = -val; },
                EventType::AxisChanged(Axis::RightStickX, val, _) => {
                    println!("Right X {}", val);
                    roll = val; },
                EventType::AxisChanged(Axis::RightStickY, val, _) => {
                    println!("Right Y {}", val);
                    pitch = -val; },
                EventType::AxisChanged(Axis::DPadX, _, _) => (),
                EventType::AxisChanged(Axis::DPadY, _, _) => (),
                EventType::AxisChanged(Axis::LeftZ, _, _) => (),
                EventType::AxisChanged(Axis::RightZ, _, _) => (),
                EventType::AxisChanged(Axis::Unknown, _, _) => (),
                EventType::ButtonPressed(_, _) => (),
                EventType::ButtonChanged(_, _, _) => (),
                EventType::ButtonRepeated(_, _) => (),
                EventType::ButtonReleased(_, _) => (),
                EventType::Connected => (),
                EventType::Disconnected => (),
                EventType::Dropped => (),
            }
        }

        // Let the Thread sleep for a bit:
        let sleepy_time = time::Duration::from_micros(500);
        thread::sleep(sleepy_time);
        let dt = time_manager.delta();

        // Apply forces and moments in the body frame:
        let forces = na::Vector3::new(0.0, 0.0, 0.0);
        let moments = na::Vector3::new(roll, pitch, yaw);

        // Rigid body step forward:
        rb.step(forces, moments, dt);

        // Render only every MS_PER_FRAME millisecond: 
        if time_manager.should_render() {

            // Move drone in the scene:
            let cube_cg = translation_transform * rb.position;
            cube_trans= na::Translation3::from(cube_cg);
            cube_rot = na::UnitQuaternion::from_quaternion(na::Quaternion::from(rotation_transform * rb.orientation));
            cube.set_local_translation(cube_trans);
            cube.set_local_rotation(cube_rot);

            // Call render:
            is_running = window.render();

            // Draw body frame axes:
            window.draw_line(&na::Point3::from(cube_cg), 
                             &na::Point3::from(cube_cg + translation_transform * (rotate_vec(rb.orientation, na::Vector3::new(2.1, 0., 0.)))),
                             &na::Point3::new(1., 0., 0.));
            
            window.draw_line(&na::Point3::from(cube_cg), 
                             &na::Point3::from(cube_cg + translation_transform * (rotate_vec(rb.orientation, na::Vector3::new(0., 3.2, 0.)))),
                             &na::Point3::new(0., 1., 0.));

            window.draw_line(&na::Point3::from(cube_cg), 
                             &na::Point3::from(cube_cg + translation_transform * (rotate_vec(rb.orientation, na::Vector3::new(0., 0., 1.2)))),
                             &na::Point3::new(0., 0., 1.));
        }
    }
}
