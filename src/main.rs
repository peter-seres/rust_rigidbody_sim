extern crate nalgebra as na;
extern crate kiss3d;
use std::fmt;
use std::time;

// 2. xbox joystick input handling
// 3. plotting / logging

struct RigidBody
{
    // Parameters:
    m: f32,
    inertia: na::Matrix3<f32>,

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
        
        // Initial states are all zero:
        let rb = RigidBody{m: mass, inertia: inertia, position: na::Vector3::zeros(), 
                velocity: na::Vector3::zeros(), orientation: na::UnitQuaternion::identity(),
                angular_velocity: na::Vector3::zeros()};

        return rb
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
