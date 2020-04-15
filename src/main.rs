extern crate nalgebra as na;

use std::time;
use std::thread;

mod quaternion;
mod rigid_body;
mod input_manager;
mod time_manager;
mod scene_manager;

// Todo list:

// 1. plotting / logging
// 2. add runge kutta dynamics
// 3. Add rotor dynamics
// 4. Add mixer
// 5. Add attitude rate controller


fn main() 
{
    // Gui scene setup: (contains the main window, the uav SceneNode and the ground plane SceneNode)
    let mut scene = scene_manager::MyScene::new();

    // Rigidbody:
    let mut rb = rigid_body::RigidBody::new();

    // Game pad management:
    let mut inputs = input_manager::ControllerInput::new();

    // Gamepad inputs:
    let mut _throttle = 0.0;
    let mut _roll = 0.0;
    let mut _pitch = 0.0;
    let mut _yaw = 0.0;

    // Time management:
    let mut time_manager = time_manager::TimeStep::new();

    // Main Loop:
    let mut is_running = true;
    while is_running
    {
        // Gamepad input:
        inputs.listen_to_inputs();

        // Let the Thread sleep for a bit:
        let sleepy_time = time::Duration::from_micros(500);
        thread::sleep(sleepy_time);
        let dt = time_manager.delta();

        // Apply forces and moments in the body frame:
        let forces = na::Vector3::new(0., 0., 0.);
        let moments = na::Vector3::new(0., 0., inputs.manual_control.yaw);

        // Rigid body step forward:
        rb.step(forces, moments, dt);

        // Render only every MS_PER_FRAME millisecond: 
        if time_manager.should_render() {
            is_running = scene.render(&rb);
        }
    }
}
