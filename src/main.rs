extern crate nalgebra as na;
extern crate rustc_serialize;
use std::time;
use std::thread;
use std::string::String;
mod quaternion;
mod rigid_body;
mod input_manager;
mod time_manager;
mod godot_interaction;


// Todo list:
// 1. plotting / logging
// 2. switch euler forward to runge kutta integration
// 3. Add pwm command input -> Rotor dynamics

fn main() 
{
    // Rigidbody: (contains all states and 6dof dynamics)
    let mut rb = rigid_body::RigidBody::new();

    // Game pad management (contains: manual control struct from joystick)
    let mut inputs = input_manager::ControllerInput::new();

    // Time management (keeps track of loop time and render time)
    let mut time_manager = time_manager::TimeStep::new();

    // Godot interaction:
    let godot_connection = godot_interaction::GodotStreamUDP::new(String::from("127.0.0.1:5555"), String::from("127.0.0.1:12345"));

    // Main Loop:
    loop {

        // Gamepad input:
        inputs.listen_to_inputs();

        // Let the Thread sleep for a bit:
        let sleepy_time = time::Duration::from_micros(1000);
        thread::sleep(sleepy_time);
        let dt = time_manager.delta();

        // Apply forces and moments in the body frame:
        let forces = na::Vector3::new(200.0 * inputs.manual_control.throttle, 0., 0.);
        let moments = na::Vector3::new(inputs.manual_control.roll, 0., inputs.manual_control.yaw);

        // Rigid body step forward:
        rb.step(forces, moments, dt);

        // Position and attitude updates:
        godot_connection.send(&rb, &mut time_manager);
    }
}
