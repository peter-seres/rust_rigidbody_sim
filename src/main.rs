extern crate nalgebra as na;

use gilrs::{Gilrs, Axis, Event, EventType};
use std::time;
use std::thread;

mod scene_manager;
mod quaternion;
mod time_manager;
mod rigid_body;

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
    let mut gilrs = Gilrs::new().unwrap();

    for (_id, gamepad) in gilrs.gamepads() {
        println!(" >>> Available input device: {}", gamepad.name());
    }

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
        while let Some(Event { id: _, event, time: _ }) = gilrs.next_event() {
            match event {
                EventType::AxisChanged(Axis::LeftStickX, val, _) => _yaw = val,
                EventType::AxisChanged(Axis::LeftStickY, val, _) => _throttle = val,
                EventType::AxisChanged(Axis::RightStickX, val, _) => _roll = val,
                EventType::AxisChanged(Axis::RightStickY, val, _) => _pitch = -val,
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
        let forces = na::Vector3::new(0., 0., 0.);
        let moments = na::Vector3::new(0., 0., 0.);

        // Rigid body step forward:
        rb.step(forces, moments, dt);

        // Render only every MS_PER_FRAME millisecond: 
        if time_manager.should_render() {

            // Call render:
            is_running = scene.render(&rb);
        }
    }
}
