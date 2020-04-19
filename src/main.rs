extern crate nalgebra as na;
extern crate rustc_serialize;
use rustc_serialize::json;
use std::time;
use std::thread;
mod quaternion;
mod rigid_body;
mod input_manager;
mod time_manager;
// mod scene_manager;


// Todo list:

// 1. plotting / logging
// 2. add runge kutta dynamics
// 3. Add rotor dynamics
// 4. Add mixer
// 5. Add attitude rate controller



#[derive(RustcEncodable)]
struct GodotStateUpdate {
    position: Vec<f32>,
    attitude: Vec<f32>,
}

impl GodotStateUpdate {
    fn new(pos: na::Vector3<f32>, att: na::Vector4<f32>) -> GodotStateUpdate {
        
        let position = GodotStateUpdate::transform_translation_to_scene(pos);
        let attitude = GodotStateUpdate::transform_rotation_to_scene(att);
        
        GodotStateUpdate {
            position,
            attitude
        }
    }

    fn to_json(&self) -> String {
        json::encode(self).unwrap()
    }

    fn transform_translation_to_scene(v: na::Vector3<f32>) -> Vec<f32> {
        vec![v[0], -v[2], v[1]]
    }
    
    fn transform_rotation_to_scene(q: na::Vector4<f32>) -> Vec<f32> {
       vec![q[1], -q[3], q[2], q[0]]
    }
}


fn main() 
{
    // Gui scene setup: (contains the main window, the uav SceneNode and the ground plane SceneNode)
    // let mut scene = scene_manager::MyScene::new();

    // Rigidbody: (contains all states and 6dof dynamics)
    let mut rb = rigid_body::RigidBody::new();

    // Game pad management (contains: manual control struct from joystick)
    let mut inputs = input_manager::ControllerInput::new();

    // Time management (keeps track of loop time and render time)
    let mut time_manager = time_manager::TimeStep::new();

    // Main Loop:
    loop {

        // Gamepad input:
        inputs.listen_to_inputs();

        // Let the Thread sleep for a bit:
        let sleepy_time = time::Duration::from_micros(1000);
        thread::sleep(sleepy_time);
        let dt = time_manager.delta();

        // Apply forces and moments in the body frame:
        let forces = na::Vector3::new(0., 0., 0.);
        let moments = na::Vector3::new(0., 0., inputs.manual_control.yaw);

        // Rigid body step forward:
        rb.step(forces, moments, dt);

        let state_update = GodotStateUpdate::new(rb.position, rb.orientation).to_json();

        // println!("Encoded json state_upstead string: {}", state_update);


        // // Render only every MS_PER_FRAME millisecond: 
        // if time_manager.should_render() {
        //     is_running = scene.render(&rb);
        // }
    }
}
