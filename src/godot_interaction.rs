use std::net::UdpSocket;
use rustc_serialize::json;
use crate::time_manager;
use crate::rigid_body;

#[derive(RustcEncodable)]
pub struct GodotStateUpdate {
    position: Vec<f32>,
    attitude: Vec<f32>,
}

impl GodotStateUpdate {
    pub fn new(pos: na::Vector3<f32>, att: na::Vector4<f32>) -> GodotStateUpdate {
        
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


pub struct GodotStreamUDP {
    godot_address: String,
    socket: UdpSocket,
}

impl GodotStreamUDP {

    const MS_PER_GODOT_PROCESS: f32 = 17.0;

    pub fn new(rust_address: String, godot_address: String) -> GodotStreamUDP {
        let socket = UdpSocket::bind(&rust_address).unwrap();

        GodotStreamUDP {
            godot_address,
            socket
        }
    }

    pub fn send(&self, rigid_body: &rigid_body::RigidBody, time_manager: &mut time_manager::TimeStep) {
        
        // Check if enought time has passed for a new UDP update (Godot renders every 17 ms)
        if time_manager.should_render(GodotStreamUDP::MS_PER_GODOT_PROCESS) {

            // Get the position and orientation from the rigid body reference:
            let state_update = GodotStateUpdate::new(rigid_body.get_position(), rigid_body.get_orientation());

            // Send the JSON packet
            match self.socket.send_to(state_update.to_json().as_bytes(), &self.godot_address) {
                Ok(_) => (),
                Err(e) => println!("Error while sending data to socket: {}", e)
            }  
        }
    }
}