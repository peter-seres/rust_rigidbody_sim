extern crate nalgebra as na;
extern crate kiss3d;
#[macro_use] extern crate log;
use gilrs::{Gilrs, Axis, Event, EventType};
use std::time;
use std::thread;
use std::path::Path;

mod quaternion;
mod time_manager;
mod rigid_body;

// Todo list:
// 1. plotting / logging
// 2. add runge kutta dynamics
// 3. Add rotor dynamics
// 4. Add mixer
// 5. Add attitude rate controller


struct MyScene {
    window: kiss3d::window::Window,
    uav: kiss3d::scene::SceneNode,
    _ground: kiss3d::scene::SceneNode,
}

impl MyScene {

    pub fn new() -> MyScene{
        
        // Setup of the main window:
        let mut window = kiss3d::window::Window::new("RigidBody Demo");
        window.set_light(kiss3d::light::Light::StickToCamera);

        // Load .obj file for UAV:
        let obj_path = Path::new("assets/aera_low_poly.obj");
        let mtl_path = Path::new("assets/");
        let mut uav = window.add_obj(&obj_path, &mtl_path, na::Vector3::new(0.6, 0.6, 0.6));
        uav.set_color(0.6, 0.6, 0.93);

        // Ground plane:
        let mut ground = window.add_quad(13.0, 13.0, 10, 10);
        ground.set_local_translation(na::Translation3::from(na::Vector3::new(0.0, -1.0, 0.0)));
        ground.set_local_rotation(na::UnitQuaternion::from_euler_angles(1.5708, 0., 0.));
        ground.set_color(0.3, 0.3, 0.3);

        let scene = MyScene{
            window: window,
            uav: uav,
            _ground: ground
        };

        return scene;
    }

    pub fn render(&mut self, rb: &rigid_body::RigidBody) -> bool {

        // Move the uav in the scene:
        let pos: na::Vector3<f32> = MyScene::to_scene_translation_transform(rb.position);
        self.uav.set_local_translation(na::Translation3::from(pos));

        // Rotate the uav in the scene:
        let rot_scene: na::Vector4<f32> = MyScene::to_scene_rotation_transform(rb.orientation);
        let rot: na::Quaternion<f32> = na::Quaternion::from(rot_scene);
        self.uav.set_local_rotation(na::UnitQuaternion::from_quaternion(rot));

        // Render command:
        let running: bool = self.window.render();
        return running
    }

    fn to_scene_translation_transform(v: na::Vector3<f32>) -> na::Vector3<f32> {
        let v_scene: na::Vector3<f32> = na::Vector3::new(v[0], -v[2], v[1]);
        return v_scene;
    }

    fn to_scene_rotation_transform(q: na::Vector4<f32>) -> na::Vector4<f32> {
        let q_scene: na::Vector4<f32> = na::Vector4::new(q[1], -q[3], q[2], q[0]);
        return q_scene;
    }
}


fn main() 
{
    // Gui scene setup:
    let mut scene: MyScene = MyScene::new();

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

            // Draw body frame axes:
            // window.draw_line(&na::Point3::from(cube_cg), 
            //                  &na::Point3::from(cube_cg + translation_transform * (rotate_vec(rb.orientation, na::Vector3::new(2.1, 0., 0.)))),
            //                  &na::Point3::new(1., 0., 0.));
            
            // window.draw_line(&na::Point3::from(cube_cg), 
            //                  &na::Point3::from(cube_cg + translation_transform * (rotate_vec(rb.orientation, na::Vector3::new(0., 3.2, 0.)))),
            //                  &na::Point3::new(0., 1., 0.));

            // window.draw_line(&na::Point3::from(cube_cg), 
            //                  &na::Point3::from(cube_cg + translation_transform * (rotate_vec(rb.orientation, na::Vector3::new(0., 0., 1.2)))),
            //                  &na::Point3::new(0., 0., 1.));

            // Draw Frame frequency and 
            // window.draw_text(text: &str, pos: &Point2<f32>, scale: f32, font: &Rc<Font>, color: &Point3<f32>)
            // window.draw_text(text: &str, pos: &Point2<f32>, scale: f32, font: &Rc<Font>, color: &Point3<f32>)
        }
    }
}
