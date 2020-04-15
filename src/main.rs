extern crate nalgebra as na;
extern crate kiss3d;
#[macro_use] extern crate log;
use gilrs::{Gilrs, Axis, Event, Gamepad, EventType};
use std::time;
use std::thread;
use std::path::Path;

mod quaternion;
mod time_manager;
mod rigid_body;

// Todo list:
// 2. plotting / logging
// 3. add runge kutta dynamics

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
    quad.set_local_rotation(na::UnitQuaternion::from_euler_angles(1.5708, 0., 0.));

    // Transformation matrices from NED to OpenGL coorinate system:
    let translation_transform: na::Matrix3<f32> = na::Matrix3::new(1., 0., 0., 0., 0., -1., 0., 1., 0.);
    let rotation_transform: na::Matrix4<f32> = na::Matrix4::new(0., 1., 0., 0., 0., 0., 0., -1., 0., 0., 1., 0., 1., 0., 0., 0.);

    // Rigidbody:
    let mut rb = rigid_body::RigidBody::build_default();

    // Game pad management:
    let mut gilrs = Gilrs::new().unwrap();
    let mut _gp: Option<Gamepad> = None;

    for (_id, gamepad) in gilrs.gamepads() {
        println!(" >>> Available input device: {}", gamepad.name());
        _gp = Some(gilrs.gamepad(gamepad.id()));
    }

    // Time management:
    let mut time_manager = time_manager::TimeStep::new();

    let mut _throttle = 0.0;
    let mut _roll = 0.0;
    let mut _pitch = 0.0;
    let mut _yaw = 0.0;

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

            // Move drone in the scene:
            let cube_cg = translation_transform * rb.position;
            cube_trans= na::Translation3::from(cube_cg);
            cube_rot = na::UnitQuaternion::from_quaternion(na::Quaternion::from(rotation_transform * rb.orientation));
            cube.set_local_translation(cube_trans);
            cube.set_local_rotation(cube_rot);

            // Call render:
            is_running = window.render();

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
