extern crate nalgebra as na;
extern crate kiss3d;
use crate::rigid_body;
use crate::quaternion;
use std::path::Path;
use std::f32::consts::FRAC_PI_2;


pub struct MyScene {
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
        ground.set_local_rotation(na::UnitQuaternion::from_euler_angles(FRAC_PI_2, 0., 0.));
        ground.set_color(0.3, 0.3, 0.3);

        MyScene{
            window,
            uav,
            _ground: ground
        }
    }

    pub fn render(&mut self, rb: &rigid_body::RigidBody) -> bool {

        // Move the uav in the scene:
        let pos: na::Vector3<f32> = MyScene::transform_translation_to_scene(rb.position);
        self.uav.set_local_translation(na::Translation3::from(pos));

        // Rotate the uav in the scene:
        let rot_scene: na::Vector4<f32> = MyScene::transform_rotation_to_scene(rb.orientation);
        let rot: na::Quaternion<f32> = na::Quaternion::from(rot_scene);
        self.uav.set_local_rotation(na::UnitQuaternion::from_quaternion(rot));

        // Draw the axis lines:
        let start = &na::Point3::from(pos);

        let end_i = &na::Point3::from(
            pos + MyScene::transform_translation_to_scene(
            quaternion::rotate_vec(rb.orientation, na::Vector3::new(2.1, 0., 0.))
            ));
        let end_j = &na::Point3::from(
            pos + MyScene::transform_translation_to_scene(
            quaternion::rotate_vec(rb.orientation, na::Vector3::new(0., 3.2, 0.))
            ));
        let end_k = &na::Point3::from(
            pos + MyScene::transform_translation_to_scene(
            quaternion::rotate_vec(rb.orientation, na::Vector3::new(0., 0., 1.2))
            ));

        self.window.draw_line(start, end_i, &na::Point3::new(1., 0., 0.));
        self.window.draw_line(start, end_j, &na::Point3::new(0., 1., 0.));
        self.window.draw_line(start, end_k, &na::Point3::new(0., 0., 1.));

        // Render command:
        self.window.render()
    }

    fn transform_translation_to_scene(v: na::Vector3<f32>) -> na::Vector3<f32> {
        na::Vector3::new(v[0], -v[2], v[1])
    }

    fn transform_rotation_to_scene(q: na::Vector4<f32>) -> na::Vector4<f32> {
       na::Vector4::new(q[1], -q[3], q[2], q[0])
    }
}