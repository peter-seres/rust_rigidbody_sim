extern crate nalgebra as na;

// This module implements essential quaternion rotation equations for na::Vector4<f32> representation of a unit quaternion.


// For quaternion rotations:
pub fn as_matrix(q: na::Vector4<f32>) -> na::Matrix4<f32> {
    na::Matrix4::new(q[0], -q[1], -q[2], -q[3],
                     q[1],  q[0], -q[3],  q[2],
                     q[2],  q[3],  q[0], -q[1],
                     q[3], -q[2],  q[1],  q[0])
}

// For inverse quaternion rotations:
pub fn as_inverse_matrix(q: na::Vector4<f32>) -> na::Matrix4<f32> {
    na::Matrix4::new(q[0],   q[1],  q[2],  q[3],
                    -q[1],  q[0],  q[3], -q[2],
                    -q[2], -q[3],  q[0],  q[1],
                    -q[3],  q[2],  -q[1],  q[0])
}

// Prepend 0 to Vector3:
pub fn as_quat(v: na::Vector3<f32>) -> na::Vector4<f32> {
    na::Vector4::new(0.0, v[0], v[1], v[2])
}

// Rotate vector p using quaternion:  p' = q p q.inv()
pub fn rotate_vec(q: na::Vector4<f32>, p: na::Vector3<f32>) -> na::Vector3<f32> {
    let p_extended = as_quat(p);
    let p_rot_extended = as_matrix(q) * (as_matrix(p_extended) * as_inverse_matrix(q));
    
    na::Vector3::new(p_rot_extended[1], p_rot_extended[2], p_rot_extended[3])
}

// Rotate quaternion q_2 by q_1:
pub fn _rotate_quat(q_1: na::Vector4<f32>, q_2: na::Vector4<f32>) -> na::Vector4<f32> {
    as_matrix(q_1) * q_2
}

// Quaternion derivative: q_dot = 0.5 * Q * [0, omega]^T
pub fn q_dot(q: na::Vector4<f32>, omega: na::Vector3<f32> ) -> na::Vector4<f32> {
    0.5 * as_matrix(q) * as_quat(omega)
}
