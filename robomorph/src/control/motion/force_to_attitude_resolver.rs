use core::f64;

use nalgebra::{UnitQuaternion, UnitVector3, Vector3};
use tokio::sync::broadcast::{Receiver, Sender};
use crate::{control::motion::motor_controller::working_axis_i32_to_vec3, messages::{motor_messages::WorkingAxis, pose_messages::Pose, registered_message::{AnyMessage, UnitQuat, Vec3}}};


pub struct ForceToAttitudeResolver {
    setpoint_receiver: Option<Receiver<Pose>>,
    setpoint: Pose,
    vehicle_dof: Vec<WorkingAxis>,
}

impl ForceToAttitudeResolver {
    pub fn new() -> Self {
        ForceToAttitudeResolver {
            setpoint_receiver: None,
            setpoint: Pose::default(),
            vehicle_dof: vec![],
        }
    }

    pub fn set_receiver(&mut self, receiver: Receiver<Pose>) {
        self.setpoint_receiver = Some(receiver);
    }

    pub fn compute_attitude(&mut self, input: AnyMessage) -> Option<Pose> {
        if let AnyMessage::ForceVec(input_force) = input &&
                let Some(receiver) = &mut self.setpoint_receiver {
            if let Ok(setpoint) = receiver.try_recv() {
                // Here you would implement the logic to convert the force setpoint to an attitude setpoint.
                // For now, we just return the received setpoint as a placeholder.
                self.setpoint = setpoint.clone();
            } if let Some(target_orientation) = self.setpoint.orientation {
                let force_norm= Vector3::from(input_force) / input_force.norm();
                let mut dof_vec= UnitVector3::new_normalize(Vector3::new(1.0, 0.0, 0.0));
                let mut target_angle= 0.0;
                if self.vehicle_dof.len() == 1 {
                    let axis= working_axis_i32_to_vec3(self.vehicle_dof[0] as i32);
                    dof_vec= UnitQuaternion::from(target_orientation) * axis;
                    target_angle= 0.0;
                    //dof_vec= UnitVector3::new_normalize(Vector3::from(input_force));
                } if self.vehicle_dof.len() == 2 {
                    let axis1= working_axis_i32_to_vec3(self.vehicle_dof[0] as i32);
                    let axis2= working_axis_i32_to_vec3(self.vehicle_dof[1] as i32);
                    let normal_vec= axis1.cross(&axis2);
                    dof_vec = UnitVector3::new_normalize(UnitQuaternion::from(target_orientation).transform_vector(&normal_vec));
                    target_angle= f64::consts::PI / 2.0;
                } if self.vehicle_dof.len() == 3 {
                    println!("[WARNING::ForceToAttitudeResolver] ->  resolver not implemented for 3 DOF vehicles, coming soon...");
                }
                //acos(a · b)= atan2(‖a × b‖, a · b)
                //trick to avoid acos usage because acos have an infinite derivative close to 1 and -1
                //  which can cause numerical instability
                let required_rotation= f64::atan2((dof_vec.cross(&force_norm)).norm(), dof_vec.dot(&force_norm));
                let norm_rotation_vec= dof_vec.cross(&force_norm) / dof_vec.cross(&force_norm).norm();
                //let rotation_quat= UnitQuaternion::from_axis_angle(&norm_rotation_vec, required_rotation)
                let mut rotation_quat= UnitQuaternion::new(Vector3::x());
                if self.vehicle_dof.len() == 1 || self.vehicle_dof.len() == 3{ 
                    rotation_quat= UnitQuaternion::from_axis_angle(&UnitVector3::new_normalize(norm_rotation_vec), required_rotation - 0.0);
                } if self.vehicle_dof.len() == 2 {
                    rotation_quat= UnitQuaternion::from_axis_angle(&UnitVector3::new_normalize(norm_rotation_vec), required_rotation - f64::consts::PI / 2.0);
                }
                self.setpoint.orientation= Some(UnitQuat::from(rotation_quat * UnitQuaternion::from(target_orientation)));
            }
            return Some(self.setpoint.clone());
        }
        return None;
    }
    
}