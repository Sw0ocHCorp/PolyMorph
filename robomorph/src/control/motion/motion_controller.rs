use std::{collections::HashMap, fmt::{Display, Formatter}, iter::Map, time::Duration};

use nalgebra::{DVector, Matrix6xX, Quaternion, Rotation3, UnitQuaternion, UnitVector3, Vector3, Vector6};
use prost_derive::Message;

use crate::{control::{motion::motor_controller::{MotorController, working_axis_i32_to_vec3}, pid_controller::PIDController}, core::scheduler::Process, messages::{motor_messages::{MotorCommand, MotorCommandType, MotorFeedBack, MotorModel, MotorStatus, WorkVec, WorkingAxis}, pose_messages::{Pose, Transform}, registered_message::{AnyMessage, UnitQuat, Vec3}}};

pub const GRAVITY: f64= 9.81;

pub trait MotionController : Process {

    fn compute_command_law(&mut self, inputs: AnyMessage, setpoint: AnyMessage, dt: Duration) -> AnyMessage;

    fn send_motor_command(&self);

    fn add_or_update_motor(&mut self, new_motor: MotorController);
}

pub struct VehicleKinematicConfig {
    pub error_linear_factor: f64,
    pub error_angular_factor: f64,
    pub error_attitude_factor: f64,
    pub com_relative_location: Vec3,        //location of the CoM, relative to the origin of the motor
    pub weight: f64,
    pub moments_matrix: [[f64; 3]; 3],
}
