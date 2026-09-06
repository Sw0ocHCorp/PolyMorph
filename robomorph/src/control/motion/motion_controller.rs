//! Common vocabulary of the control cascade: the `MotionController` trait implemented by every
//! stage, the physical description of the vehicle (`VehicleKinematicConfig`) and the gravity constant.
//!
//! See `docs/src/concepts/cascade-asservissement.md` for the theory of the cascade and
//! `docs/src/concepts/wrench-inertie.md` for mass / inertia.

use std::{collections::HashMap, fmt::{Display, Formatter}, iter::Map, sync::mpsc, time::Duration};

use nalgebra::{DVector, Matrix3, Matrix6xX, Quaternion, Rotation3, UnitQuaternion, UnitVector3, Vector3, Vector6};
use prost_derive::Message;
use tokio::sync::broadcast::Receiver;

use crate::{control::{motion::motor_controller::{MotorController, working_axis_i32_to_vec3}, pid_controller::PIDController}, core::scheduler::Process, messages::{motor_messages::{MotorCommand, MotorCommandType, MotorFeedBack, MotorModel, MotorStatus, WorkVec, WorkingAxis}, pose_messages::{Pose, Transform}, registered_message::{AnyMessage, UnitQuat, Vec3}}};

/// Standard gravity (m/s^2). Multiplied by `VehicleKinematicConfig::weight` it gives the
/// gravity feedforward force `m * g` that the attitude stage asks the mixer to produce.
pub const GRAVITY: f64= 9.81;

/// A stage of the control cascade: something that turns (measurement, setpoint) into the
/// setpoint of the stage below it. Every stage is also a scheduled `Process`.
///
/// Two implementors exist today, with a strict interface contract between them:
///   - `AttitudeController`: (pose, pose setpoint)  ->  `AnyMessage::VehicleWrench`
///   - `MotorsMixer`:        (motor feedbacks, wrench setpoint) -> `AnyMessage::MotorCommands`
/// The 6D wrench is the pivot of the whole chain: the mixer does not know what commands it,
/// the attitude stage does not know which vehicle executes it.
pub trait MotionController : Process {

    /// The pure control law of the stage: consumes a measurement (`input_data`) and a setpoint,
    /// returns the setpoint of the next stage, or `None` when it cannot compute (missing data).
    /// `None` means ABSTAIN: the consumer keeps its last setpoint. It must never be replaced by
    /// a zero-valued message, which would be an active "cut everything" command.
    /// `dt` is the period of the stage; `verbose` enables the per-cycle prints (costly: a
    /// `println!` is a blocking call inside the period budget).
    fn compute_command_law(&mut self, input_data: Option<AnyMessage>, setpoint: Option<AnyMessage>, dt: Duration, verbose: bool) -> Option<AnyMessage>;

    /// Asynchronous setpoint input (broadcast channel), alternative to the scheduler pipe.
    fn set_setpoint_receiver(&mut self, receiver: Receiver<Pose>);

    //fn update_setpoint(&mut self, setpoint: AnyMessage):


}

/// Plain read-only description of the vehicle. Every field is a scalar or a `Copy` type,
/// so the whole struct is passed by value: cheaper than sharing it behind a pointer.
#[derive(Copy, Clone, Debug)]
/// It is the PHYSICAL identity card of the vehicle - what the vehicle IS - shared by every stage
/// (mixer row normalisation, attitude `M = I * alpha`, future velocity loop `f = m * a`).
/// Controller tunings (time constants) do NOT belong here: a tuning belongs to the component that
/// uses it.
pub struct VehicleKinematicConfig {
    /// Legacy fields, not read by the current chain (kept for compatibility of constructors).
    pub error_linear_factor: f64,
    pub error_angular_factor: f64,
    pub error_attitude_factor: f64,
    /// Position of the centre of mass in the vehicle frame (m). Added to every ROOT motor position
    /// so that all lever arms, hence all moments, are taken about the CoM (the "reduction point").
    pub com_relative_location: Vec3,        //location of the CoM, relative to the origin of the motor
    /// Total vehicle mass `m` (kg) - despite the name it is a mass, not a weight in newtons.
    pub weight: f64,
    /// The INERTIA matrix `I` (kg.m^2): the matrix of moments of INERTIA of the whole vehicle
    /// (base + arms + rotors), NOT a matrix of moment-torques. Its diagonal `I_xx, I_yy, I_zz` is the
    /// resistance to angular acceleration about each body axis; off-diagonal terms are the inertia
    /// products of an asymmetric mass distribution (zero for a symmetric vehicle, which is what
    /// allows the per-axis decoupling `M_k = I_k * alpha_k` used by the attitude stage).
    pub moments_matrix: Matrix3<f64>,
}
