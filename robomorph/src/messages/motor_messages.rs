//! Motor-side messages: the description of one node of the vehicle's kinematic tree
//! (`MotorModel`), the feedback and command exchanged with the actuators (`MotorFeedBack`,
//! `MotorCommand`), the 6D wrench (`WorkVec`) that is the currency between the attitude controller
//! and the motor mixer, and the protobuf form of the PID gains (`PIDConfig`).
//!
//! Units convention ("mixer units"): every effort is expressed as the physical quantity the motor
//! applies to the vehicle, a THRUST in newtons for a thruster and an ANGLE in radians for a joint.
//! Motor-native units (rotor speed in rad/s, PWM, ...) only exist at the hardware boundary (the
//! Gazebo vehicle controller), where the effort law
//! `T = transmission_factor * effort_constant * w^exp_command_law` converts in both directions.

use nalgebra::{Quaternion, UnitQuaternion, Vector3, Vector6};
use prost_derive::{Enumeration, Message};
use prost_types::Timestamp;

use crate::messages::{pose_messages::Transform, registered_message::{MessageType, Translatable, UnitQuat, Vec3}};

/// Generate the classic component-wise arithmetic for a plain struct of `f64` fields:
/// `+ - -(neg) += -=`, scalar `* /` (both `vec * f64` and `f64 * vec`), and `Sum`.
/// The field list is written once; every operator is derived from it.
macro_rules! impl_operators {
    ($name:ident { $($field:ident),+ $(,)? }) => {
        impl std::ops::Add for $name {
            type Output = $name;
            fn add(self, rhs: Self) -> Self { Self { $($field: self.$field + rhs.$field),+ } }
        }
        impl std::ops::Sub for $name {
            type Output = $name;
            fn sub(self, rhs: Self) -> Self { Self { $($field: self.$field - rhs.$field),+ } }
        }
        impl std::ops::Neg for $name {
            type Output = $name;
            fn neg(self) -> Self { Self { $($field: -self.$field),+ } }
        }
        impl std::ops::AddAssign for $name {
            fn add_assign(&mut self, rhs: Self) { $(self.$field += rhs.$field;)+ }
        }
        impl std::ops::SubAssign for $name {
            fn sub_assign(&mut self, rhs: Self) { $(self.$field -= rhs.$field;)+ }
        }
        // Scale by a scalar, in both orders so `w * k` and `k * w` both work.
        impl std::ops::Mul<f64> for $name {
            type Output = $name;
            fn mul(self, s: f64) -> Self { Self { $($field: self.$field * s),+ } }
        }
        impl std::ops::Mul<$name> for f64 {
            type Output = $name;
            fn mul(self, v: $name) -> $name { $name { $($field: self * v.$field),+ } }
        }
        impl std::ops::Div<f64> for $name {
            type Output = $name;
            fn div(self, s: f64) -> Self { Self { $($field: self.$field / s),+ } }
        }
        // Lets you `iter.sum()` a stream of wrenches (e.g. summing child contributions).
        impl std::iter::Sum for $name {
            fn sum<I: Iterator<Item = $name>>(iter: I) -> Self {
                iter.fold(<$name>::default(), |acc, x| acc + x)
            }
        }
    };
}

/// Enum that represent the life status of a motor
///
/// Carried as an `i32` in `MotorFeedBack::status` (prost enumeration). The discriminants start at
/// 1, so the prost default `0` maps to no variant. The Gazebo controller reports `IDLE`.
#[derive(Clone, Copy, PartialEq, Eq, Enumeration, Debug)]
pub enum MotorStatus {
    /// Motor known but not driven.
    IDLE= 1,
    /// Motor driven.
    RUNNING=2,
    /// Motor in fault.
    ERROR= 3,
}

/// Enum that represent the type of being apply to a motor
///
/// Declares the UNIT of `MotorCommand::setpoint_value` / `MotorFeedBack::setpoint_value`. The
/// mixer emits `THRUST` for thrusters and `ANGULARPOSITION` for joints; the conversion from
/// newtons to a rotor speed happens in the vehicle controller, at the hardware boundary only.
#[derive(Clone, Copy, PartialEq, Eq, Enumeration, Debug)]
pub enum MotorCommandType {
    /// Setpoint is an angle in rad (position-controlled joint).
    ANGULARPOSITION= 5,
    /// Setpoint is a rotational speed in rad/s (motor-native unit; not emitted by the mixer today).
    VELOCITY= 6,
    /// Setpoint is a thrust in N (thruster).
    THRUST= 7,
}

/// Enum that represent the Working axis of a motor
/// 
/// That is, the motion that the motor transmits to the object or vehicle
///
/// Splits the motors into the two families handled by the mixer:
/// * `RotatingAround{X,Y,Z}` (1..=3): an angular joint. Decision variable = its angle (rad),
///   bounds = the joint limits (`MotorModel::min_value` / `max_value`), `max_rot_speed` = the
///   joint speed used as trust region per cycle. A joint produces no wrench of its own: it
///   reorients what its children produce.
/// * `LinearMotionALong{X,Y,Z}` (4..=6): a thruster. Decision variable = its thrust (N).
///
/// The axis is expressed in the motor's OWN frame; the mixer rotates it into the body frame with
/// the resolved `Transform::orientation`. The family is tested by ordering on the discriminant
/// (`working_axis <= RotatingAroundZ` = joint, `>= LinearMotionALongX` = thruster).
// NOTE: because of those ordering tests, `Unknown` (0) falls in the joint family inside the mixer.
#[derive(Clone, Copy, PartialEq, Eq, Enumeration, Debug)]
pub enum WorkingAxis {
    /// Not set (prost default).
    Unknown= 0,
    /// Rotation only around X axis
    RotatingAroundX= 1, 
    /// Rotation only around Y axis
    RotatingAroundY= 2, 
    /// Rotation only around Z axis
    RotatingAroundZ= 3,
    /// Produce Linear Motion along the X axis 
    LinearMotionALongX= 4,
    /// Produce Linear Motion along the Y axis 
    LinearMotionALongY= 5,
    /// Produce Linear Motion along the Z axis
    LinearMotionALongZ= 6,
}

/// A 6D wrench (spatial force) expressed in the vehicle body frame, with the moment
/// taken about the center of mass. Used to carry a child motor's current wrench up to
/// its parent when assembling an angular motor's effort gradient.
///
/// Layout `` [fx fy fz mx my mz] ``: forces in N along the body axes, moments in N.m about the
/// centre of mass (the mixer's "reduction point"). It is the interface between the attitude
/// controller and the mixer (`AnyMessage::VehicleWrench`: the wrench the vehicle should apply) and
/// the mixer's internal currency: the wrench each motor produces right now, their sum over the
/// roots (current vehicle wrench, the linearisation point) and the child wrench a joint reorients.
///
/// Plain `f64` struct, not a protobuf message: it never goes on the wire (`encode_frame` emits an
/// empty frame for `AnyMessage::VehicleWrench`). `Default` is the null wrench.
#[derive(Clone, Copy, Default, Debug)]
pub struct WorkVec {
    // Force components along the body X / Y / Z axes (N).
    pub fx: f64,
    pub fy: f64,
    pub fz: f64,
    // Moment components about the body X / Y / Z axes, taken at the CoM (N.m).
    pub mx: f64,
    pub my: f64,
    pub mz: f64,
}

impl WorkVec {
    /// Build from a force part (N) and a moment part (N.m), both in the body frame.
    pub fn new_from_vector3(thrust_force: Vector3<f64>, moments: Vector3<f64>) -> Self {
        return Self { fx: thrust_force.x, fy: thrust_force.y, fz: thrust_force.z, 
                mx: moments.x, my: moments.y, mz: moments.z };
    }

    /// Same as `new_from_vector3` with the wire vector type.
    pub fn new_from_vec3(thrust_force: Vec3, moments: Vec3) -> Self {
        return Self { fx: thrust_force.x, fy: thrust_force.y, fz: thrust_force.z, 
                mx: moments.x, my: moments.y, mz: moments.z };
    }
}

/// nalgebra names the six components `x, y, z, w, a, b`: they map onto `fx, fy, fz, mx, my, mz` in
/// that order (the control-allocation solver of the mixer works on `Vector6`).
impl From<Vector6<f64>> for WorkVec {
    fn from(v: Vector6<f64>) -> Self {
        return Self{fx: v.x,  fy: v.y, fz: v.z, mx: v.w, my: v.a, mz: v.b};
    }
}

/// Inverse of the above: `[fx, fy, fz, mx, my, mz]` as a `Vector6`.
impl From<WorkVec> for Vector6<f64> {
    fn from(w: WorkVec) -> Self {
        return Self::new(w.fx, w.fy, w.fz, w.mx, w.my, w.mz);
    }
}

impl_operators!(WorkVec { fx, fy, fz, mx, my, mz });

/// Protobuf form of the `control::pid_controller::PIDController` parameters, so that a motor's
/// servo gains can travel inside `MotorModel` / `MotorFeedBack`.
///
/// Same meaning as `PIDController::new(p, i, d, min_correction_error, max_error_accum)`: gains on
/// an error expressed in the servoed unit (rad for a joint), dead band, integrator clamp.
#[derive(Clone, PartialEq, Message)]
pub struct PIDConfig {
    /// Proportional gain.
    #[prost(double, tag = "1")]
    pub p: f64,
    /// Integral gain (on the clamped error integral).
    #[prost(double, tag = "2")]
    pub i: f64,
    /// Derivative gain (derivative on error).
    #[prost(double, tag = "3")]
    pub d: f64,
    /// Dead band: below this absolute error (two consecutive samples) the controller resets and
    /// outputs zero.
    #[prost(double, tag = "4")]
    pub min_correction_error: f64,
    /// Clamp of the error integral (anti-windup), in error-unit * s.
    #[prost(double, tag = "5")]
    pub max_error_accum: f64,
}


/// Serializable / Deserializable struct using protobuf that represent a motor command
///
/// One element of `AnyMessage::MotorCommands`, produced by `MotorsMixer` (one per motor per cycle)
/// and consumed by the vehicle controller. `setpoint_value` is an EFFORT in mixer units and
/// `command_type` says which: `THRUST` = N, `ANGULARPOSITION` = rad, `VELOCITY` = rad/s. The Gazebo
/// controller converts newtons into a rotor speed at the hardware boundary, never the mixer.
///
/// Derives `Message` but is not `Translatable`: it does not go on the wire on its own.
#[derive(Clone, PartialEq, Message)]
pub struct MotorCommand {
    /// Time the command was produced.
    #[prost(message, optional, tag = "1")]
    pub timestamp: Option<Timestamp>,
    /// Id of the target motor (`MotorModel::id`).
    #[prost(uint32, tag = "2")]
    pub id: u32,
    /// `MotorCommandType` as i32: declares the unit of `setpoint_value`.
    #[prost(enumeration = "MotorCommandType", tag = "3")]
    pub command_type: i32,
    /// Effort to apply, in the unit declared by `command_type` (N or rad in practice).
    #[prost(double, tag = "4")]
    pub setpoint_value: f64,
}

/// Serializable / Deserializable struct using protobuf that represent motor specs, the last command applied, it's status, ...
///
/// Published per motor by the vehicle controller (on the motor-feedback channel, or on the wire as
/// `AnyMessage::MotorState`, tag `MessageType::MotorFeedBackMessage`) and copied into the mixer's
/// `MotorController` (`update_motor_feedback`), where `current_value` is the state the
/// allocation is linearised around.
#[derive(Clone, PartialEq, Message)]
pub struct MotorFeedBack {
    /// Time of the measurement.
    #[prost(message, optional, tag = "1")]
    pub timestamp: Option<Timestamp>,
    /// Id of the motor (`MotorModel::id`).
    #[prost(uint32, tag = "2")]
    pub id: u32,
    /// `MotorStatus` as i32.
    #[prost(enumeration = "MotorStatus", tag = "3")]
    pub status: i32,
    /// CURRENT EFFORT of the motor in mixer units: the thrust T in N for a thruster (reconstructed by
    /// the vehicle controller from the measured rotor speed with the effort law), the angle in rad for
    /// a joint. The mixer reads it as the linearisation point of a thruster and as the joint angle
    /// theta when resolving the motor transforms (`compute_motor_transforms`).
    #[prost(double, tag = "4")]
    pub current_value: f64,
    /// Last effort commanded to the motor, same unit as `current_value`.
    #[prost(double, tag = "5")]
    pub setpoint_value: f64,
    /// `MotorCommandType` as i32: declares the unit of `setpoint_value`.
    #[prost(enumeration = "MotorCommandType", tag = "6")]
    pub command_type: i32,
    /// Gains of the per-motor servo loop (see `PIDConfig`).
    #[prost(message, optional, tag = "7")]
    pub pid_config: Option<PIDConfig>,
    /// Frequency (Hz) of the per-motor servo loop on the hardware side (the Gazebo controller
    /// reports 50).
    #[prost(uint32, tag = "8")]
    pub control_frequency: u32,
}

impl Translatable for MotorFeedBack {
    const MSG_TYPE: MessageType= MessageType::MotorFeedBackMessage;
}

/// Serializable / Deserializable struct using protobuf that represent a a motor in the motor graph of a vehicle
///
/// The motors of a vehicle form a tree (the "motor graph"): a node is mounted on its parent
/// (`parent_id`, `0` = mounted on the body) and carries its children (`child_ids`). Its pose at rest
/// is given in the PARENT frame (`relative_location`, `relative_orientation`); the mixer resolves
/// the pose of every node in the body frame (`pose_messages::Transform`) by composing the chain,
/// adding each joint's current angle. `working_axis` selects the family (joint / thruster) and with
/// it the meaning of the bounds and constants below.
///
/// Wire tag `MessageType::MotorModelMessage` (encodable, but not decoded by `decode_frame`; in the
/// Gazebo binary the models travel on a dedicated channel, wrapped in a `MotorController`).
#[derive(Clone, PartialEq, Message)]
pub struct MotorModel {
    /// Unique id of the motor. `0` cannot be used as an id, since `parent_id == 0` means "mounted on
    /// the body". The Gazebo controller derives it from the joint name.
    #[prost(uint32, tag = "1")]
    pub id: u32,
    /// Id of this motor's parent in the kinematic chain. `0` if the motor is a root
    /// (mounted directly on the vehicle body).
    #[prost(uint32, tag = "2")]
    pub parent_id: u32,
    /// Ids of the motors mounted on this one. A single joint can drive several children
    /// (e.g. a steering column turning both front wheels).
    #[prost(uint32, repeated, tag = "3")]
    pub child_ids: Vec<u32>,
    /// Position of this motor relative to its parent, in the parent's frame. For a root
    /// motor it is relative to the vehicle origin (the CoM offset is applied at use).
    /// Unit: m.
    #[prost(message, optional, tag = "4")]
    pub relative_location: Option<Vec3>,
    /// Orientation of this motor at rest relative to its parent. Does NOT include a
    /// joint's current angle (that is passed separately when resolving the transform).
    /// For a root motor it is expressed in the body frame.
    #[prost(message, optional, tag = "5")]
    pub relative_orientation: Option<UnitQuat>,
    /// Axis the motor acts on: a rotation axis for an angular joint, a force direction
    /// for a linear motor / thruster.
    #[prost(enumeration = "WorkingAxis", tag = "6")]
    pub working_axis: i32,
    /// PID gains used to servo this motor (angular motors).
    #[prost(message, optional, tag = "7")]
    pub pid_config: Option<PIDConfig>,
    /// Lower / upper bound of the motor's command: joint limits (rad) for an angular
    /// motor, `0`..max effort for a thruster once mapped through the effort law.
    /// The mixer clamps both the commanded effort and the per-cycle increment to this range.
    // NOTE: the vehicle controller deliberately sets a strictly POSITIVE floor on rotors (not 0):
    // a joint's allocation column is proportional to its children's thrust, so at exactly zero
    // thrust the joint would lose all the authority it borrows from its child and vanish from the
    // allocation.
    #[prost(double, tag = "8")]
    pub min_value: f64,
    /// Upper bound of the command (rad for a joint, N for a thruster).
    #[prost(double, tag = "9")]
    pub max_value: f64,
    /// Effort-law constant `k` in `effort = k * command^exp_command_law`
    /// (e.g. thrust `T = k * w^2`).
    /// Full law with the transmission: `T = transmission_factor * k * w^exp_command_law`, `w` in
    /// rad/s and `T` in N. Used in both directions at the hardware boundary (speed -> thrust for the
    /// feedback, thrust -> speed for the command).
    #[prost(double, tag = "10")]
    pub effort_constant: f64,
    /// Intrinsic reaction moment per unit effort (a rotor's drag torque), along the
    /// effort axis. Carries the sign of the spin direction; `0` when there is none.
    /// `k_m` in N.m per N of thrust, along the thrust axis. SIGNED by the spin direction:
    /// counter-rotating rotors need opposite signs so that their reaction torques cancel at hover.
    #[prost(double, tag = "11")]
    pub moment_constant: f64,
    /// Maximum rotational speed of the motor (rad/s) - the command ceiling of a thruster.
    /// For an angular joint it is the joint speed (rad/s) the mixer uses as a trust region: the
    /// angle increment per cycle is clamped to `+/- max_rot_speed * dt`.
    #[prost(double, tag = "12")]
    pub max_rot_speed: f64,
    /// KV rating of the motor (rpm per volt). Descriptive only: not read by the control stack today.
    #[prost(double, tag = "13")]
    pub motor_kv: f64,
    /// Exponent of the effort law (2 for a propeller `k*w^2`, 1 for a linear torque motor).
    #[prost(uint32, tag = "14")]
    pub exp_command_law: u32,
    /// Reduction ratio between this joint's command and the motion transmitted to its
    /// subtree (`1` = direct drive).
    /// For a thruster it multiplies the effort law (see `effort_constant`).
    #[prost(double, tag = "15")]
    pub transmission_factor: f64,
}

impl Translatable for MotorModel {
    const MSG_TYPE: MessageType= MessageType::MotorModelMessage;
}