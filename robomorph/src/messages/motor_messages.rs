use nalgebra::{Quaternion, UnitQuaternion, Vector3, Vector6};
use prost_derive::{Enumeration, Message};

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
#[derive(Clone, Copy, PartialEq, Eq, Enumeration, Debug)]
pub enum MotorStatus {
    IDLE= 1,
    RUNNING=2,
    ERROR= 3,
}

/// Enum that represent the type of being apply to a motor
#[derive(Clone, Copy, PartialEq, Eq, Enumeration, Debug)]
pub enum MotorCommandType {
    ANGULARPOSITION= 5,
    VELOCITY= 6,
    THRUST= 7,
}

/// Enum that represent the Working axis of a motor
/// 
/// That is, the motion that the motor transmits to the object or vehicle
#[derive(Clone, Copy, PartialEq, Eq, Enumeration, Debug)]
pub enum WorkingAxis {
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
#[derive(Clone, Copy, Default)]
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
    pub fn new_from_vector3(thrust_force: Vector3<f64>, moments: Vector3<f64>) -> Self {
        return Self { fx: thrust_force.x, fy: thrust_force.y, fz: thrust_force.z, 
                mx: moments.x, my: moments.y, mz: moments.z };
    }

    pub fn new_from_vec3(thrust_force: Vec3, moments: Vec3) -> Self {
        return Self { fx: thrust_force.x, fy: thrust_force.y, fz: thrust_force.z, 
                mx: moments.x, my: moments.y, mz: moments.z };
    }
}

impl From<Vector6<f64>> for WorkVec {
    fn from(v: Vector6<f64>) -> Self {
        return Self{fx: v.x,  fy: v.y, fz: v.z, mx: v.w, my: v.a, mz: v.b};
    }
}

impl From<WorkVec> for Vector6<f64> {
    fn from(w: WorkVec) -> Self {
        return Self::new(w.fx, w.fy, w.fz, w.mx, w.my, w.mz);
    }
}

impl_operators!(WorkVec { fx, fy, fz, mx, my, mz });

#[derive(Clone, PartialEq, Message)]
pub struct PIDConfig {
    #[prost(double, tag = "1")]
    pub p: f64,
    #[prost(double, tag = "2")]
    pub i: f64,
    #[prost(double, tag = "3")]
    pub d: f64,
    #[prost(double, tag = "4")]
    pub min_correction_error: f64,
    #[prost(double, tag = "5")]
    pub max_error_accum: f64,
}


/// Serializable / Deserializable struct using protobuf that represent a motor command
#[derive(Clone, PartialEq, Message)]
pub struct MotorCommand {
    #[prost(uint32, tag = "1")]
    pub id: u32,
    #[prost(enumeration = "MotorCommandType", tag = "2")]
    pub command_type: i32,
    #[prost(double, tag = "3")]
    pub setpoint_value: f64,
}

/// Serializable / Deserializable struct using protobuf that represent motor specs, the last command applied, it's status, ...
#[derive(Clone, PartialEq, Message)]
pub struct MotorFeedBack {
    #[prost(enumeration = "MotorStatus", tag = "2")]
    pub status: i32,
    #[prost(double, tag = "3")]
    pub current_value: f64,
    #[prost(double, tag = "4")]
    pub setpoint_value: f64,
    #[prost(enumeration = "MotorCommandType", tag = "5")]
    pub command_type: i32,
    #[prost(message, optional, tag = "11")]
    pub pid_config: Option<PIDConfig>,
    #[prost(uint32, tag = "12")]
    pub control_frequency: u32,
}

impl Translatable for MotorFeedBack {
    const MSG_TYPE: MessageType= MessageType::MotorFeedBackMessage;
}

/// Serializable / Deserializable struct using protobuf that represent a a motor in the motor graph of a vehicle
#[derive(Clone, PartialEq, Message)]
pub struct MotorModel {
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
    #[prost(message, optional, tag = "4")]
    pub relative_location: Option<Vec3>,
    /// Orientation of this motor at rest relative to its parent. Does NOT include a
    /// joint's current angle (that is passed separately when resolving the transform).
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
    #[prost(double, tag = "8")]
    pub min_value: f64,
    #[prost(double, tag = "9")]
    pub max_value: f64,
    /// Effort-law constant `k` in `effort = k * command^exp_command_law`
    /// (e.g. thrust `T = k * w^2`).
    #[prost(double, tag = "10")]
    pub effort_constant: f64,
    /// Intrinsic reaction moment per unit effort (a rotor's drag torque), along the
    /// effort axis. Carries the sign of the spin direction; `0` when there is none.
    #[prost(double, tag = "11")]
    pub moment_constant: f64,
    /// Maximum rotational speed of the motor (rad/s) - the command ceiling of a thruster.
    #[prost(double, tag = "12")]
    pub max_rot_speed: f64,
    #[prost(double, tag = "13")]
    pub motor_kv: f64,
    /// Exponent of the effort law (2 for a propeller `k*w^2`, 1 for a linear torque motor).
    #[prost(uint32, tag = "14")]
    pub exp_command_law: u32,
    /// Reduction ratio between this joint's command and the motion transmitted to its
    /// subtree (`1` = direct drive).
    #[prost(double, tag = "15")]
    pub transmission_factor: f64,
}

impl Translatable for MotorModel {
    const MSG_TYPE: MessageType= MessageType::MotorModelMessage;
}