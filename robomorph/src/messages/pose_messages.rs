//! Vehicle state messages: the raw sensor samples (`IMUMeasurements`, `GNSSMeasurement`), the
//! aggregated vehicle state `Pose` (also used as the SETPOINT of the attitude controller) and
//! `Transform`, the resolved pose of a motor in the body frame computed by the mixer.
//!
//! Frames: "body" is the frame attached to the vehicle; "world" is the inertial frame the attitude
//! quaternion is expressed against, z up (the attitude controller compensates the weight along
//! +z world).

use faer::col;
use nalgebra::Vector3;
use prost::Message;
use prost_types::Timestamp;

use crate::messages::registered_message::{MessageType, Translatable, UnitQuat, Vec3};

/// Struct serializable / deserializable using protobuf that represent gnss sensor measurements and accuracy
///
/// Wire tag `MessageType::GNSSRawMessage`; also embedded in `Pose::gnss_measurement`. Not read by
/// the control loops today.
#[derive(Clone, PartialEq, Message)]
pub struct GNSSMeasurement {
    #[prost(message, optional, tag = "1")]
    pub timestamp: Option<Timestamp>,
    /// Longitude in degrees.
    #[prost(double, tag = "2")]
    pub longitude: f64,
    /// Latitude in degrees.
    #[prost(double, tag = "3")]
    pub latitude: f64, 
    /// Altitude in m.
    #[prost(double, tag = "4")]
    pub altitude: f64,
    /// Fix status flag of the receiver, passed through as reported (the Gazebo controller
    /// publishes 1 for its GNSS samples).
    #[prost(uint32, tag = "5")]
    pub fix_status: u32,
}

impl Translatable for GNSSMeasurement {
    const MSG_TYPE: MessageType= MessageType::GNSSRawMessage;
}

/// Struct serializable / deserializable using protobuf that represent the measurements of a 9DOFs IMU
///
/// All the vectors are in the BODY frame. Wire tag `MessageType::ImuRawMessage`; also embedded in
/// `Pose::imu_measurement`, which is where the attitude controller reads it.
#[derive(Clone, PartialEq, Message)]
pub struct IMUMeasurements {
    #[prost(message, optional, tag = "1")]
    pub timestamp: Option<Timestamp>,
    /// Specific force (m/s^2) measured by the accelerometer, body frame: its norm is about 9.81 when
    /// the vehicle is supported (ground, hover) and about 0 in free fall. Usable as a contact /
    /// free-fall detector; it is NOT a linear acceleration estimate.
    #[prost(message, optional, tag = "2")]
    pub l_accel: Option<Vec3>,
    /// Angular velocity (rad/s) from the gyroscope, body frame. Used DIRECTLY as the rate (D) term of
    /// the attitude loop: the controller damps it without differentiating anything.
    #[prost(message, optional, tag = "3")]
    pub a_velocity: Option<Vec3>,
    /// Magnetometer reading, body frame, in the sensor's native unit. Not consumed by the control
    /// stack today.
    #[prost(message, optional, tag = "4")]
    pub magnetic_field: Option<Vec3>,

}

impl Translatable for IMUMeasurements {
    const MSG_TYPE: MessageType= MessageType::ImuRawMessage;
}

/// Resolved pose of a motor in the BODY frame, computed by `MotorsMixer::compute_motor_transforms`
/// from the `MotorModel` chain (parent poses composed, each joint's current angle included).
///
/// Internal type: not a protobuf message, never on the wire.
#[derive(Clone, Copy)]
pub struct Transform {
    /// Position of the motor (m) measured from the reduction point (the centre of mass): the lever
    /// arm used for the `p x f` moments.
    pub location: Vec3,
    /// Rotation taking a vector from the MOTOR frame to the body frame; applied to the `WorkingAxis`
    /// to get the thrust direction / joint axis in the body frame.
    pub orientation: UnitQuat,
}

/// Struct serializable / deserializable using protobuf that represent the pose of an object / vehcle
///
/// Aggregate of the vehicle state as published by the vehicle controller (`AnyMessage::PoseState`,
/// wire tag `MessageType::PoseMessage`). The SAME struct is used as the SETPOINT of the attitude
/// controller: there only `orientation` is read (the target angular velocity is implicitly zero),
/// while `orientation` and `imu_measurement.a_velocity` are read from the current state.
#[derive(Clone, PartialEq, Message)]
pub struct Pose {
    /// Time of the state.
    #[prost(message, optional, tag = "1")]
    pub timestamp: Option<Timestamp>,
    /// Latest GNSS sample.
    #[prost(message, optional, tag = "2")]
    pub gnss_measurement: Option<GNSSMeasurement>,
    /// Position of the vehicle (m) relative to a local reference origin. Left at zero by the Gazebo
    /// controller today.
    #[prost(message, optional, tag = "3")]
    pub relative_location: Option<Vec3>,
    /// Attitude quaternion BODY -> WORLD: applied to a body vector it yields world coordinates; its
    /// inverse maps world -> body (the attitude controller uses the inverse to bring the setpoint
    /// error and the weight vector into the body frame).
    #[prost(message, optional, tag = "4")]
    pub orientation: Option<UnitQuat>,
    /// Latest IMU sample (body frame); `a_velocity` is the gyro used by the attitude loop.
    #[prost(message, optional, tag = "5")]
    pub imu_measurement: Option<IMUMeasurements>,
    /// Estimated linear velocity in the body frame. Not a raw measurement: derived by the state
    /// estimator (or provided directly by the simulator). `None` when no estimate is available.
    #[prost(message, optional, tag = "6")]
    pub l_velocity: Option<Vec3>,
}

impl Translatable for Pose {
    const MSG_TYPE: MessageType= MessageType::PoseMessage;
}