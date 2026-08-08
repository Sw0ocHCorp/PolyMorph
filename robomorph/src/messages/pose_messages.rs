use faer::col;
use nalgebra::Vector3;
use prost::Message;

use crate::messages::registered_message::{MessageType, Translatable, UnitQuat, Vec3};

/// Struct serializable / deserializable using protobuf that represent gnss sensor measurements and accuracy
#[derive(Clone, PartialEq, Message)]
pub struct GNSSMeasurement {
    #[prost(double, tag = "1")]
    pub longitude: f64,
    #[prost(double, tag = "2")]
    pub latitude: f64, 
    #[prost(double, tag = "3")]
    pub altitude: f64,
    #[prost(uint32, tag = "4")]
    pub fix_status: u32,
}

impl Translatable for GNSSMeasurement {
    const MSG_TYPE: MessageType= MessageType::GNSSRawMessage;
}

/// Struct serializable / deserializable using protobuf that represent the measurements of a 9DOFs IMU
#[derive(Clone, PartialEq, Message)]
pub struct IMUMeasurements {
    #[prost(message, optional, tag = "1")]
    pub l_accel: Option<Vec3>,
    #[prost(message, optional, tag = "2")]
    pub a_velocity: Option<Vec3>,
    #[prost(message, optional, tag = "3")]
    pub magnetic_field: Option<Vec3>,

}

impl Translatable for IMUMeasurements {
    const MSG_TYPE: MessageType= MessageType::ImuRawMessage;
}

#[derive(Clone, Copy)]
pub struct Transform {
    pub location: Vec3,
    pub orientation: UnitQuat,
}

/// Struct serializable / deserializable using protobuf that represent the pose of an object / vehcle
#[derive(Clone, PartialEq, Message)]
pub struct Pose {
    #[prost(message, optional, tag = "1")]
    pub gnss_measurement: Option<GNSSMeasurement>,
    #[prost(message, optional, tag = "2")]
    pub relative_location: Option<Vec3>,
    #[prost(message, optional, tag = "3")]
    pub orientation: Option<UnitQuat>,
    #[prost(message, optional, tag = "4")]
    pub imu_measurement: Option<IMUMeasurements>,
    /// Estimated linear velocity in the body frame. Not a raw measurement: derived by the state
    /// estimator (or provided directly by the simulator). `None` when no estimate is available.
    #[prost(message, optional, tag = "5")]
    pub l_velocity: Option<Vec3>,
}

impl Translatable for Pose {
    const MSG_TYPE: MessageType= MessageType::PoseMessage;
}