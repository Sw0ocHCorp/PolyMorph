use faer::col;
use num_quaternion::{EulerAngles, Q64, Quaternion, UQ64};
use prost::Message;

use crate::messages::registered_message::{MessageType, Translatable};

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
    #[prost(double, tag = "1")]
    pub l_accel_x: f64,
    #[prost(double, tag = "2")]
    pub l_accel_y: f64,
    #[prost(double, tag = "3")]
    pub l_accel_z: f64,
    #[prost(double, tag = "4")]
    pub a_velocity_x: f64,
    #[prost(double, tag = "5")]
    pub a_velocity_y: f64,
    #[prost(double, tag = "6")]
    pub a_velocity_z: f64,
    #[prost(double, tag = "7")]
    pub magnetic_field_x: f64,
    #[prost(double, tag = "8")]
    pub magnetic_field_y: f64,
    #[prost(double, tag = "9")]
    pub magnetic_field_z: f64,

}

impl Translatable for IMUMeasurements {
    const MSG_TYPE: MessageType= MessageType::ImuRawMessage;
}

/// Struct serializable / deserializable using protobuf that represent the pose of an object / vehcle
#[derive(Clone, PartialEq, Message)]
pub struct Pose {
    #[prost(double, tag = "1")]
    pub location_x: f64,
    #[prost(double, tag = "2")]
    pub location_y: f64,
    #[prost(double, tag = "3")]
    pub location_z: f64,
    #[prost(double, tag = "4")]
    pub roll: f64,
    #[prost(double, tag = "5")]
    pub pitch: f64,
    #[prost(double, tag = "6")]
    pub yaw: f64,
    #[prost(double, tag = "7")]
    pub l_velocity_x: f64,
    #[prost(double, tag = "8")]
    pub l_velocity_y: f64,
    #[prost(double, tag = "9")]
    pub l_velocity_z: f64,
    #[prost(double, tag = "10")]
    pub a_velocity_x: f64,
    #[prost(double, tag = "11")]
    pub a_velocity_y: f64,
    #[prost(double, tag = "12")]
    pub a_velocity_z: f64,
}

impl Translatable for Pose {
    const MSG_TYPE: MessageType= MessageType::PoseMessage;
}