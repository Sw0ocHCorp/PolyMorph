use prost::Message;

use crate::messages::registered_message::{MessageType, Translatable};

/// Struct serializable / deserializable using protobuf that represent a Ray of the lidar
#[derive(Clone, PartialEq, Message)]
pub struct Ray {
    #[prost(double, tag = "1")]
    pub vertical_angle: f64,
    #[prost(double, tag = "2")]
    pub horizontal_angle: f64,
    #[prost(double, tag = "3")]
    pub length: f64,
}

/// Struct serializable / deserializable using protobuf that represent a Point in 3D space
#[derive(Clone, PartialEq, Message)]
pub struct Point3D {
    #[prost(double, tag = "1")]
    pub x: f64,
    #[prost(double, tag = "2")]
    pub y: f64,
    #[prost(double, tag = "3")]
    pub z: f64,
}

/// Struct serializable / deserializable using protobuf that represent lidar sensor specs and measurements
/// 
/// to be able to rebuild the Lidar map using all the Rays produced by the lidar sensor
#[derive(Clone, PartialEq, Message)]
pub struct LidarMeasurements {
    #[prost(double, tag = "1")]
    pub vertical_min_angle: f64,
    #[prost(double, tag = "2")]
    pub vertical_angle_resolution: f64,
    #[prost(double, tag = "3")]
    pub vertical_max_angle: f64,
    #[prost(double, tag = "4")]
    pub horizontal_min_angle: f64,
    #[prost(double, tag = "5")]
    pub horizontal_angle_resolution: f64,
    #[prost(double, tag = "6")]
    pub horizontal_max_angle: f64,
    #[prost(message, repeated, tag = "7")]
    pub rays: Vec<Ray>,
}

impl Translatable for LidarMeasurements {
    const MSG_TYPE: MessageType = MessageType::LidarMappingMessage;
}