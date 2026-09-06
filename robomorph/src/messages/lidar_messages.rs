//! Lidar messages: the scan geometry and the rays of one sweep (`LidarMeasurements`, wire tag
//! `MessageType::LidarMappingMessage`), built by the vehicle controller from a Gazebo `LaserScan`
//! (one `Ray` per range sample). Not consumed by the control loops today.

use prost::Message;
use prost_types::Timestamp;

use crate::messages::registered_message::{MessageType, Translatable};

/// Struct serializable / deserializable using protobuf that represent a Ray of the lidar
///
/// Angles are in the sensor frame, in radians, as delivered by the scan.
#[derive(Clone, PartialEq, Message)]
pub struct Ray {
    /// Elevation of the ray (rad).
    #[prost(double, tag = "1")]
    pub vertical_angle: f64,
    /// Azimuth of the ray (rad).
    #[prost(double, tag = "2")]
    pub horizontal_angle: f64,
    /// Measured range along the ray (m).
    #[prost(double, tag = "3")]
    pub length: f64,
}

/// Struct serializable / deserializable using protobuf that represent a Point in 3D space
///
/// Cartesian point (m). Not part of any message today (unused): intended for the map rebuilt
/// from the rays.
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
    #[prost(message, optional, tag = "1")]
    pub timestamp: Option<Timestamp>,
    /// Lower elevation bound of the sweep (rad).
    #[prost(double, tag = "2")]
    pub vertical_min_angle: f64,
    /// Elevation step between two rays (rad).
    #[prost(double, tag = "3")]
    pub vertical_angle_resolution: f64,
    /// Upper elevation bound of the sweep (rad).
    #[prost(double, tag = "4")]
    pub vertical_max_angle: f64,
    /// Lower azimuth bound of the sweep (rad).
    #[prost(double, tag = "5")]
    pub horizontal_min_angle: f64,
    /// Azimuth step between two rays (rad).
    #[prost(double, tag = "6")]
    pub horizontal_angle_resolution: f64,
    /// Upper azimuth bound of the sweep (rad).
    #[prost(double, tag = "7")]
    pub horizontal_max_angle: f64,
    /// One ray per range sample, in scan order.
    #[prost(message, repeated, tag = "8")]
    pub rays: Vec<Ray>,
}

impl Translatable for LidarMeasurements {
    const MSG_TYPE: MessageType = MessageType::LidarMappingMessage;
}