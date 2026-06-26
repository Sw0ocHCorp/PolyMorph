use prost_derive::{Enumeration, Message};

use crate::messages::registered_message::{MessageType, Translatable};

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
    TORQUE= 7,
}

/// Struct serializable / deserializable using protobuf that represent a motor command
#[derive(Clone, PartialEq, Message)]
pub struct MotorCommand {
    #[prost(uint32, tag = "1")]
    pub id: u32,
    #[prost(enumeration = "MotorCommandType", tag = "2")]
    pub command_type: i32,
    #[prost(double, tag = "3")]
    pub setpoint_value: f64,
}

/// Struct serializable / deserializable using protobuf that represent motor specs, the last command applied, it's status, ...
#[derive(Clone, PartialEq, Message)]
pub struct MotorFeedBack {
    #[prost(uint32, tag = "1")]
    pub id: u32,
    #[prost(enumeration = "MotorStatus", tag = "2")]
    pub status: i32,
    #[prost(double, tag = "3")]
    pub current_value: f64,
    #[prost(double, tag = "4")]
    pub setpoint_value: f64,
    #[prost(enumeration = "MotorCommandType", tag = "5")]
    pub command_type: i32,
    #[prost(double, tag = "6")]
    pub min_value: f64,
    #[prost(double, tag = "7")]
    pub max_value: f64,
    #[prost(double, tag = "8")]
    pub p: f64,
    #[prost(double, tag = "9")]
    pub i: f64,
    #[prost(double, tag = "10")]
    pub d: f64,
    #[prost(uint32, tag = "11")]
    pub control_frequency: u32,
}

impl Translatable for MotorFeedBack {
    const MSG_TYPE: MessageType= MessageType::MotorFeedBackMessage;
}