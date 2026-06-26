use prost::Message;

use crate::messages::{lidar_messages::LidarMeasurements, motor_messages::MotorFeedBack, pose_messages::{GNSSMeasurement, IMUMeasurements, Pose}};

/// Enum that represent the ID of the message can be sent and / or received
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum MessageType {
    ImuRawMessage = 0,
    GNSSRawMessage = 1,
    MotorFeedBackMessage = 2,
    PoseMessage = 3,
    LidarMappingMessage = 4,
}

/// Enum that respresent the different kind of message that can be transmitted
#[derive(Clone)]
pub enum AnyMessage {
    ImuState(IMUMeasurements),
    GnssState(GNSSMeasurement),
    MotorState(MotorFeedBack),
    PoseState(Pose),
    LidarState(LidarMeasurements),
}

/// Trait that allow to parse or fill message params according to it's type
/// 
/// Notes:
/// 
/// Each Struct that implement the Translatable trait must define it's own MSG_TYPE 
/// 
/// to be able to put the message type before the content of the message in frame / decode correctly the incoming frame 
pub trait Translatable: prost::Message + Default + Sized {
    // ID of the type of message
    const MSG_TYPE: MessageType;

    // [1-byte MessageType][protobuf bytes]
    // Add an 1 byte ID in the result frame to know directly what is the type of the message 
    fn to_frame(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(1 + self.encoded_len());
        out.push(Self::MSG_TYPE as u8);
        out.extend_from_slice(&self.encode_to_vec());
        return out;
    }

    /// inverse: drop the tag byte, prost-decode the rest
    fn from_frame(frame: &[u8]) -> Result<Self, prost::DecodeError> {
        <Self as prost::Message>::decode(&frame[1..])
    }
}