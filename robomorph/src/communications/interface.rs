use crate::{core::scheduler::Process, messages::{lidar_messages::LidarMeasurements, motor_messages::MotorFeedBack, pose_messages::{GNSSMeasurement, IMUMeasurements, Pose}, registered_message::{AnyMessage, MessageType, Translatable}}};

/// Trait that give the fundamental methods of all the hardware communication interfaces (UART / UDP / etc...)
pub trait HardwareInterface {
    fn connect(&mut self) -> Result<(), String>;
    fn send_message(&mut self, msg: AnyMessage);
    fn listen(&mut self) -> Option<AnyMessage>;
    fn disconnect(&mut self);
    /// Set the frame receiver that is frame to send buffer
    fn set_outbound_rx(&mut self, receiver: tokio::sync::mpsc::Receiver<AnyMessage>);
    /// Connect the interface to a process to be able to send incoming data and receive frame to send from the given Process
    fn connect_process(&mut self, proc: &mut dyn Process);
}


/// Decode incoming byte array to protobuf struct
/// 
/// Notes:
/// 
/// Generic function because the is only one way to decode byte array to protobuf struct
pub fn decode_frame(buf: &[u8]) -> Option<AnyMessage> {
    if buf.len() < 2 { return None; }
    match buf[0] {
        x if x == MessageType::ImuRawMessage as u8 =>
            IMUMeasurements::from_frame(buf).ok().map(AnyMessage::ImuState),
        x if x == MessageType::GNSSRawMessage as u8 =>
            GNSSMeasurement::from_frame(buf).ok().map(AnyMessage::GnssState),
        x if x == MessageType::MotorFeedBackMessage as u8 =>
            MotorFeedBack::from_frame(buf).ok().map(AnyMessage::MotorState),
        x if x == MessageType::PoseMessage as u8 =>
            Pose::from_frame(buf).ok().map(AnyMessage::PoseState),
        x if x == MessageType::LidarMappingMessage as u8 =>
            LidarMeasurements::from_frame(buf).ok().map(AnyMessage::LidarState),
        _ => None,
    }
}

/// Encode protobuf struct to frame (byte array)
/// 
/// Notes:
/// 
/// Generic function because the is only one way to encode protobuf struct in u8 vector
pub fn encode_frame(msg: &AnyMessage) -> Vec<u8> {
    match msg {
        AnyMessage::ImuState(m)   => m.to_frame(),
        AnyMessage::GnssState(m)  => m.to_frame(),
        AnyMessage::MotorState(m) => m.to_frame(),
        AnyMessage::PoseState(m)  => m.to_frame(),
        AnyMessage::LidarState(m) => m.to_frame(),
        AnyMessage::MotorCommands(m) => {return vec![]},
    }
}