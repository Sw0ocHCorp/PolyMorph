//! External links of the stack (`HardwareInterface`: UDP today, UART later) and the wire codec
//! shared by all of them: a frame is `[1-byte MessageType][protobuf payload]`.

use crate::{core::scheduler::Process, messages::{lidar_messages::LidarMeasurements, motor_messages::MotorFeedBack, pose_messages::{GNSSMeasurement, IMUMeasurements, Pose}, registered_message::{AnyMessage, MessageType, Translatable}}};

/// Trait that give the fundamental methods of all the hardware communication interfaces (UART / UDP / etc...)
///
/// An interface is NOT a scheduled `Process`: the scheduler only calls `connect()` once
/// (`Scheduler::start_all_interfaces`) and `disconnect()` at shutdown. The interface runs its own
/// RX / TX threads and exchanges `AnyMessage`s with the processes through broadcast channels.
pub trait HardwareInterface {
    /// Open the link and start the background threads. `Err` carries a human readable reason.
    fn connect(&mut self) -> Result<(), String>;
    /// Encode `msg` and send it synchronously on the caller's thread (bypasses the TX thread).
    fn send_message(&mut self, msg: AnyMessage);
    /// Pull-style read of one incoming message, when the implementation supports it
    /// (`UdpInterface` pushes received frames on its broadcast channel instead and returns `None`).
    fn listen(&mut self) -> Option<AnyMessage>;
    /// Close the link and join the background threads.
    fn disconnect(&mut self);
}


/// Decode incoming byte array to protobuf struct
/// 
/// Notes:
/// 
/// Generic function because the is only one way to decode byte array to protobuf struct
///
/// Dispatches on the tag byte `buf[0]` and prost-decodes the rest. Returns `None` for a frame
/// shorter than 2 bytes, an unknown tag, or a payload that fails to decode.
// NOTE: `MotorModelMessage` and `RemoteControlMessage` have no arm here although both types are
// encodable (`Translatable`): such frames are silently dropped on reception.
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
// NOTE: `MotorCommands` and `VehicleWrench` are internal-only and encode to an EMPTY frame. The
// UDP TX thread sends whatever this returns, so such a message reaching the interface's channel
// goes out as an empty datagram, which is exactly the "poison pill" a `UdpInterface` RX thread
// interprets as a stop request (see `UdpInterface::disconnect`).
pub fn encode_frame(msg: &AnyMessage) -> Vec<u8> {
    match msg {
        AnyMessage::ImuState(m)   => m.to_frame(),
        AnyMessage::GnssState(m)  => m.to_frame(),
        AnyMessage::MotorState(m) => m.to_frame(),
        AnyMessage::PoseState(m)  => m.to_frame(),
        AnyMessage::LidarState(m) => m.to_frame(),
        AnyMessage::MotorCommands(m) => return vec![],
        AnyMessage::VehicleWrench(work_vec) => return vec![],
        AnyMessage::RemoteControl(m) => {return m.to_frame()},
    }
}