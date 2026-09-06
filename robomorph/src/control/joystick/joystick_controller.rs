//! Remote control input: the `RemoteControl` protobuf message (wire tag
//! `MessageType::RemoteControlMessage`) and the `JoyStickController` trait, implemented by
//! `xbox_pad_controller::XboxPadControl`.

use gilrs::{Axis, Button};
use prost_derive::Message;
use prost_types::Timestamp;

use crate::{core::scheduler::Process, messages::registered_message::{MessageType, Translatable}};

/// Snapshot of the gamepad events seen during ONE tick of the joystick process
/// (`JoyStickController::scan_buttons`). Fields are only written for the events seen in that tick
/// and stay at their default (`0`) otherwise: a zero stick value means "no axis event this tick",
/// not necessarily "stick centred". It is not a persistent state of the pad.
///
/// Travels as `AnyMessage::RemoteControl` (on the pipe in the current wiring). The button and
/// axis codes are the discriminants of the gilrs `Button` / `Axis` enums.
#[derive(Clone, PartialEq, Message)]
pub struct RemoteControl {
    /// Time of the snapshot.
    // NOTE: never filled by `XboxPadControl` today (stays `None`).
    #[prost(message, optional, tag = "1")]
    pub timestamp: Option<Timestamp>,
    /// gilrs `Button` discriminant of the button whose event was seen this tick (`0` = none).
    // NOTE: despite the name it is set on the `ButtonReleased` event, not on the press.
    #[prost(uint32, tag = "2")]
    pub button_pressed: u32,
    /// gilrs `Axis` discriminant of the analogue trigger (`LeftZ` / `RightZ`) that moved this tick
    /// (`0` = none); the trigger value itself is not kept.
    // NOTE: despite the name it does not report a stick click (gilrs reports those as buttons).
    #[prost(uint32, tag = "3")]
    pub stick_pressed: u32,
    /// Left stick horizontal position in `[-1, 1]` as reported by gilrs (`0` when no event this tick).
    #[prost(double, tag = "4")]
    pub left_stick_x: f64,
    /// Left stick vertical position in `[-1, 1]` (gilrs: up is positive).
    #[prost(double, tag = "5")]
    pub left_stick_y: f64,
    /// Right stick horizontal position in `[-1, 1]`.
    #[prost(double, tag = "6")]
    pub right_stick_x: f64,
    /// Right stick vertical position in `[-1, 1]` (gilrs: up is positive).
    #[prost(double, tag = "7")]
    pub right_stick_y: f64,
}

impl Translatable for RemoteControl {
    const MSG_TYPE: MessageType= MessageType::RemoteControlMessage;
}

/// Contract of a remote-control input device. It is also a `Process`: `exec` is expected to call
/// `scan_buttons` and return the snapshot as `AnyMessage::RemoteControl` on the pipe.
pub trait JoyStickController : Process {

    /// Open the device (e.g. initialise gilrs). A failure is logged by the implementation, not returned.
    fn connect(&mut self);

    /// Poll the pending device events and return the snapshot for this tick; `None` when the device
    /// is not connected.
    fn scan_buttons(&mut self) -> Option<RemoteControl>;

    /// Release the device.
    fn disconnect(&mut self);

}