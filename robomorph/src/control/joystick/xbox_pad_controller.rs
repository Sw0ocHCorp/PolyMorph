//! Xbox gamepad input through the `gilrs` crate, scheduled as a `Process` (100 Hz in the Gazebo
//! binary). Each tick polls at most `n_listening` pending events with the non-blocking
//! `Gilrs::next_event()` and returns a `RemoteControl` snapshot on the pipe.

use std::{ptr::null, time::Duration};

use gilrs::{Event, Gilrs};
use tokio::sync::broadcast::{Receiver, Sender};

use crate::{control::joystick::joystick_controller::{JoyStickController, RemoteControl}, core::scheduler::Process, messages::registered_message::AnyMessage};


/// Gamepad process. `connect` opens gilrs, `exec` polls it; see `RemoteControl` for the snapshot
/// semantics (fields only set for the events seen in the tick).
pub struct XboxPadControl {
    /// Process name (log prints).
    name: String,
    /// Broadcast sender given by `set_sender`.
    // NOTE: stored but never used to send: the snapshot only leaves through the pipe (`exec` return
    // value). The Gazebo binary does not wire it either.
    button_pressed_sender: Option<Sender<AnyMessage>>,
    /// Execution period set by `set_period_from_freq`.
    period: Duration,
    /// gilrs context, `Some` between `connect` and `disconnect`.
    pad: Option<Gilrs>,
    /// Maximum number of events polled per `exec` (`consecutive_listenings` in `new`).
    n_listening: u8
}

impl Process for XboxPadControl {
    /// Polls the pad and hands the snapshot to the next process of the chain (pipe). The pipe input
    /// is ignored.
    // NOTE: on a tick without any event this returns `Some(RemoteControl::default())`, an all-zero
    // snapshot; `None` is returned only when the pad is not connected.
    fn exec(&mut self, _input: &Option<AnyMessage>, _dt: std::time::Duration) -> Option<AnyMessage> {
        match self.scan_buttons() {
            Some(rc) => 
                return Some(AnyMessage::RemoteControl(rc)),
            None => 
                return None,
        }
    }

    fn set_name(&mut self, name: String) {
        self.name= name;
    }
    fn get_name(&self) ->String {
        return self.name.clone();
    }

    /// The pad has no asynchronous input: the receiver is dropped with a warning.
    fn set_receiver(&mut self, _receiver: Receiver<AnyMessage>) {
        println!("[WARNING] -> {} doesn't have any receiver", self.name);
    }

    fn set_sender(&mut self, sender: Sender<AnyMessage>) {
        self.button_pressed_sender= Some(sender);
    }

    fn set_period_from_freq(&mut self, frequency: u64) {
        self.period= Duration::from_nanos(1_000_000_000 / frequency);
    }

    fn get_period(&self) -> std::time::Duration {
        return self.period;
    }
}

impl JoyStickController for XboxPadControl {
    /// Initialise gilrs; on failure the pad stays `None` and `exec` returns `None` at every tick.
    fn connect(&mut self) {
        if let Ok(pad) = Gilrs::new() {
            self.pad= Some(pad);
        } else {
            println!("[WARNING] -> {} connexion failed", self.name);
        }
    }

    /// Poll up to `n_listening` pending events. `next_event()` is non-blocking (it returns `None` at
    /// once when the queue is empty, so spare iterations cost nothing). Only two event kinds are
    /// mapped: `ButtonReleased` -> `button_pressed`, `AxisChanged` -> sticks / triggers. When several
    /// events of the same kind arrive in one tick the last one wins.
    fn scan_buttons(&mut self) -> Option<RemoteControl> {
        if let Some(pad) = &mut self.pad {
            let mut rc= RemoteControl::default();
            for _ in 0..self.n_listening {
                if let Some(Event{id, event, ..})= pad.next_event() {
                    match event {
                        gilrs::EventType::ButtonReleased(button, _) => {
                            rc.button_pressed= button as u32;
                        },
                        gilrs::EventType::AxisChanged(axis, val, _) => {
                            match axis {
                                gilrs::Axis::LeftStickX => rc.left_stick_x= val as f64,
                                gilrs::Axis::LeftStickY => rc.left_stick_y= val as f64,
                                gilrs::Axis::LeftZ => rc.stick_pressed= axis as u32,
                                gilrs::Axis::RightStickX => rc.right_stick_x= val as f64,
                                gilrs::Axis::RightStickY => rc.right_stick_y= val as f64,
                                gilrs::Axis::RightZ => rc.stick_pressed= axis as u32,
                                _ => {},
                            }
                        },
                        _ => {},
                    }
                }
            }
            return Some(rc);
        }
        return None;
    }

    /// Drop the gilrs context.
    fn disconnect(&mut self) {
        self.pad= None;
    }
}

impl XboxPadControl {
    /// `consecutive_listenings` is the maximum number of events polled per tick (`n_listening`).
    /// The pad is not opened here: call `connect`.
    pub fn new(name: String, consecutive_listenings: u8) -> Self {
        return Self { name, button_pressed_sender: None, period: Duration::from_millis(0), pad: None , n_listening: consecutive_listenings};
    }
}