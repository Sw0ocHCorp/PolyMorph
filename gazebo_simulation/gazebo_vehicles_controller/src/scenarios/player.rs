//! `ScenarioPlayer`: plays a scenario on the SIMULATED clock and produces the mission setpoint.
//!
//! It is the single source of setpoints of the binary, so a stage never sees two producers fighting
//! over its setpoint. Registered after the vehicle bridge, it reads the simulated time of the pose
//! published in the same pass, and hands the setpoint `Pose` (attitude preference, desired velocity)
//! to the head of the control cascade through the PIPE, in the same pass [P4]. It also publishes it
//! on its setpoint channel, for the recorder.

use std::time::Duration;

use prost_types::Timestamp;
use robomorph::{core::scheduler::Process, messages::{pose_messages::Pose, registered_message::AnyMessage}};
use tokio::sync::broadcast::{Receiver, Sender};

use crate::scenarios::{Scenario, timestamp_to_s};

/// The scenario player process. Time zero of the scenario is the first simulated time it sees.
pub struct ScenarioPlayer {
    name: String,
    scenario: Scenario,
    period: Duration,
    /// Telemetry channel (`AnyMessage::PoseState`): only the timestamp of the pose is used.
    telemetry_receiver: Option<Receiver<AnyMessage>>,
    /// Setpoint channel, typed `Pose` like the setpoint receivers of the motion stages.
    setpoint_sender: Option<Sender<Pose>>,
    /// Simulated time (s) of the first stamped pose: t = 0 of the scenario. `None` until then [P3].
    start_s: Option<f64>,
    /// Latest simulated time seen, kept as received to stamp the setpoint with it.
    last_stamp: Option<Timestamp>,
    /// The end of the scripted part is announced once.
    end_announced: bool,
}

impl ScenarioPlayer {
    pub fn new(name: String, scenario: Scenario) -> Self {
        return Self { name, scenario, period: Duration::from_millis(0),
                        telemetry_receiver: None, setpoint_sender: None,
                        start_s: None, last_stamp: None, end_announced: false };
    }

    /// The setpoint channel: a copy of each setpoint, for the processes that are not in the cascade
    /// (the recorder).
    pub fn set_setpoint_sender(&mut self, sender: Sender<Pose>) {
        self.setpoint_sender= Some(sender);
    }
}

impl Process for ScenarioPlayer {
    /// One pass: read the simulated time, publish the setpoint of the scenario at that time.
    fn exec(&mut self, _input: &Option<AnyMessage>, _dt: Duration) -> Option<AnyMessage> {
        // 1. the freshest simulated time of this pass: drain the telemetry, keep the last stamp
        if let Some(telemetry_receiver)= &mut self.telemetry_receiver {
            let sz= telemetry_receiver.len();
            for _ in 0..sz {
                if let Ok(AnyMessage::PoseState(pose))= telemetry_receiver.try_recv() && let Some(stamp)= pose.timestamp {
                    self.last_stamp= Some(stamp);
                }
            }
        }
        // 2. [P3] no simulated time yet (Gazebo still paused, no sample received): no setpoint at
        //    all. The attitude stage abstains and the mixer holds; nothing is invented.
        let Some(stamp)= self.last_stamp else {
            return None;
        };
        let now_s= timestamp_to_s(&stamp);
        // 3. the scenario starts at the first simulated time seen: t = 0 there
        let start_s= *self.start_s.get_or_insert(now_s);
        let t_s= now_s - start_s;
        // 4. the setpoint of the active segment, stamped with the simulated time it was computed for
        let mut setpoint= self.scenario.setpoint_at(t_s);
        setpoint.timestamp= Some(stamp);
        if let Some(setpoint_sender)= &self.setpoint_sender {
            // never blocks; an Err only means nobody listens, which is not an error here
            let _= setpoint_sender.send(setpoint.clone());
        }
        // 5. tell the operator, once, that the scripted part is over (the last setpoint is held)
        if !self.end_announced && self.scenario.is_finished(t_s) {
            println!("[SCENARIO] {} finished at t = {:.2} s (simulated): the recording is complete, stop the controller (Ctrl-C)",
                        self.scenario.name, t_s);
            self.end_announced= true;
        }
        // 6. the setpoint goes down the pipe, to the head of the control cascade
        return Some(AnyMessage::PoseState(setpoint));
    }

    fn set_receiver(&mut self, receiver: Receiver<AnyMessage>) {
        self.telemetry_receiver= Some(receiver);
    }

    /// The player publishes `Pose` setpoints, not `AnyMessage`: see `set_setpoint_sender`.
    fn set_sender(&mut self, _sender: Sender<AnyMessage>) {
        println!("[WARNING] -> {} publishes on its setpoint channel only (set_setpoint_sender)", self.name);
    }

    fn set_name(&mut self, name: String) {
        self.name= name;
    }

    fn get_name(&self) -> String {
        return self.name.clone();
    }

    fn set_period_from_freq(&mut self, frequency: u64) {
        self.period= Duration::from_nanos(1_000_000_000 / frequency);
    }

    fn get_period(&self) -> Duration {
        return self.period;
    }
}

/// The player's output: the setpoint of the scenario at the simulated time, handed to the next
/// process through the pipe.
#[cfg(test)]
mod tests {
    use prost_types::Timestamp;
    use tokio::sync::broadcast::channel;

    use crate::scenarios::by_name;

    use super::*;

    #[test]
    fn the_setpoint_goes_down_the_pipe() {
        // 1. the player of roll_step, and a telemetry channel carrying one stamped pose
        let (telemetry, receiver)= channel(4);
        let mut player= ScenarioPlayer::new("player".to_string(), by_name("roll_step").unwrap());
        player.set_receiver(receiver);
        // 2. no simulated time yet: no setpoint at all [P3]
        assert!(player.exec(&None, Duration::from_millis(10)).is_none());
        // 3. a pose stamped 10 s: the scenario starts there, t = 0, so the setpoint is level
        let mut pose= Pose::default();
        pose.timestamp= Some(Timestamp { seconds: 10, nanos: 0 });
        assert!(telemetry.send(AnyMessage::PoseState(pose)).is_ok());
        let Some(AnyMessage::PoseState(setpoint))= player.exec(&None, Duration::from_millis(10)) else {
            panic!("no setpoint in the pipe");
        };
        // 4. level attitude preference, stamped with the simulated time it was computed for
        assert_eq!(setpoint.orientation.map(|q| (q.w, q.x, q.y, q.z)), Some((1.0, 0.0, 0.0, 0.0)));
        assert_eq!(setpoint.timestamp, Some(Timestamp { seconds: 10, nanos: 0 }));
    }
}

