//! Force -> attitude resolver: the stage between the velocity stage and the attitude stage. It
//! answers "which attitude must the vehicle take so that the desired force becomes producible?".
//! Contract: `docs/docs/pipelines/motor-control/resolver.mdx`.
//!
//! The setpoint `Pose` comes down the pipe. This stage READS `force` (f_d, world, N, filled by the
//! velocity stage) and `orientation` (the attitude preference q_pref, filled by the mission), and
//! FILLS `orientation` (q_d) and `imu_measurement.a_velocity` (omega_d, body, rad/s) for the attitude
//! stage. `force` goes on unchanged: the attitude stage rotates it into the body frame.
//!
//! STUB for now (brick 3): q_d = q_pref and omega_d = 0. In stabilize this is already the exact
//! answer: f_d = (0, 0, m g) is producible at the preferred, level attitude. The alignment
//! (reachable force subspace S and authority axes derived from the motor tree,
//! q_d = q_align (x) q_pref) comes with brick 4, without changing what the stage reads and fills.

use std::time::Duration;

use tokio::sync::broadcast::{Receiver, Sender};

use crate::{control::motion::motion_controller::MotionController, core::scheduler::Process, messages::{pose_messages::{IMUMeasurements, Pose}, registered_message::{AnyMessage, Vec3}}};

/// The resolver. Stateless: everything it needs comes down the pipe in the setpoint of the pass.
pub struct ForceToAttitudeResolver {
    name: String,
    period: Duration,
}

impl ForceToAttitudeResolver {
    pub fn new(name: String) -> Self {
        return Self { name, period: Duration::from_millis(0) };
    }
}

impl Process for ForceToAttitudeResolver {
    fn set_name(&mut self, name: String) {
        self.name= name;
    }

    /// One cycle: the law on the setpoint of THIS pass (pipe input, never held: the velocity stage
    /// runs in every pass the resolver runs), the result handed to the next process through the pipe
    /// (or `None`: abstain, the attitude stage holds its last setpoint).
    fn exec(&mut self, input: &Option<AnyMessage>, dt: Duration) -> Option<AnyMessage> {
        return self.compute_command_law(None, input.clone(), dt, false);
    }

    /// Not used: the stub needs no measurement. Brick 4 reads the current attitude q here, for
    /// tie-breaking and hysteresis only.
    fn set_receiver(&mut self, _receiver: Receiver<AnyMessage>) {
        println!("[WARNING::ForceToAttitudeResolver] -> no telemetry input yet: receiver ignored");
    }

    /// Not used: the setpoint goes through the pipe to the attitude stage.
    fn set_sender(&mut self, _sender: Sender<AnyMessage>) {
        println!("[WARNING::ForceToAttitudeResolver] -> the setpoint goes through the pipe: sender ignored");
    }

    fn set_period_from_freq(&mut self, frequency: u64) {
        self.period= Duration::from_nanos(1_000_000_000 / frequency);
    }

    fn get_period(&self) -> Duration {
        return self.period;
    }

    fn get_name(&self) -> String {
        return self.name.clone();
    }
}

impl MotionController for ForceToAttitudeResolver {
    /// The law (STUB), stateless:
    ///   1. the setpoint `Pose` of this pass (`setpoint`); none: abstain [P3]
    ///   2. it must carry f_d (`force`) and q_pref (`orientation`); either missing: abstain [P3]
    ///   3. q_d = q_pref (kept in `orientation`), omega_d = 0 (written into `imu_measurement`),
    ///      f_d unchanged
    /// Brick 4 replaces step 3 by the alignment.
    fn compute_command_law(&mut self, _input_data: Option<AnyMessage>, setpoint: Option<AnyMessage>, _dt: Duration, _verbose: bool) -> Option<AnyMessage> {
        // Step 1. the setpoint of this pass; nothing came down the pipe: abstain
        let Some(AnyMessage::PoseState(mut setpoint))= setpoint else {
            return None;
        };
        // Step 2. what the resolver reads: the force to make producible, and the preference
        if setpoint.force.is_none() || setpoint.orientation.is_none() {
            return None;
        }
        // Step 3. STUB: the preference is the attitude to reach (orientation left as it is), at zero
        // rate; the force goes on unchanged
        let mut rates= setpoint.imu_measurement.unwrap_or_default();
        rates.a_velocity= Some(Vec3::new(0.0, 0.0, 0.0));
        setpoint.imu_measurement= Some(rates);
        return Some(AnyMessage::PoseState(setpoint));
    }

    /// Not used: the preference comes down the pipe, inside the setpoint.
    fn set_setpoint_receiver(&mut self, _receiver: Receiver<Pose>) {
        println!("[WARNING::ForceToAttitudeResolver] -> the setpoint comes through the pipe: receiver ignored");
    }
}

/// Hand-predicted values: the stub must hand over the preference itself, a zero rate, and the very
/// force it received; and abstain when either input is missing.
#[cfg(test)]
mod tests {
    use nalgebra::{UnitQuaternion, Vector3};

    use crate::messages::registered_message::UnitQuat;

    use super::*;

    /// A setpoint as the velocity stage hands it over: a 30 deg yaw preference (a non-trivial
    /// q_pref) and a force that is NOT the weight, to see both pass unchanged.
    fn incoming() -> Pose {
        let mut setpoint= Pose::default();
        setpoint.orientation= Some(UnitQuat::from(UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 30f64.to_radians())));
        setpoint.force= Some(Vec3::new(1.0, -2.0, 11.772));
        return setpoint;
    }

    #[test]
    fn the_stub_keeps_the_preference_and_the_force() {
        // 1. one pass
        let mut resolver= ForceToAttitudeResolver::new("resolver".to_string());
        let Some(AnyMessage::PoseState(setpoint))= resolver.exec(&Some(AnyMessage::PoseState(incoming())), Duration::from_millis(10)) else {
            panic!("no setpoint handed over");
        };
        // 2. q_d = q_pref exactly, f_d unchanged, omega_d = 0 written into the setpoint
        assert_eq!(setpoint.orientation, incoming().orientation);
        assert_eq!(setpoint.force, incoming().force);
        assert_eq!(setpoint.imu_measurement.and_then(|imu| imu.a_velocity), Some(Vec3::new(0.0, 0.0, 0.0)));
    }

    #[test]
    fn the_stub_abstains_without_force_or_preference() {
        let mut resolver= ForceToAttitudeResolver::new("resolver".to_string());
        // 1. nothing in the pipe
        assert!(resolver.exec(&None, Duration::from_millis(10)).is_none());
        // 2. a preference but no force (no velocity stage before): abstain
        let mut no_force= incoming();
        no_force.force= None;
        assert!(resolver.exec(&Some(AnyMessage::PoseState(no_force)), Duration::from_millis(10)).is_none());
        // 3. a force but no preference: abstain
        let mut no_orientation= incoming();
        no_orientation.orientation= None;
        assert!(resolver.exec(&Some(AnyMessage::PoseState(no_orientation)), Duration::from_millis(10)).is_none());
    }
}
