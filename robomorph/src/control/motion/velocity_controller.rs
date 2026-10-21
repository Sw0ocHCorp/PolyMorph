//! Velocity stage of the cascade, MINIMAL for now: its law is a_d = 0, so the force it asks for is
//! the one that holds the vehicle at rest in the air, f_d = m (a_d + g z) = (0, 0, m g), WORLD
//! frame. It sits where the velocity loop will run: brick 5 replaces the law
//! (a_d = kp (v_d - v) + ki int(v_d - v), with the generic `PIDController<Vec3>`) without changing
//! its place in the pipeline or its output.
//! Contract: `docs/docs/pipelines/motor-control/roadmap.mdx`, "Velocity loop".
//!
//! The setpoint `Pose` comes down the pipe (the mission: attitude preference, desired velocity v_d).
//! This stage READS nothing of it yet and FILLS `force`, then hands it on to the next stage.
//! There is no mode flag: in stabilize, f_d = (0, 0, m g) is exactly the output of a velocity loop
//! at rest.

use std::time::Duration;

use tokio::sync::broadcast::{Receiver, Sender};

use crate::{control::motion::motion_controller::{GRAVITY, MotionController, VehicleKinematicConfig}, core::scheduler::Process, messages::{pose_messages::Pose, registered_message::{AnyMessage, Vec3}}};

/// The velocity stage: adds f_d (world, N) to the setpoint `Pose` coming down the pipe, and hands
/// it on to the process registered right after it (the resolver).
pub struct VelocityController {
    name: String,
    vehicle_config: VehicleKinematicConfig,
    period: Duration,
}

impl VelocityController {
    pub fn new(name: String, vehicle_config: VehicleKinematicConfig) -> Self {
        return Self { name, vehicle_config, period: Duration::from_millis(0) };
    }
}

impl Process for VelocityController {
    fn set_name(&mut self, name: String) {
        self.name= name;
    }

    /// One cycle: the law on the setpoint of THIS pass (pipe input, never held: the mission runs in
    /// every pass this stage runs), the enriched setpoint handed to the next process through the pipe.
    fn exec(&mut self, input: &Option<AnyMessage>, dt: Duration) -> Option<AnyMessage> {
        return self.compute_command_law(None, input.clone(), dt, false);
    }

    /// Not used while a_d = 0: the measured velocity enters the law with brick 5.
    fn set_receiver(&mut self, _receiver: Receiver<AnyMessage>) {
        println!("[WARNING::VelocityController] -> no telemetry input yet (law a_d = 0): receiver ignored");
    }

    /// Not used: the setpoint goes through the pipe to the next stage.
    fn set_sender(&mut self, _sender: Sender<AnyMessage>) {
        println!("[WARNING::VelocityController] -> the force goes through the pipe: sender ignored");
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

impl MotionController for VelocityController {
    /// The law, stateless:
    ///   1. the setpoint `Pose` of this pass (`setpoint`); none: abstain [P3]
    ///   2. a_d = 0                     m/s^2, world: the law of brick 5 replaces this line
    ///   3. f_d = m (a_d + g z)         N, world: the force producing a_d PLUS the weight compensation
    ///   4. the setpoint, with `force` = f_d, every other field untouched
    /// The measurement (`input_data`) is not read yet: `None` today.
    fn compute_command_law(&mut self, _input_data: Option<AnyMessage>, setpoint: Option<AnyMessage>, _dt: Duration, _verbose: bool) -> Option<AnyMessage> {
        // Step 1. the setpoint to enrich; nothing came down the pipe (no mission setpoint yet): abstain
        let Some(AnyMessage::PoseState(mut setpoint))= setpoint else {
            return None;
        };
        // Step 2. the desired acceleration: zero, the vehicle is to stay at rest
        let a_d= Vec3::new(0.0, 0.0, 0.0);
        // Step 3. the force that produces a_d, plus the one that cancels the weight: both in the
        // world frame, where gravity is (0, 0, -g) whatever the attitude
        let f_d= Vec3::new(self.vehicle_config.weight * a_d.x,
                           self.vehicle_config.weight * a_d.y,
                           self.vehicle_config.weight * (a_d.z + GRAVITY));
        // Step 4. this stage's contribution to the setpoint; the rest goes on unchanged
        setpoint.force= Some(f_d);
        return Some(AnyMessage::PoseState(setpoint));
    }

    /// Not used while a_d = 0: the desired velocity v_d enters the law with brick 5.
    fn set_setpoint_receiver(&mut self, _receiver: Receiver<Pose>) {
        println!("[WARNING::VelocityController] -> no setpoint input yet (law a_d = 0): receiver ignored");
    }
}

/// Hand-predicted values: a vehicle of 1.2 kg needs 1.2 * 9.81 = 11.772 N upward, world frame.
#[cfg(test)]
mod tests {
    use nalgebra::Matrix3;

    use crate::messages::registered_message::UnitQuat;

    use super::*;

    /// The velocity stage of a 1.2 kg vehicle; the rest of its description does not enter this law.
    fn velocity() -> VelocityController {
        let vehicle_config= VehicleKinematicConfig {
            com_relative_location: Vec3::new(0.0, 0.0, 0.0),
            error_angular_factor: 0.0,
            error_linear_factor: 0.0,
            error_attitude_factor: 0.0,
            weight: 1.2,
            moments_matrix: Matrix3::identity(),
        };
        return VelocityController::new("velocity".to_string(), vehicle_config);
    }

    #[test]
    fn at_rest_the_setpoint_gets_the_weight_compensation() {
        // 1. the mission setpoint coming down the pipe: level, at rest
        let mut mission= Pose::default();
        mission.orientation= Some(UnitQuat { w: 1.0, x: 0.0, y: 0.0, z: 0.0 });
        mission.l_velocity= Some(Vec3::new(0.0, 0.0, 0.0));
        // 2. one pass: the same setpoint goes on, with f_d = (0, 0, 11.772) N in the world frame
        let Some(AnyMessage::PoseState(setpoint))= velocity().exec(&Some(AnyMessage::PoseState(mission.clone())), Duration::from_millis(10)) else {
            panic!("no setpoint handed over");
        };
        let f_d= setpoint.force.expect("no force added");
        assert!(f_d.x == 0.0 && f_d.y == 0.0);
        assert!((f_d.z - 11.772).abs() < 1e-12, "f_d.z = {}", f_d.z);
        // 3. what the stage does not produce is passed on untouched
        assert_eq!(setpoint.orientation, mission.orientation);
        assert_eq!(setpoint.l_velocity, mission.l_velocity);
    }

    #[test]
    fn without_a_setpoint_the_stage_abstains() {
        // nothing in the pipe (no mission setpoint yet): nothing to enrich, abstain [P3]
        assert!(velocity().exec(&None, Duration::from_millis(10)).is_none());
    }
}
