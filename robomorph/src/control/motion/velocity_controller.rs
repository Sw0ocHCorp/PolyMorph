//! Velocity stage of the cascade: the velocity loop. From the desired velocity v_d (the mission) and
//! the measured velocity v, both in the WORLD frame, it decides the acceleration
//! a_d = kp (v_d - v) + ki int(v_d - v), with the generic `PIDController<Vec3>`, and asks for the
//! force that produces it on top of the weight compensation, f_d = m (a_d + g z), WORLD frame.
//! The gains come from a time constant tau_v and a damping zeta: kp = 2 zeta / tau_v,
//! ki = 1 / tau_v^2. It holds the cascade's only integrator.
//! Contract and theory: `docs/docs/pipelines/motor-control/roadmap.mdx`, "Velocity loop".
//!
//! The setpoint `Pose` comes down the pipe (the mission: attitude preference, desired velocity v_d
//! in `l_velocity`). This stage READS `l_velocity` and FILLS `force`, then hands it on to the next
//! stage. The measured velocity comes from the telemetry (`set_receiver`). There is no mode flag:
//! without v_d (stabilize) the loop does not run and f_d = (0, 0, m g), exactly the output of a
//! velocity loop at rest.
//!
//! Time: the integral runs on the SIMULATED time between two measured poses (their time stamps),
//! never on the wall clock of the scheduler: when the simulator stalls, the physics does not move.

use std::time::Duration;

use prost_types::Timestamp;
use tokio::sync::broadcast::{Receiver, Sender};

use crate::{control::{motion::motion_controller::{GRAVITY, MotionController, VehicleKinematicConfig}, pid_controller::PIDController}, core::scheduler::Process, messages::{control_messages::VelocitySetpoint, registered_message::{AnyMessage, Vec3}}};

/// The velocity stage: adds f_d (world, N) to the setpoint `Pose` coming down the pipe, and hands
/// it on to the process registered right after it (the resolver).
pub struct VelocityController {
    name: String,
    vehicle_config: VehicleKinematicConfig,
    period: Duration,
    pid: PIDController<Vec3>,
    telemetry_receiver: Option<Receiver<AnyMessage>>,
    /// The last measured velocity (world, m/s), held while no new pose arrives.
    last_velocity: Option<Vec3>,
    /// The simulated time stamp of the pose `last_velocity` comes from.
    last_pose_instant: Option<Timestamp>,
}

impl VelocityController {
    /// `time_constant` tau_v (s) and `damping_factor` zeta (1 = critical) set the gains:
    /// kp = 2 zeta / tau_v, ki = 1 / tau_v^2.
    pub fn new(name: String, vehicle_config: VehicleKinematicConfig, time_constant: f64, damping_factor: f64) -> Self {
        return Self { name, vehicle_config, period: Duration::from_millis(0),
                        pid: PIDController::new(2.0*damping_factor/ time_constant, 1.0 / time_constant.powf(2.0), 0.0, 0.0, f64::INFINITY),
                        telemetry_receiver: None,
                        last_velocity: None,
                        last_pose_instant: None
                    };
    }
}

impl Process for VelocityController {
    fn set_name(&mut self, name: String) {
        self.name= name;
    }

    /// One cycle: the latest measured velocity and the simulated time elapsed since the last one,
    /// then the law on the setpoint of THIS pass (pipe input, never held: the mission runs in every
    /// pass this stage runs), the enriched setpoint handed to the next process through the pipe.
    /// The wall-clock `dt` of the scheduler is not used.
    fn exec(&mut self, input: &Option<AnyMessage>, _dt: Duration) -> Option<AnyMessage> {
        //IF pose measurements from telemetry receiver is available: the newest one wins
        let mut pose= None;
        if let Some(receiver) = self.telemetry_receiver.as_mut() {
            while let Ok(msg)= receiver.try_recv() {
                if let AnyMessage::PoseState(p)= msg {
                    pose= Some(p);
                }
            }
        }
        //the real dt (simulated time between the pose used now and the one used before): 0 without a
        //new pose (stalled simulator), without a previous pose, or without a time stamp
        let mut delta_time= Duration::ZERO;
        if let Some(current_pose)= pose && let Some(measured_velocity)= current_pose.l_velocity {
            //IF the current pose measurement timestamp is available
            if let Some(meas_timestamp)= current_pose.timestamp {
                //IF the last pose measurement timestamp is available
                if let Some(last_pose_timestamp)= self.last_pose_instant {
                    //compute the real dt(time between pose at time T and T-1), never negative
                    delta_time= Duration::from_nanos(((meas_timestamp.seconds * 1_000_000_000 + meas_timestamp.nanos as i64)
                                                    - (last_pose_timestamp.seconds * 1_000_000_000 + last_pose_timestamp.nanos as i64)).max(0) as u64);
                }
                //update the last pose timestamp
                self.last_pose_instant= Some(meas_timestamp);
            }
            self.last_velocity= Some(measured_velocity);
        }
        //the setpoint to enrich; nothing came down the pipe (no mission setpoint yet): abstain [P3]
        let Some(AnyMessage::PoseState(mut setpoint))= input.clone() else {
            return None;
        };
        //compute the output of the speed controller (real target speed) using the real dt, with the
        //last measured velocity (held when no new pose came: none before the first one)
        let f_d= self.compute_command_law(self.last_velocity, Some(VelocitySetpoint::from_pose(&setpoint)), delta_time, false);
        //this stage's contribution to the setpoint; the rest goes on unchanged
        setpoint.force= f_d;
        return Some(AnyMessage::PoseState(setpoint));
    }

    /// The telemetry: the measured pose, whose `l_velocity` (world, m/s) is the measured velocity v.
    fn set_receiver(&mut self, receiver: Receiver<AnyMessage>) {
        self.telemetry_receiver= Some(receiver);
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
    /// v, the last measured velocity (world, m/s).
    type Measurement= Vec3;
    type Setpoint= VelocitySetpoint;
    /// f_d, the force to produce (world, N).
    type Command= Vec3;

    /// The law (its state is the PI's integral):
    ///   1. the setpoint of this pass (`setpoint`); none: abstain [P3]
    ///   2. a_d, m/s^2, world:
    ///        - no desired velocity v_d (stabilize): a_d = 0, and the PI starts again from zero;
    ///        - v_d but no measured velocity v yet: a_d = 0, no regulation blind;
    ///        - otherwise a_d = kp (v_d - v) + ki int(v_d - v), over `dt` of simulated time
    ///   3. f_d = m (a_d + g z)         N, world: the force producing a_d PLUS the weight compensation
    /// `measurement` is the last measured velocity, `None` before the first one.
    fn compute_command_law(&mut self, measurement: Option<Vec3>, setpoint: Option<VelocitySetpoint>, dt: Duration, _verbose: bool) -> Option<Vec3> {
        // Step 1. no setpoint: abstain
        let Some(setpoint)= setpoint else {
            return None;
        };
        // Step 2. the desired acceleration
        //compute the target acceleraion to determine the force to produce to fit this acceleration
        let mut a_d= Vec3::new(0.0, 0.0, 0.0);
        //IF the mission gives no desired velocity (stabilize): no regulation, and the integral is
        //cleared so the loop starts again from zero when a v_d comes back
        if setpoint.v_d.is_none() {
            self.pid.reset();
        }
        //ELSE IF the velocity is measured: the PI on the velocity error (without a measurement, a_d
        //stays 0: the weight is still carried)
        else if let Some(target_lin_vel)= setpoint.v_d && let Some(lin_vel)= measurement {
            a_d= self.pid.compute_output(lin_vel, target_lin_vel, dt, true);
        }
        // Step 3. the force that produces a_d, plus the one that cancels the weight: both in the
        // world frame, where gravity is (0, 0, -g) whatever the attitude
        //f_d = m · a_d + m·g·ẑ
        let f_d= Vec3::new(self.vehicle_config.weight * a_d.x,
                           self.vehicle_config.weight * a_d.y,
                           self.vehicle_config.weight * (a_d.z + GRAVITY));
        return Some(f_d);
    }
}

/// Hand-predicted values, the executable specification of the velocity law (brick 5):
///   - the vehicle: m = 1.2 kg, so the weight compensation is 1.2 * 9.81 = 11.772 N upward;
///   - the loop: tau_v = 2 s, zeta = 1, so kp = 2 zeta / tau_v = 1 /s and ki = 1 / tau_v^2 = 0.25 /s^2;
///   - the error is v_d - v, world frame; f_d = m (a_d + g z), world frame.
/// Only the public API is used (`new`, `set_receiver`, `exec`): how the stage is organised inside is
/// free. `exec` is always given a WALL-CLOCK dt of 10 ms, which the law must ignore: it integrates
/// over the SIMULATED time between two measured poses.
#[cfg(test)]
mod tests {
    use nalgebra::Matrix3;
    use prost_types::Timestamp;
    use tokio::sync::broadcast::{channel, Sender};

    use crate::messages::{pose_messages::Pose, registered_message::UnitQuat};

    use super::*;

    const WALL_DT: Duration= Duration::from_millis(10);
    const WEIGHT_N: f64= 1.2 * GRAVITY;

    /// The velocity stage of a 1.2 kg vehicle, tau_v = 2 s, zeta = 1, listening to `telemetry`.
    fn velocity() -> (VelocityController, Sender<AnyMessage>) {
        let vehicle_config= VehicleKinematicConfig {
            com_relative_location: Vec3::new(0.0, 0.0, 0.0),
            error_angular_factor: 0.0,
            error_linear_factor: 0.0,
            error_attitude_factor: 0.0,
            weight: 1.2,
            moments_matrix: Matrix3::identity(),
        };
        let mut stage= VelocityController::new("velocity".to_string(), vehicle_config, 2.0, 1.0);
        let (telemetry, receiver)= channel(8);
        stage.set_receiver(receiver);
        return (stage, telemetry);
    }

    /// The mission setpoint coming down the pipe: level, with or without a desired velocity v_d.
    fn mission(v_d: Option<Vec3>) -> Option<AnyMessage> {
        let mut pose= Pose::default();
        pose.orientation= Some(UnitQuat { w: 1.0, x: 0.0, y: 0.0, z: 0.0 });
        pose.l_velocity= v_d;
        return Some(AnyMessage::PoseState(pose));
    }

    /// A measured pose, published on the telemetry channel: simulated time `t_s` (s) and velocity `v`
    /// (world, m/s).
    fn measure(telemetry: &Sender<AnyMessage>, t_s: f64, v: Vec3) {
        let mut pose= Pose::default();
        pose.timestamp= Some(Timestamp { seconds: t_s.floor() as i64, nanos: ((t_s - t_s.floor()) * 1e9).round() as i32 });
        pose.l_velocity= Some(v);
        assert!(telemetry.send(AnyMessage::PoseState(pose)).is_ok());
    }

    /// One pass of the stage: the force f_d it adds to the setpoint (world, N).
    fn force(stage: &mut VelocityController, setpoint: Option<AnyMessage>) -> Vec3 {
        let Some(AnyMessage::PoseState(out))= stage.exec(&setpoint, WALL_DT) else {
            panic!("no setpoint handed over");
        };
        return out.force.expect("no force added");
    }

    fn close(a: Vec3, b: Vec3) -> bool {
        return (a.0 - b.0).norm() < 1e-9;
    }

    #[test]
    fn at_rest_the_setpoint_gets_the_weight_compensation() {
        // 1. the mission setpoint coming down the pipe: level, at rest
        let (mut stage, _telemetry)= velocity();
        let Some(AnyMessage::PoseState(mission_pose))= mission(Some(Vec3::new(0.0, 0.0, 0.0))) else { unreachable!() };
        // 2. one pass: the same setpoint goes on, with f_d = (0, 0, 11.772) N in the world frame
        let Some(AnyMessage::PoseState(setpoint))= stage.exec(&Some(AnyMessage::PoseState(mission_pose.clone())), WALL_DT) else {
            panic!("no setpoint handed over");
        };
        let f_d= setpoint.force.expect("no force added");
        assert!(f_d.x == 0.0 && f_d.y == 0.0);
        assert!((f_d.z - 11.772).abs() < 1e-12, "f_d.z = {}", f_d.z);
        // 3. what the stage does not produce is passed on untouched
        assert_eq!(setpoint.orientation, mission_pose.orientation);
        assert_eq!(setpoint.l_velocity, mission_pose.l_velocity);
    }

    #[test]
    fn without_a_setpoint_the_stage_abstains() {
        // nothing in the pipe (no mission setpoint yet): nothing to enrich, abstain [P3]
        let (mut stage, _telemetry)= velocity();
        assert!(stage.exec(&None, WALL_DT).is_none());
    }

    #[test]
    fn without_a_desired_velocity_the_stage_only_carries_the_weight() {
        // the mission gives no v_d (stabilize): no regulation at all, even with the vehicle sliding
        // at 3 m/s, only the weight compensation
        let (mut stage, telemetry)= velocity();
        measure(&telemetry, 10.0, Vec3::new(0.0, 3.0, 0.0));
        assert!(close(force(&mut stage, mission(None)), Vec3::new(0.0, 0.0, WEIGHT_N)));
    }

    #[test]
    fn without_a_measured_velocity_the_loop_does_not_act_blindly() {
        // v_d is given but no pose has been measured yet: a_d = 0, the weight is still carried
        let (mut stage, _telemetry)= velocity();
        assert!(close(force(&mut stage, mission(Some(Vec3::new(0.0, 0.5, 0.0)))), Vec3::new(0.0, 0.0, WEIGHT_N)));
    }

    #[test]
    fn the_first_pass_is_the_proportional_term() {
        // v_d - v = (0, 0.5, 0) on the first measured pose: nothing integrated yet (no time step),
        // a_d = kp e = 1 * 0.5 = 0.5 m/s^2 along y, so f_d = (0, 1.2 * 0.5, 11.772) = (0, 0.6, 11.772)
        let (mut stage, telemetry)= velocity();
        measure(&telemetry, 10.0, Vec3::new(0.0, 0.0, 0.0));
        assert!(close(force(&mut stage, mission(Some(Vec3::new(0.0, 0.5, 0.0)))), Vec3::new(0.0, 0.6, WEIGHT_N)));
    }

    #[test]
    fn the_integral_grows_with_the_simulated_time() {
        // the same error (0, 0.5, 0) on two poses 0.1 s apart in SIMULATED time (exec is still told
        // 10 ms of wall clock): integral = 0.5 * 0.1 = 0.05, a_d = 1 * 0.5 + 0.25 * 0.05 = 0.5125,
        // f_d.y = 1.2 * 0.5125 = 0.615. With the wall-clock dt it would be 0.6015
        let (mut stage, telemetry)= velocity();
        let v_d= Some(Vec3::new(0.0, 0.5, 0.0));
        measure(&telemetry, 10.0, Vec3::new(0.0, 0.0, 0.0));
        force(&mut stage, mission(v_d));
        measure(&telemetry, 10.1, Vec3::new(0.0, 0.0, 0.0));
        let f_d= force(&mut stage, mission(v_d));
        assert!(close(f_d, Vec3::new(0.0, 0.615, WEIGHT_N)), "{:?}", f_d);
    }

    #[test]
    fn a_pass_without_a_new_pose_changes_nothing() {
        // after the two passes above (f_d.y = 0.615), the simulator stalls: first the same pose comes
        // again (same time stamp), then nothing at all. No simulated time has passed: same force
        let (mut stage, telemetry)= velocity();
        let v_d= Some(Vec3::new(0.0, 0.5, 0.0));
        measure(&telemetry, 10.0, Vec3::new(0.0, 0.0, 0.0));
        force(&mut stage, mission(v_d));
        measure(&telemetry, 10.1, Vec3::new(0.0, 0.0, 0.0));
        force(&mut stage, mission(v_d));
        measure(&telemetry, 10.1, Vec3::new(0.0, 0.0, 0.0));
        assert!(close(force(&mut stage, mission(v_d)), Vec3::new(0.0, 0.615, WEIGHT_N)));
        assert!(close(force(&mut stage, mission(v_d)), Vec3::new(0.0, 0.615, WEIGHT_N)));
    }

    #[test]
    fn when_the_desired_velocity_disappears_the_loop_starts_again_from_zero() {
        // 1. the loop has integrated (f_d.y = 0.615 after two poses)
        let (mut stage, telemetry)= velocity();
        let v_d= Some(Vec3::new(0.0, 0.5, 0.0));
        measure(&telemetry, 10.0, Vec3::new(0.0, 0.0, 0.0));
        force(&mut stage, mission(v_d));
        measure(&telemetry, 10.1, Vec3::new(0.0, 0.0, 0.0));
        force(&mut stage, mission(v_d));
        // 2. the mission stops giving v_d: only the weight
        assert!(close(force(&mut stage, mission(None)), Vec3::new(0.0, 0.0, WEIGHT_N)));
        // 3. v_d comes back, with no new pose: the integral was reset, only the P term remains
        assert!(close(force(&mut stage, mission(v_d)), Vec3::new(0.0, 0.6, WEIGHT_N)));
    }
}
