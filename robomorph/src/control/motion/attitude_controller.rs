//! Attitude controller: geometric PD on the quaternion error + gravity feedforward
//! (stabilize mode). Theory and validation: `docs/docs/pipelines/motor-control/attitude-controller.mdx`.
//!
//! Notation (also in `docs/docs/glossary.mdx`): q attitude body->world, omega gyro (body, rad/s),
//! q_d / omega_d setpoints, e_R rotation-vector error (rad), tau / zeta tunings, alpha desired angular
//! acceleration (rad/s^2), I inertia matrix, M moments (N.m), m mass, g gravity.

use std::{ops::Neg, time::Duration};

use nalgebra::{UnitQuaternion, Vector3};
use tokio::sync::broadcast::{Receiver, Sender};

use crate::{control::{motion::motion_controller::{MotionController, VehicleKinematicConfig}, pid_controller::PIDController}, core::scheduler::Process, messages::{motor_messages::WorkVec, pose_messages::Pose, registered_message::{AnyMessage, UnitQuat, Vec3}}};



/// The attitude stage of the cascade: holds a desired attitude, and turns the desired force into
/// body coordinates for the mixer.
///
/// Measurements `q` (attitude, body->world) and `omega` (gyro, body) come from one `Pose` message
/// (telemetry channel). The setpoint is the `Pose` that comes down the scheduler pipe, its single
/// source; this stage READS its `orientation` (q_d), `imu_measurement.a_velocity` (omega_d, body)
/// and `force` (f_d, world). Any process may come before it, as long as it fills these three
/// fields (today the force -> attitude resolver). Output: an ABSOLUTE wrench `[q^-1 f_d ; M]`
/// handed to the mixer, or `None`. The weight compensation is part of f_d (velocity stage): this
/// stage does not compute it.
/// Only two tunings: `time_constant` (tau, s) and `damping_factor` (zeta, 1 by default) - both
/// vehicle-independent because the mass and inertia are applied at the very end of the law.
pub struct AttitudeController {
    name: String,
    vehicle_config: VehicleKinematicConfig,
    telemetry_receiver: Option<Receiver<AnyMessage>>,
    wrench_sender: Option<Sender<AnyMessage>>,
    period: Duration,
    /// tau (s): closed-loop time constant asked for - ~63 % of a step recovered in tau. Bounded
    /// below by ~10x the loop period and by ~3-5x the rotor time constant. kp = 1/tau^2.
    time_constant: f64,
    /// zeta: damping ratio of the closed loop, 1 = critical (fastest without overshoot). kd = 2*zeta/tau.
    damping_factor: f64,
    /// Last setpoint received, HELD between two updates (zero-order hold): the stage above may run
    /// slower than this one, or abstain. The measurement, on the contrary, is never held for computing.
    setpoint: Pose,

}

impl AttitudeController {
    /// `time_constant` = tau (s), `damping_factor` = zeta. The initial setpoint is
    /// `Pose::default()`, every field `None`: the law abstains until a real setpoint arrives.
    pub fn new(name: String, veh_config: VehicleKinematicConfig, time_constant: f64, damping_factor: f64) -> Self {
        return Self{name, 
                    vehicle_config: veh_config,
                    telemetry_receiver: None, 
                    wrench_sender: None,
                    period: Duration::from_millis(0),
                    time_constant: time_constant,
                    damping_factor,
                    setpoint: Pose::default()
                }
    }

    pub fn get_time_constant(&self) -> f64 {
        return self.time_constant;
    }
}

impl Process for AttitudeController {
    fn set_name(&mut self, name: String) {
        self.name= name;
    }

    /// One cycle: read the freshest pose of this tick (never held: a loop computing on a stale
    /// measurement is servoing at the measurement rate, and the D term on a stale gyro is a fake
    /// derivative), update the held setpoint if the resolver handed a new one through the pipe, run
    /// the law. The wrench goes to `wrench_sender` when someone listens, else into the pipe.
    /// When the law cannot compute, nothing is published: the mixer keeps its last wrench.
    fn exec(&mut self, input: &Option<AnyMessage>, dt: std::time::Duration) -> Option<AnyMessage> {
        // Fresh measurement of THIS tick. If several arrived, the last one wins; if none arrived the
        // pose stays default (orientation None) and the law abstains below.
        let mut current_pose= Pose::default();
        //get the current vehicle pose from the pose asynchronous receiver
        if let Some(current_pose_rcvr)= &mut self.telemetry_receiver {
            let sz= current_pose_rcvr.len();
            for _ in 0..sz {
                if let Ok(pose_msg)= current_pose_rcvr.try_recv() && 
                        let AnyMessage::PoseState(pose) = pose_msg {
                    current_pose= pose;
                }
            }
        }
        // The setpoint of THIS pass, from the process registered right before this stage (pipe, its
        // single source). Nothing in the pipe (that process abstained): the previous one is held.
        if let Some(AnyMessage::PoseState(setpoint))= input {
            self.setpoint= setpoint.clone();
        }
        let wrench_setpoint= self.compute_command_law(Some(AnyMessage::PoseState(current_pose)), 
                                                            Some(AnyMessage::PoseState(self.setpoint.clone())), dt, false);
        if let Some(setpoint)= wrench_setpoint {
            if let Some(wrench_sender)= &mut self.wrench_sender && 
                        wrench_sender.receiver_count() > 0{
                let _= wrench_sender.send(setpoint);
                return None;
            } else {
                return Some(setpoint);
            }
        } else {
            return None;
        }
                
    }

    fn set_receiver(&mut self, receiver: tokio::sync::broadcast::Receiver<AnyMessage>) {
        self.telemetry_receiver= Some(receiver);
    }

    fn set_sender(&mut self, sender: tokio::sync::broadcast::Sender<AnyMessage>) {
        self.wrench_sender= Some(sender);
    }

    fn set_period_from_freq(&mut self, frequency: u64) {
        self.period= Duration::from_nanos(1_000_000_000 / frequency);
    }

    fn get_period(&self) -> std::time::Duration {
        return self.period.clone();
    }

    fn get_name(&self) ->String {
        return self.name.clone();
    }
}

impl MotionController for AttitudeController {
    /// The attitude law, stateless (same inputs -> same outputs):
    ///   1. q, q_d -> UnitQuaternion (renormalises the wire quaternion; mandatory before inverting)
    ///   2. q_err = q^-1 (x) q_d                 error expressed in the BODY frame
    ///   3. if q_err.w < 0: q_err = -q_err       shortest path (double cover of the rotation group)
    ///   4. theta = 2 atan2(|v|, w); e_R = (theta/|v|) v, or 2v below eps    log map -> rotation vector
    ///   5. alpha = (1/tau^2) e_R + (2 zeta/tau) (omega_d - omega)          rad/s^2, P on angle + P on gyro
    ///   6. M_k = I_k alpha_k                                                N.m, per axis
    ///   7. f_body = q^-1 f_d                                                the setpoint force, body
    ///   8. return [f_body ; M]
    /// `setpoint` is an `AnyMessage::PoseState`: q_d = `orientation`, omega_d =
    /// `imu_measurement.a_velocity`, f_d = `force`. Returns `None` when any required field is missing
    /// (measurement q or omega; setpoint q_d, omega_d or f_d): ABSTAIN, never a zero wrench [P3]. `dt` is not needed by this law (no integrator, no numerical derivative).
    fn compute_command_law(&mut self, input_data: Option<AnyMessage>, setpoint: Option<AnyMessage>, dt: std::time::Duration, verbose: bool) -> Option<AnyMessage> {
        //compute the command law ONLY if the attitude controller receive the current pose
        //and the setpoint to reach
        //but also IF the input and setpoint contains the required data struct
        //  current quaternion orientation, current imu measurements(for gyro) and the setpoint quaternion orientation
        if let Some(pose_state) = input_data && let AnyMessage::PoseState(current_pose)= pose_state {
            if verbose {
                println!("Input pose:\n{:?}", current_pose);
            }
            if let Some(setpoint_data) = setpoint && let AnyMessage::PoseState(pose_setpoint)= setpoint_data {
                if verbose {
                    println!("Setpoint pose:\n{:?}", pose_setpoint);
                }
                if let Some(current_orientation)= current_pose.orientation && 
                        let Some(current_imu)= current_pose.imu_measurement && let Some(current_angular_vel)= current_imu.a_velocity &&
                            let Some(orientation_setpoint)= pose_setpoint.orientation &&
                                let Some(angular_vel_setpoint)= pose_setpoint.imu_measurement.and_then(|imu| imu.a_velocity) &&
                                    let Some(force_setpoint)= pose_setpoint.force {
                    //apply inverse on current orientation because current orientation if the orientation of the vehicle in the world frame
                    //so body frame => world frame
                    //and the inverse represent orientation in body frame
                    //so world frame => body frame
                    //the attitude controller will send a setpoint wrench relative to the current wrench of the vehicle
                    //so the orientation needs to be express in the body frame
                    // Step 2. q^-1 on the LEFT: the error is expressed in the current body frame - the frame
                    // of the gyro and of the mixer's moments. (q_d * q^-1 would express it in the world.)
                    let mut error_body= UnitQuaternion::from(current_orientation).inverse() * UnitQuaternion::from(orientation_setpoint);
                    // Step 3. q and -q are the same rotation; w < 0 means the encoded angle exceeds 180 deg
                    // (the long way round). Taking the other representative keeps theta in [0, pi].
                    if error_body.w < 0.0 {
                        error_body= UnitQuaternion::from(UnitQuat{w: -error_body.w, x: -error_body.i, 
                                                                        y: -error_body.j, z: -error_body.k});
                    }
                    let error_body_vec= error_body.vector();
                    if verbose {
                        println!("Attitude Error in body frame: {:?}", error_body);
                        println!("Attitude Error in body frame (vector part): {:?}", error_body_vec.into_owned());
                    }
                    // Step 4. theta = 2 atan2(|v|, w): (w, |v|) = (cos(theta/2), sin(theta/2)) is a point on the unit
                    // circle. atan2 rather than acos(w): acos has an infinite slope near 1, i.e. at equilibrium.
                    let remaining_rotation= 2.0*f64::atan2(error_body_vec.norm(), error_body.w);
                    // e_R = theta * e, the rotation vector (rad), in the BODY frame despite the variable name.
                    // NOTE: the division by |v| is evaluated before the guard below (0/0 = NaN at equilibrium);
                    // that is harmless ONLY because the guard tests |v| itself, never the (possibly NaN) result.
                    let mut error_world= (remaining_rotation / error_body_vec.norm()) * error_body_vec;
                    // Small-angle branch: v = e sin(theta/2) ~ e theta/2, so theta e ~ 2 v. Not optional: the
                    // controller lives at equilibrium, where |v| -> 0. The threshold value is not sensitive.
                    let eps= 1e-6;
                    if error_body_vec.norm() <= eps {
                        error_world= 2.0 * error_body_vec;
                    }
                    // Step 5. Two proportional feedbacks on the two components of the rotational state
                    // (angle error, angular-rate error). The rate error uses the MEASURED gyro: no numerical
                    // derivative anywhere. omega_d comes from the resolver (zero while it does not
                    // differentiate q_d). Gains identical on the three axes.
                    let angular_accel= 1.0 / f64::powf(self.time_constant, 2.0) * error_world + 
                                                                            (2.0*self.damping_factor / self.time_constant) * (Vector3::from(angular_vel_setpoint) - Vector3::from(current_angular_vel));
                    // Step 7. The force to PRODUCE, f_d, given in the WORLD frame (it includes the weight
                    // compensation, (0, 0, +m g) in stabilize), translated into body coordinates by q^-1.
                    // When the vehicle is tilted the part of it the vehicle cannot produce (the lateral
                    // component, for an under-actuated vehicle) stays in the mixer residual: expected, not
                    // to be "fixed" here.
                    let force_vec= UnitQuaternion::from(current_orientation).inverse() * Vector3::from(force_setpoint);
                    // Step 6. M = I * alpha: the inertia matrix (diagonal in practice) turns the desired angular
                    // acceleration into moments. This is where the vehicle enters the law, and why (tau, zeta)
                    // are portable between vehicles.
                    let result_wrench= WorkVec::new_from_vec3(Vec3::from(force_vec), 
                                                                        Vec3::from(self.vehicle_config.moments_matrix * angular_accel));
                    if verbose {
                        println!("[AttitudeController:INFO] -> Resulting Vehicle Wrench:\n{:?}", result_wrench);
                    }
                    return Some(AnyMessage::VehicleWrench(result_wrench));
                }
            } 
        }
        // Missing measurement or setpoint: abstain. A zero wrench here would be an active
        // "cut everything" command, executed faithfully by the mixer (it once made the rotors
        // alternate between hover thrust and minimum at the loop rate).
        // verbose only: at start-up (no pose yet, Gazebo paused) this would print at every pass
        if verbose {
            println!("[AttitudeController:WARNING] -> Wrong data passed to Attitude controller");
        }
        return None;
    }

    /// Not used: the setpoint comes from the resolver through the pipe, and only from there. A second
    /// source would overwrite it at every tick.
    fn set_setpoint_receiver(&mut self, _receiver: Receiver<Pose>) {
        println!("[WARNING::AttitudeController] -> the setpoint comes from the resolver through the pipe: receiver ignored");
    }
}

/// Hand-predicted values for the force path and the rate setpoint of the law. Vehicle: m = 1.2 kg,
/// I = diag(0.017, 0.0239, 0.0357) kg.m^2, tau = 0.2 s, zeta = 1.
#[cfg(test)]
mod tests {
    use nalgebra::{Matrix3, UnitQuaternion, Vector3};

    use crate::messages::{pose_messages::IMUMeasurements, registered_message::UnitQuat};

    use super::*;

    /// The controller of the tests.
    fn controller() -> AttitudeController {
        let vehicle_config= VehicleKinematicConfig {
            com_relative_location: Vec3::new(0.0, 0.0, 0.0),
            error_angular_factor: 0.0,
            error_linear_factor: 0.0,
            error_attitude_factor: 0.0,
            weight: 1.2,
            moments_matrix: Matrix3::from_diagonal(&Vector3::new(0.017, 0.0239, 0.0357)),
        };
        return AttitudeController::new("attitude".to_string(), vehicle_config, 0.2, 1.0);
    }

    /// A pose: attitude q and angular rate omega; and, for a setpoint, the force f_d (world).
    fn pose(q: UnitQuaternion<f64>, omega: Vec3, force: Option<Vec3>) -> AnyMessage {
        let mut pose= Pose::default();
        pose.orientation= Some(UnitQuat::from(q));
        pose.imu_measurement= Some(IMUMeasurements { timestamp: None, l_accel: None, a_velocity: Some(omega), magnetic_field: None });
        pose.force= force;
        return AnyMessage::PoseState(pose);
    }

    /// The wrench of one law evaluation; a missing wrench fails the test.
    fn wrench(measured: AnyMessage, setpoint: AnyMessage) -> WorkVec {
        let Some(AnyMessage::VehicleWrench(wrench))= controller().compute_command_law(Some(measured), Some(setpoint), Duration::from_millis(10), false) else {
            panic!("no wrench produced");
        };
        return wrench;
    }

    #[test]
    fn the_force_is_the_setpoint_force_seen_from_the_body() {
        // 1. yawed by 90 deg about z, on target (q = q_d, no rate): no moment at all
        let yawed= UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 90f64.to_radians());
        let zero= Vec3::new(0.0, 0.0, 0.0);
        let w= wrench(pose(yawed, zero, None), pose(yawed, zero, Some(Vec3::new(1.0, 0.0, 11.772))));
        // 2. f_body = q^-1 f_d: the world x axis is the body -y axis once yawed by +90 deg, so
        //    (1, 0, 11.772) world reads (0, -1, 11.772) in the body frame
        assert!(w.fx.abs() < 1e-12 && (w.fy + 1.0).abs() < 1e-12 && (w.fz - 11.772).abs() < 1e-12, "{:?}", w);
        assert!(w.mx.abs() < 1e-12 && w.my.abs() < 1e-12 && w.mz.abs() < 1e-12, "{:?}", w);
    }

    #[test]
    fn the_rate_setpoint_enters_the_damping_term() {
        // 1. level, on target, at rest, but asked to roll at omega_d = 0.5 rad/s
        let level= UnitQuaternion::identity();
        let w= wrench(pose(level, Vec3::new(0.0, 0.0, 0.0), None), pose(level, Vec3::new(0.5, 0.0, 0.0), Some(Vec3::new(0.0, 0.0, 11.772))));
        // 2. alpha_x = (2 zeta / tau) (omega_d - omega) = (2 / 0.2) * 0.5 = 5 rad/s^2, so
        //    M_x = I_xx alpha_x = 0.017 * 5 = 0.085 N.m; nothing on the other axes
        assert!((w.mx - 0.085).abs() < 1e-12, "{:?}", w);
        assert!(w.my.abs() < 1e-12 && w.mz.abs() < 1e-12, "{:?}", w);
    }

    #[test]
    fn the_law_abstains_without_a_force() {
        // a setpoint with no force (no velocity stage before): abstain [P3], never a zero force (a
        // "cut the rotors" command)
        let level= UnitQuaternion::identity();
        let zero= Vec3::new(0.0, 0.0, 0.0);
        let result= controller().compute_command_law(Some(pose(level, zero, None)), Some(pose(level, zero, None)), Duration::from_millis(10), false);
        assert!(result.is_none());
    }
}
