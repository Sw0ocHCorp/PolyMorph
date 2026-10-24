//! Force -> attitude resolver: the stage between the velocity stage and the attitude stage. It
//! answers "which attitude must the vehicle take so that the desired force becomes producible?",
//! for any vehicle, from its motor tree. Contract and theory:
//! `docs/docs/pipelines/motor-control/resolver.mdx`.
//!
//! The setpoint `Pose` comes down the pipe. This stage READS `force` (f_d, world, N, filled by the
//! velocity stage) and `orientation` (the attitude preference q_pref, filled by the mission), and
//! FILLS `orientation` (q_d) and `imu_measurement.a_velocity` (omega_d, body, rad/s) for the attitude
//! stage. `force` goes on unchanged: the attitude stage rotates it into the body frame.
//!
//! The law, in short: the thrusters can only push inside a subspace S of the body frame (the
//! reachable force subspace, derived by the shared `MotorTree`); its dimension k says how many
//! rotational degrees of freedom the force constraint consumes. The resolver turns the PREFERENCE by
//! the smallest rotation that makes f_d producible, and keeps everything else the preference asked
//! for.
//!
//! Scope (brick 4, "aerial first"): S derived from the tree, alignment for k = 1, 2, 3, guards.
//! Not yet: the restriction to the authority axes and the forward / reverse choice of bidirectional
//! thrusters (ground vehicles), the tilt limit (with the velocity loop). Known limit: S only looks
//! at forces, not at the moment balance (see the documentation: the OSPRAI cannot hold a pitch).

use std::{f64::consts::FRAC_PI_2, sync::{Arc, OnceLock}, time::Duration};

use nalgebra::{Unit, UnitQuaternion, Vector3};
use tokio::sync::broadcast::{Receiver, Sender};

use crate::{control::motion::{motion_controller::{GRAVITY, MotionController, VehicleKinematicConfig}, motor_tree::{ForceSubspace, SharedMotorTree}}, core::scheduler::Process, messages::{pose_messages::Pose, registered_message::{AnyMessage, UnitQuat, Vec3}}};

/// Below this fraction of the weight, the desired force has no usable direction: the last q_d is
/// held. Free fall is never a valid demand for an aerial vehicle.
const MIN_FORCE_FRACTION: f64= 1e-3;

/// q_d, the attitude closest to the preference q_pref that makes f_d producible:
///   1. f_hat = f_d / |f_d| (world), v_w = q_pref v (world);
///   2. gamma = angle between v_w and f_hat, gamma* = 0 (k = 1: align) or pi/2 (k = 2: make them
///      perpendicular);
///   3. q_align = Rot(u, gamma - gamma*), u = (v_w x f_hat) / |v_w x f_hat| (turns v_w TOWARD f_hat
///      when positive);
///   4. q_d = q_align (x) q_pref: a world-frame rotation applied AFTER the preference.
/// Always from q_pref, never from the current attitude: the free degrees of freedom stay exactly
/// those of the preference (no drift). `f_d` must be finite and non-zero (the caller guards it).
pub fn align(q_pref: &UnitQuaternion<f64>, f_d: &Vector3<f64>, subspace: &ForceSubspace) -> UnitQuaternion<f64> {
    // Step 0. the characteristic vector v and its target angle gamma* with f_d: 0 for a line (align
    //         them), pi/2 for a plane (make them perpendicular). All of space (or nothing
    //         reachable): every force is producible, the preference stands
    let (v, target)= match subspace {
        ForceSubspace::Line(direction) => (direction, 0.0),
        ForceSubspace::Plane(normal) => (normal, FRAC_PI_2),
        ForceSubspace::Empty | ForceSubspace::Space => return *q_pref,
    };
    // Step 1. the desired direction, and the characteristic vector of the PREFERRED attitude
    let f_hat= f_d.normalize();
    let v_w= q_pref * v;
    // Step 2. their angle, with atan2: acos has an infinite slope at 0 and pi
    let cross= v_w.cross(&f_hat);
    let sin_gamma= cross.norm();
    let gamma= sin_gamma.atan2(v_w.dot(&f_hat));
    // Step 3. the rotation axis, perpendicular to both. When they are parallel any axis
    //         perpendicular to f_hat works (gamma - gamma* is then 0, pi or +/- pi/2): take the
    //         preference's x axis, projected (its y axis if x lies along f_hat)
    let u= if sin_gamma > 1e-12 { cross / sin_gamma } else { perpendicular_axis(q_pref, &f_hat) };
    let q_align= UnitQuaternion::from_axis_angle(&Unit::new_unchecked(u), gamma - target);
    // Step 4. the alignment, applied after the preference (both are world-frame rotations)
    return q_align * q_pref;
}

/// A unit axis perpendicular to `f_hat`: the preference's x axis, projected onto the plane
/// perpendicular to f_hat; its y axis if x lies along f_hat (x and y cannot both).
fn perpendicular_axis(q_pref: &UnitQuaternion<f64>, f_hat: &Vector3<f64>) -> Vector3<f64> {
    let project= |w: Vector3<f64>| w - f_hat * w.dot(f_hat);
    let x= project(q_pref * Vector3::x());
    return if x.norm() > 1e-6 { x } else { project(q_pref * Vector3::y()) }.normalize();
}

/// The resolver. Its only state is the vehicle (the shared motor tree, hence S) and the last valid
/// q_d.
pub struct ForceToAttitudeResolver {
    name: String,
    period: Duration,
    vehicle_config: VehicleKinematicConfig,
    /// The motor tree of the vehicle, the same instance as the mixer's; empty until it is configured.
    tree: SharedMotorTree,
    /// The last valid q_d, held when the desired force has no usable direction.
    last_q_d: Option<UnitQuaternion<f64>>,
}

impl ForceToAttitudeResolver {
    /// A resolver for a given vehicle. The motor tree is not passed here: it is configured later
    /// (`set_motor_tree`), once the hardware / simulator has described it.
    pub fn new(name: String, vehicle_config: VehicleKinematicConfig) -> Self {
        return Self { name, period: Duration::from_millis(0), vehicle_config, tree: Arc::new(OnceLock::new()), last_q_d: None };
    }

    /// The vehicle's motor tree (shared with the mixer): S is read from it.
    pub fn set_motor_tree(&mut self, tree: SharedMotorTree) {
        self.tree= tree;
    }
}

impl Process for ForceToAttitudeResolver {
    fn set_name(&mut self, name: String) {
        self.name= name;
    }

    /// One cycle: the law on the setpoint of THIS pass (pipe input, never held: the velocity stage
    /// runs in every pass the resolver runs), its result handed to the next process through the pipe
    /// (or `None`: abstain, the attitude stage holds its last setpoint).
    fn exec(&mut self, input: &Option<AnyMessage>, dt: Duration) -> Option<AnyMessage> {
        return self.compute_command_law(None, input.clone(), dt, false);
    }

    /// Not used yet: the current attitude q only enters the forward / reverse choice of
    /// bidirectional thrusters (ground vehicles, not implemented).
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
    /// The law:
    ///   1. the setpoint `Pose` of this pass (`setpoint`); none: abstain [P3]
    ///   2. it must carry f_d (`force`) and q_pref (`orientation`), and S must be known (the motor
    ///      tree is configured, with at least one thruster); otherwise abstain [P3]
    ///   3. f_d without a usable direction (|f_d| < MIN_FORCE_FRACTION m g, or not finite): hold the
    ///      last valid q_d (abstain if there is none yet)
    ///   4. otherwise q_d = align(q_pref, f_d, S), kept as the last valid one
    ///   5. the setpoint goes on with `orientation` = q_d, omega_d = 0 in `imu_measurement`, and
    ///      `force` unchanged
    fn compute_command_law(&mut self, _input_data: Option<AnyMessage>, setpoint: Option<AnyMessage>, _dt: Duration, _verbose: bool) -> Option<AnyMessage> {
        // Step 1. the setpoint of this pass; nothing came down the pipe: abstain
        let Some(AnyMessage::PoseState(mut setpoint))= setpoint else {
            return None;
        };
        // Step 2. what the resolver reads, and the vehicle it resolves for
        let (Some(force), Some(orientation))= (setpoint.force, setpoint.orientation) else {
            return None;
        };
        let Some(subspace)= self.tree.get().map(|tree| tree.force_subspace()).filter(|s| **s != ForceSubspace::Empty) else {
            return None;
        };
        let f_d= Vector3::from(force);
        let q_pref= UnitQuaternion::from(orientation);
        // Step 3. a force with no usable direction: the last valid attitude is held
        let q_d= if !f_d.iter().all(|c| c.is_finite()) || f_d.norm() < MIN_FORCE_FRACTION * self.vehicle_config.weight * GRAVITY {
            self.last_q_d?
        } else {
            // Step 4. the smallest rotation of the preference that makes f_d producible
            let q_d= align(&q_pref, &f_d, subspace);
            self.last_q_d= Some(q_d);
            q_d
        };
        // Step 5. this stage's contribution to the setpoint; the force goes on unchanged
        setpoint.orientation= Some(UnitQuat::from(q_d));
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

/// Hand-predicted values, from the validation tables of `resolver.mdx`. Vehicles (`test_trees`): a
/// quadrotor (4 thrusters along z: k = 1, v = z), the OSPRAI (2 thrusters along z, each under a joint
/// about y: k = 2, v = y) and a holonomic vehicle (thrusters along x, y and z: k = 3). How S is
/// derived from each tree is tested with the tree (`motor_tree.rs`).
#[cfg(test)]
mod tests {
    use nalgebra::Matrix3;

    use crate::control::motion::motor_tree::test_trees::{holonomic, osprai, quadrotor};

    use super::*;

    const MG: f64= 1.2 * GRAVITY;

    fn rot(axis: Vector3<f64>, degrees: f64) -> UnitQuaternion<f64> {
        return UnitQuaternion::from_axis_angle(&Unit::new_normalize(axis), degrees.to_radians());
    }

    #[test]
    fn a_multirotor_tilts_its_thrust_onto_the_force() {
        let s= quadrotor().force_subspace().clone();
        // 1. the weight alone, level preference: level
        assert!(align(&UnitQuaternion::identity(), &Vector3::new(0.0, 0.0, MG), &s).angle_to(&UnitQuaternion::identity()) < 1e-12);
        // 2. a 30 deg yaw preference: kept, yaw is free
        let yaw_30= rot(Vector3::z(), 30.0);
        assert!(align(&yaw_30, &Vector3::new(0.0, 0.0, MG), &s).angle_to(&yaw_30) < 1e-12);
        // 3. f_d = (m g tan 20 deg, 0, m g): the thrust (body z) must point along f_d, a +20 deg
        //    rotation about +y (it takes z towards +x)
        let f_d= Vector3::new(MG * 20f64.to_radians().tan(), 0.0, MG);
        let q_d= align(&UnitQuaternion::identity(), &f_d, &s);
        assert!(q_d.angle_to(&rot(Vector3::y(), 20.0)) < 1e-12, "{:?}", q_d.euler_angles());
        // 4. f_d = (0, m g tan 20 deg, m g) with a 90 deg yaw preference: 20 deg of tilt, body z along
        //    f_d, and the heading is still 90 deg (body x, seen from above, points along +y)
        let f_d= Vector3::new(0.0, MG * 20f64.to_radians().tan(), MG);
        let q_d= align(&rot(Vector3::z(), 90.0), &f_d, &s);
        assert!(((q_d * Vector3::z()) - f_d.normalize()).norm() < 1e-12);
        let heading= (q_d * Vector3::x()).xy();
        assert!((heading.y.atan2(heading.x) - 90f64.to_radians()).abs() < 1e-12, "{:?}", heading);
    }

    #[test]
    fn the_osprai_serves_in_plane_forces_and_refuses_the_roll() {
        let s= osprai().force_subspace().clone();
        // 1. a force in the body xz plane: producible as it is (the arms serve f_x), q_d = q_pref
        assert!(align(&UnitQuaternion::identity(), &Vector3::new(2.0, 0.0, MG), &s).angle_to(&UnitQuaternion::identity()) < 1e-12);
        // 2. a force with a y component: a pure roll that brings the xz plane onto it. Body z goes
        //    to (0, sin r, cos r) for a roll of -r about x, so r = atan2(f_y, f_z) and the roll is -r
        let f_d= Vector3::new(0.0, 2.0, MG);
        let q_d= align(&UnitQuaternion::identity(), &f_d, &s);
        assert!(q_d.angle_to(&rot(Vector3::x(), -(2.0f64).atan2(MG).to_degrees())) < 1e-12, "{:?}", q_d.euler_angles());
        // 3. roll_refused: a 10 deg roll preference with the weight alone. With 10 deg of roll the
        //    body xz plane no longer contains the vertical: the roll is undone, q_d is level
        let q_d= align(&rot(Vector3::x(), 10.0), &Vector3::new(0.0, 0.0, MG), &s);
        assert!(q_d.angle_to(&UnitQuaternion::identity()) < 1e-12, "{:?}", q_d.euler_angles());
    }

    #[test]
    fn a_holonomic_vehicle_keeps_its_preference() {
        // every force is producible whatever the attitude: q_d = q_pref
        let s= holonomic().force_subspace().clone();
        let roll_10= rot(Vector3::x(), 10.0);
        assert!(align(&roll_10, &Vector3::new(1.0, -2.0, MG), &s).angle_to(&roll_10) < 1e-12);
    }

    /// A setpoint Pose: preference q_pref and force f_d.
    fn setpoint(q_pref: UnitQuaternion<f64>, force: Option<Vec3>) -> AnyMessage {
        let mut pose= Pose::default();
        pose.orientation= Some(UnitQuat::from(q_pref));
        pose.force= force;
        return AnyMessage::PoseState(pose);
    }

    /// The vehicle description the tests need: only the mass enters the resolver.
    fn vehicle() -> VehicleKinematicConfig {
        return VehicleKinematicConfig { com_relative_location: Vec3::new(0.0, 0.0, 0.0), error_angular_factor: 0.0,
                                        error_linear_factor: 0.0, error_attitude_factor: 0.0, weight: 1.2,
                                        moments_matrix: Matrix3::identity() };
    }

    /// The resolver of the OSPRAI, its tree configured.
    fn osprai_resolver() -> ForceToAttitudeResolver {
        let mut resolver= ForceToAttitudeResolver::new("resolver".to_string(), vehicle());
        resolver.set_motor_tree(Arc::new(OnceLock::from(osprai())));
        return resolver;
    }

    #[test]
    fn the_stage_fills_its_fields_and_holds_without_a_usable_force() {
        let mut resolver= osprai_resolver();
        let dt= Duration::from_millis(10);
        // 1. roll_refused through the stage: q_d level written into `orientation`, omega_d = 0,
        //    the force unchanged
        let f_d= Vec3::new(0.0, 0.0, MG);
        let Some(AnyMessage::PoseState(out))= resolver.exec(&Some(setpoint(rot(Vector3::x(), 10.0), Some(f_d))), dt) else {
            panic!("no setpoint handed over");
        };
        assert!(UnitQuaternion::from(out.orientation.unwrap()).angle_to(&UnitQuaternion::identity()) < 1e-12);
        assert_eq!(out.imu_measurement.and_then(|imu| imu.a_velocity), Some(Vec3::new(0.0, 0.0, 0.0)));
        assert_eq!(out.force, Some(f_d));
        // 2. a force with no direction (zero, then NaN): the last q_d is held, never a NaN
        for bad in [Vec3::new(0.0, 0.0, 0.0), Vec3::new(f64::NAN, 0.0, MG)] {
            let Some(AnyMessage::PoseState(out))= resolver.exec(&Some(setpoint(rot(Vector3::z(), 45.0), Some(bad))), dt) else {
                panic!("nothing handed over");
            };
            assert!(UnitQuaternion::from(out.orientation.unwrap()).angle_to(&UnitQuaternion::identity()) < 1e-12);
        }
    }

    #[test]
    fn the_stage_abstains_without_force_preference_or_tree() {
        let dt= Duration::from_millis(10);
        // 1. no tree yet: nothing to resolve for
        let mut no_tree= ForceToAttitudeResolver::new("resolver".to_string(), vehicle());
        assert!(no_tree.exec(&Some(setpoint(UnitQuaternion::identity(), Some(Vec3::new(0.0, 0.0, MG)))), dt).is_none());
        // 2. a tree, but no force, no preference, nothing in the pipe
        let mut resolver= osprai_resolver();
        assert!(resolver.exec(&Some(setpoint(UnitQuaternion::identity(), None)), dt).is_none());
        let mut no_preference= Pose::default();
        no_preference.force= Some(Vec3::new(0.0, 0.0, MG));
        assert!(resolver.exec(&Some(AnyMessage::PoseState(no_preference)), dt).is_none());
        assert!(resolver.exec(&None, dt).is_none());
        // 3. a zero force before any valid one: no last q_d to hold, abstain
        assert!(resolver.exec(&Some(setpoint(UnitQuaternion::identity(), Some(Vec3::new(0.0, 0.0, 0.0)))), dt).is_none());
    }
}
