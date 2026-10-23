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
//! reachable force subspace, derived from the motor tree); its dimension k says how many rotational
//! degrees of freedom the force constraint consumes. The resolver turns the PREFERENCE by the
//! smallest rotation that makes f_d producible, and keeps everything else the preference asked for.
//!
//! Scope (brick 4, "aerial first"): S derived from the tree, alignment for k = 1, 2, 3, guards.
//! Not yet: the restriction to the authority axes and the forward / reverse choice of bidirectional
//! thrusters (ground vehicles), the tilt limit (with the velocity loop). Known limit: S only looks
//! at forces, not at the moment balance (see the documentation: the OSPRAI cannot hold a pitch).

use std::{collections::HashMap, f64::consts::FRAC_PI_2, time::Duration};

use nalgebra::{Matrix3, Unit, UnitQuaternion, Vector3};
use tokio::sync::broadcast::{Receiver, Sender};

use crate::{control::motion::{motion_controller::{GRAVITY, MotionController, VehicleKinematicConfig}, motor_controller::{MotorController, working_axis_i32_to_vec3}, motor_tree::{motor_transforms, parents_first_order, receive_motors}}, core::scheduler::Process, messages::{motor_messages::WorkingAxis, pose_messages::Pose, registered_message::{AnyMessage, UnitQuat, Vec3}}};

/// Below this fraction of the weight, the desired force has no usable direction: the last q_d is
/// held. Free fall is never a valid demand for an aerial vehicle.
const MIN_FORCE_FRACTION: f64= 1e-3;

/// The reachable force subspace S of the vehicle, in the body frame, by its dimension k, with the
/// unit vector v that characterises it when there is one.
#[derive(Clone, Debug, PartialEq)]
pub enum ForceSubspace {
    /// k = 0: no thruster, nothing is reachable.
    Empty,
    /// k = 1: a line, of direction v (oriented like the thrust).
    Line(Vector3<f64>),
    /// k = 2: a plane, of normal v.
    Plane(Vector3<f64>),
    /// k = 3: all of space, every force is reachable whatever the attitude.
    Space,
}

/// S, derived from the motor tree AT REST (every joint angle at 0), in the body frame:
///   1. resolve the rest transform of every motor (shared tree geometry);
///   2. for each thruster, its rest thrust direction f_hat;
///   3. for each joint among its ancestors, of axis a, the direction it sweeps the thrust towards,
///      a x f_hat (a thruster hanging under a joint covers a plane, not a line);
///   4. S = the span of all these directions: the eigenvectors of their Gram matrix sum(d d^T) whose
///      eigenvalue is not negligible; k = how many there are;
///   5. v = the eigenvector of the largest eigenvalue (k = 1, the direction, oriented like the
///      thrust) or of the smallest one (k = 2, the normal of the plane).
pub fn reachable_force_subspace(motors: &HashMap<u32, MotorController>) -> ForceSubspace {
    // Step 1. every motor's rest transform, parents first, every joint angle at 0. The CoM does not
    //         matter: only directions are used
    let order= parents_first_order(motors);
    let transforms= motor_transforms(motors, &order, Vec3::new(0.0, 0.0, 0.0), |_| 0.0);
    // every direction d found below goes straight into the Gram matrix sum(d d^T), in the order of
    // the tree: the sum, hence the sign of its eigenvectors, is reproducible
    let mut gram= Matrix3::zeros();
    let mut thrust_sum= Vector3::zeros();
    for id in &order {
        let model= motors[id].get_motor_model();
        // Step 2. a thruster (linear working axis) and its rest thrust direction; joints and
        //         undescribed motors are skipped
        if model.working_axis < WorkingAxis::LinearMotionALongX as i32 {
            continue;
        }
        let Some(transform)= transforms.get(id) else {
            continue;
        };
        let f_hat= UnitQuaternion::from(transform.orientation) * working_axis_i32_to_vec3(model.working_axis).into_inner();
        gram += f_hat * f_hat.transpose();
        thrust_sum += f_hat;
        // Step 3. walk up its ancestors, all resolved since the thruster is, up to the vehicle
        //         (parent 0, no transform): each joint sweeps the thrust towards a x f_hat. The joint
        //         axis seen from the body does not depend on the joint's own angle
        let mut parent_id= model.parent_id;
        while let Some(ancestor_transform)= transforms.get(&parent_id) {
            let ancestor_model= motors[&parent_id].get_motor_model();
            if ancestor_model.working_axis >= WorkingAxis::RotatingAroundX as i32 && ancestor_model.working_axis <= WorkingAxis::RotatingAroundZ as i32 {
                let axis= UnitQuaternion::from(ancestor_transform.orientation) * working_axis_i32_to_vec3(ancestor_model.working_axis).into_inner();
                let swept= axis.cross(&f_hat);
                if swept.norm() > 1e-9 {
                    let d= swept.normalize();
                    gram += d * d.transpose();
                }
            }
            parent_id= ancestor_model.parent_id;
        }
    }
    // Step 4. the span: eigen-decomposition of the Gram matrix (symmetric, 3x3). k = how many
    //         eigenvalues are not negligible next to the largest one (none without a thruster: the
    //         matrix is then zero)
    let eigen= gram.symmetric_eigen();
    let (largest, smallest)= (eigen.eigenvalues.imax(), eigen.eigenvalues.imin());
    let rank= eigen.eigenvalues.iter().filter(|lambda| **lambda > 1e-9 * eigen.eigenvalues[largest]).count();
    // Step 5. the characteristic vector
    return match rank {
        0 => ForceSubspace::Empty,
        1 => {
            let direction= eigen.eigenvectors.column(largest).into_owned();
            ForceSubspace::Line(if direction.dot(&thrust_sum) < 0.0 { -direction } else { direction })
        }
        2 => ForceSubspace::Plane(eigen.eigenvectors.column(smallest).into_owned()),
        _ => ForceSubspace::Space,
    };
}

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

/// The resolver. Its only state is the vehicle (the motor tree, hence S) and the last valid q_d.
pub struct ForceToAttitudeResolver {
    name: String,
    period: Duration,
    vehicle_config: VehicleKinematicConfig,
    /// The motor tree, received once on the configuration channel (like the mixer).
    motor_config_receiver: Option<Receiver<MotorController>>,
    motors: HashMap<u32, MotorController>,
    /// S, recomputed whenever a motor description arrives.
    subspace: ForceSubspace,
    /// The last valid q_d, held when the desired force has no usable direction.
    last_q_d: Option<UnitQuaternion<f64>>,
}

impl ForceToAttitudeResolver {
    pub fn new(name: String, vehicle_config: VehicleKinematicConfig) -> Self {
        return Self { name, period: Duration::from_millis(0), vehicle_config, motor_config_receiver: None,
                      motors: HashMap::new(), subspace: ForceSubspace::Empty, last_q_d: None };
    }

    /// The motor configuration channel (the same one the mixer reads): the tree S is derived from.
    pub fn set_motor_config_receiver(&mut self, receiver: Receiver<MotorController>) {
        self.motor_config_receiver= Some(receiver);
    }
}

impl Process for ForceToAttitudeResolver {
    fn set_name(&mut self, name: String) {
        self.name= name;
    }

    /// One cycle:
    ///   1. absorb new motor descriptions, and derive S again if any arrived;
    ///   2. run the law on the setpoint of THIS pass (pipe input, never held: the velocity stage
    ///      runs in every pass the resolver runs);
    ///   3. hand the result to the next process through the pipe (or `None`: abstain, the attitude
    ///      stage holds its last setpoint).
    fn exec(&mut self, input: &Option<AnyMessage>, dt: Duration) -> Option<AnyMessage> {
        // 1. the motor tree, and S again whenever a description arrived
        if let Some(receiver)= &mut self.motor_config_receiver && receive_motors(receiver, &mut self.motors) {
            self.subspace= reachable_force_subspace(&self.motors);
        }
        // 2. and 3. the law, its result into the pipe
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
    ///      tree arrived); otherwise abstain [P3]
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
        if self.subspace == ForceSubspace::Empty {
            return None;
        }
        let f_d= Vector3::from(force);
        let q_pref= UnitQuaternion::from(orientation);
        // Step 3. a force with no usable direction: the last valid attitude is held
        let q_d= if !f_d.iter().all(|c| c.is_finite()) || f_d.norm() < MIN_FORCE_FRACTION * self.vehicle_config.weight * GRAVITY {
            self.last_q_d?
        } else {
            // Step 4. the smallest rotation of the preference that makes f_d producible
            let q_d= align(&q_pref, &f_d, &self.subspace);
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

/// Hand-predicted values, from the validation tables of `resolver.mdx`. Vehicles: a quadrotor
/// (4 thrusters along z: k = 1, v = z), the OSPRAI (2 thrusters along z, each under a joint about y:
/// k = 2, v = y) and a holonomic vehicle (thrusters along x, y and z: k = 3).
#[cfg(test)]
mod tests {
    use tokio::sync::broadcast::channel;

    use crate::messages::motor_messages::{MotorFeedBack, MotorModel};

    use super::*;

    const MG: f64= 1.2 * GRAVITY;

    /// One motor of a test tree: id, parent (0 = the vehicle), working axis, position in its parent.
    fn motor(id: u32, parent_id: u32, axis: WorkingAxis, location: Vec3) -> MotorController {
        let model= MotorModel { id, parent_id, working_axis: axis as i32, relative_location: Some(location),
                                relative_orientation: Some(UnitQuat::from(UnitQuaternion::identity())), ..Default::default() };
        return MotorController::new(model, MotorFeedBack::default());
    }

    fn tree(motors: Vec<MotorController>) -> HashMap<u32, MotorController> {
        return motors.into_iter().map(|m| (m.get_motor_model().id, m)).collect();
    }

    fn quadrotor() -> HashMap<u32, MotorController> {
        return tree(vec![motor(1, 0, WorkingAxis::LinearMotionALongZ, Vec3::new(0.2, 0.2, 0.0)),
                         motor(2, 0, WorkingAxis::LinearMotionALongZ, Vec3::new(-0.2, 0.2, 0.0)),
                         motor(3, 0, WorkingAxis::LinearMotionALongZ, Vec3::new(-0.2, -0.2, 0.0)),
                         motor(4, 0, WorkingAxis::LinearMotionALongZ, Vec3::new(0.2, -0.2, 0.0))]);
    }

    /// The OSPRAI as its bridge describes it: two arms tilting about y, each carrying a rotor whose
    /// position is given relative to its arm (fix #9).
    fn osprai() -> HashMap<u32, MotorController> {
        return tree(vec![motor(1, 0, WorkingAxis::RotatingAroundY, Vec3::new(0.0, 0.18, 0.10)),
                         motor(2, 0, WorkingAxis::RotatingAroundY, Vec3::new(0.0, -0.18, 0.10)),
                         motor(3, 1, WorkingAxis::LinearMotionALongZ, Vec3::new(0.0, 0.12, 0.0)),
                         motor(4, 2, WorkingAxis::LinearMotionALongZ, Vec3::new(0.0, -0.12, 0.0))]);
    }

    fn holonomic() -> HashMap<u32, MotorController> {
        return tree(vec![motor(1, 0, WorkingAxis::LinearMotionALongX, Vec3::new(0.0, 0.0, 0.0)),
                         motor(2, 0, WorkingAxis::LinearMotionALongY, Vec3::new(0.0, 0.0, 0.0)),
                         motor(3, 0, WorkingAxis::LinearMotionALongZ, Vec3::new(0.0, 0.0, 0.0))]);
    }

    fn rot(axis: Vector3<f64>, degrees: f64) -> UnitQuaternion<f64> {
        return UnitQuaternion::from_axis_angle(&Unit::new_normalize(axis), degrees.to_radians());
    }

    #[test]
    fn the_subspace_comes_from_the_motor_tree() {
        // 1. quadrotor: four thrusts along z, nothing tilts them: a line, v = +z
        let ForceSubspace::Line(v)= reachable_force_subspace(&quadrotor()) else { panic!("not a line") };
        assert!((v - Vector3::z()).norm() < 1e-12, "{:?}", v);
        // 2. OSPRAI: thrusts along z, swept about y by the arms: the xz plane, normal +/- y
        let ForceSubspace::Plane(n)= reachable_force_subspace(&osprai()) else { panic!("not a plane") };
        assert!((n.dot(&Vector3::y()).abs() - 1.0).abs() < 1e-12, "{:?}", n);
        // 3. holonomic: all of space, no characteristic vector
        assert_eq!(reachable_force_subspace(&holonomic()), ForceSubspace::Space);
        // 4. no motor: nothing reachable
        assert_eq!(reachable_force_subspace(&HashMap::new()), ForceSubspace::Empty);
    }

    #[test]
    fn a_multirotor_tilts_its_thrust_onto_the_force() {
        let s= reachable_force_subspace(&quadrotor());
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
        let s= reachable_force_subspace(&osprai());
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
        let s= reachable_force_subspace(&holonomic());
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

    /// The resolver of the OSPRAI, its tree received on the configuration channel.
    fn osprai_resolver() -> ForceToAttitudeResolver {
        let mut resolver= ForceToAttitudeResolver::new("resolver".to_string(), vehicle());
        let (config, receiver)= channel(8);
        resolver.set_motor_config_receiver(receiver);
        for (_, m) in osprai() {
            assert!(config.send(m).is_ok());
        }
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
        assert!(matches!(resolver.subspace, ForceSubspace::Plane(_)));
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
