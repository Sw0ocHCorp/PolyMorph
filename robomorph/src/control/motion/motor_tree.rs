//! The motor tree of a vehicle: the description of its motors (`MotorModel`), configured ONCE and
//! shared by every stage that needs it (the mixer, the force -> attitude resolver, ...), with what
//! follows from that description alone, derived once when the tree is built:
//!   - the order parents first, and the children of each motor;
//!   - the reachable force subspace S (`force_subspace`), from the tree AT REST;
//! and, on demand, each motor's pose in the body frame for given joint angles (`transforms`).
//!
//! One instance, written once, read by all: `SharedMotorTree` (`Arc<OnceLock<MotorTree>>`), so two
//! stages can never hold two different trees. It is written either by the vehicle controller that
//! discovers the motors (Gazebo), or from a JSON configuration file (`from_json`, the real robot).
//! Until it is, every stage that reads it abstains.
//!
//! A motor is described at rest, in its parent's frame (`MotorModel::relative_location`,
//! `relative_orientation`); a root motor (`parent_id == 0`) hangs directly under the vehicle.
//! Theory: `docs/docs/pipelines/motor-control/motor-model.mdx` and `resolver.mdx`.

use std::{collections::HashMap, path::Path, sync::{Arc, OnceLock}};

use nalgebra::{Matrix3, Unit, UnitQuaternion, Vector3};
use serde::Deserialize;

use crate::messages::{motor_messages::MotorModel, pose_messages::Transform, registered_message::{UnitQuat, Vec3}};

/// The single motor tree of the vehicle, shared: written once (`OnceLock::set`, a second write is
/// refused), read by every stage (`get`, `None` until it is written).
pub type SharedMotorTree= Arc<OnceLock<MotorTree>>;

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

/// The motor tree, fixed once built (no method changes it).
pub struct MotorTree {
    /// The description of every motor, by id.
    models: HashMap<u32, MotorModel>,
    /// The ids, parents first: depth ascending, ties broken by id. Forward = root -> leaves,
    /// reversed = leaves -> root. Never the HashMap order, which is randomised per map: what is
    /// computed by sweeping the tree in this order (the mixer's allocation, S) is reproducible.
    order: Vec<u32>,
    /// The ids mounted on each motor, in `order`. Derived from `parent_id`, the only link stored.
    children: HashMap<u32, Vec<u32>>,
    /// S, derived from the tree at rest.
    force_subspace: ForceSubspace,
}

/// A JSON configuration file: `{ "motors": [ { "id": 74, "working_axis": "RotatingAroundY", ... } ] }`.
#[derive(Deserialize)]
struct MotorTreeFile {
    motors: Vec<MotorModel>,
}

impl MotorTree {
    /// Build the tree from its motors, checked, then derive what follows from it:
    ///   1. index by id: 0 is reserved (it means "the vehicle"), ids are unique, the axis is known,
    ///      the bounds are ordered;
    ///   2. every parent is the vehicle or a known motor;
    ///   3. the order parents first: depth (length of the parent chain up to the vehicle), then id.
    ///      A chain longer than the tree is a cycle;
    ///   4. the children of each motor, in that order;
    ///   5. S, from the tree at rest.
    /// A faulty description stops here, with a message, instead of flying a wrong tree.
    pub fn new(models: Vec<MotorModel>) -> Result<Self, String> {
        // 1. by id
        let mut by_id: HashMap<u32, MotorModel>= HashMap::new();
        for model in models {
            if model.id == 0 {
                return Err("a motor has the id 0, reserved for the vehicle itself (parent_id 0)".to_string());
            }
            if !model.is_joint() && !model.is_thruster() {
                return Err(format!("motor #{}: unknown working axis {}", model.id, model.working_axis));
            }
            if model.min_value > model.max_value {
                return Err(format!("motor #{}: min_value {} above max_value {}", model.id, model.min_value, model.max_value));
            }
            let id= model.id;
            if by_id.insert(id, model).is_some() {
                return Err(format!("the motor id {} is used twice", id));
            }
        }
        // 2. known parents
        for model in by_id.values() {
            if model.parent_id != 0 && !by_id.contains_key(&model.parent_id) {
                return Err(format!("motor #{}: unknown parent #{}", model.id, model.parent_id));
            }
        }
        // 3. parents first; the depth walks up the parent chain, counting the steps
        let mut depths: HashMap<u32, usize>= HashMap::new();
        for &id in by_id.keys() {
            let (mut depth, mut current)= (0, id);
            while by_id[&current].parent_id != 0 {
                current= by_id[&current].parent_id;
                depth += 1;
                if depth > by_id.len() {
                    return Err(format!("motor #{} is part of a parent cycle", id));
                }
            }
            depths.insert(id, depth);
        }
        let mut order: Vec<u32>= by_id.keys().copied().collect();
        order.sort_by_key(|id| (depths[id], *id));
        // 4. children, in that order
        let mut children: HashMap<u32, Vec<u32>>= HashMap::new();
        for id in &order {
            let parent_id= by_id[id].parent_id;
            if parent_id != 0 {
                children.entry(parent_id).or_default().push(*id);
            }
        }
        // 5. S
        let mut tree= Self { models: by_id, order, children, force_subspace: ForceSubspace::Empty };
        tree.force_subspace= tree.reachable_force_subspace();
        return Ok(tree);
    }

    /// The tree described by a JSON configuration file (see `MotorTreeFile`, and `MotorModel` for
    /// the form of each motor), checked like `new`.
    pub fn from_json(path: &Path) -> Result<Self, String> {
        let text= std::fs::read_to_string(path).map_err(|error| format!("{}: {}", path.display(), error))?;
        let file: MotorTreeFile= serde_json::from_str(&text).map_err(|error| format!("{}: {}", path.display(), error))?;
        return Self::new(file.motors);
    }

    /// The description of motor `id`.
    pub fn get(&self, id: u32) -> Option<&MotorModel> {
        return self.models.get(&id);
    }

    /// The number of motors.
    pub fn len(&self) -> usize {
        return self.models.len();
    }

    /// The motor ids, parents first.
    pub fn order(&self) -> &[u32] {
        return &self.order;
    }

    /// The ids of the motors mounted on motor `id` (none for a leaf).
    pub fn children(&self, id: u32) -> &[u32] {
        return self.children.get(&id).map_or(&[], Vec::as_slice);
    }

    /// S, the subspace of body-frame forces the motors can produce.
    pub fn force_subspace(&self) -> &ForceSubspace {
        return &self.force_subspace;
    }

    /// Resolve every motor's absolute transform in the BODY frame, by walking the kinematic chain
    /// parents first: a child's pose is expressed relative to its parent's.
    ///
    /// Notation:      R_j = orientation of motor j in the body frame       (quaternion)
    ///                p_j = position    of motor j in the body frame       (vector)
    ///                R_j^rel, p_j^rel  = pose of j at rest, in its PARENT's frame  (from the model)
    ///                theta_j           = current angle of j, 0 if j is not an angular joint
    ///                e_j               = j's own working axis, in its own frame
    ///
    /// FORMULAS
    ///     orientation:   R_j = R_parent * R_j^rel * Rot(e_j, theta_j)
    ///     position:      p_j = p_parent + R_parent * p_j^rel
    ///
    /// Two points that are easy to get wrong:
    ///   - the joint angle is composed on the RIGHT of R_j^rel, because theta_j is a rotation about
    ///     the joint's OWN axis, expressed in its own frame, not in the parent's.
    ///   - p_j^rel is expressed in the PARENT's frame, so it is rotated by R_parent, never by R_j.
    ///
    /// Note that Rot(e_j, theta_j) leaves e_j itself unchanged, so the joint axis seen in the body
    /// frame is R_parent * R_j^rel * e_j: it does not depend on the joint's own angle. That is why the
    /// mixer and the resolver read a joint axis straight from R_j.
    ///
    /// For a root motor (parent_id == 0) the parent is the vehicle itself:
    ///     R_j = R_j^rel * Rot(e_j, theta_j)          p_j = p_j^rel - p_com
    /// the CoM offset being what makes every lever arm below be taken about the reduction point:
    /// p_j^rel is measured from the vehicle origin, p_com is the CoM measured from that same origin,
    /// so the vector from the CoM to the motor is their difference.
    ///
    /// `joint_angle` gives theta_j of the joint of a given id: the mixer passes the measured angle,
    /// the resolver passes 0 to get the tree AT REST. `com` is the CoM in the vehicle frame (the
    /// reduction point of the positions; directions do not depend on it).
    pub fn transforms(&self, com: Vec3, joint_angle: impl Fn(u32) -> f64) -> HashMap<u32, Transform> {
        let mut motor_transforms: HashMap<u32, Transform>= HashMap::new();
        for motor_id in &self.order {
            let motor_model= &self.models[motor_id];
            // IF the motor is the first motor of the kinematic chain
            if motor_model.parent_id == 0 {
                let mut motor_location= Vec3::default();
                let mut motor_orientation= UnitQuaternion::identity();
                // ROOT MOTOR: the "parent" is the vehicle itself, so R_parent = identity and the
                //             positions are re-based on the CoM, the reduction point every lever
                //             arm is taken about.
                // POSITION         p_j = p_j^rel - p_com
                if let Some(m_location)= motor_model.relative_location {
                    // p_j^rel : position of the root motor, given relative to the vehicle origin
                    motor_location= m_location;
                    // - p_com : p_com is the CoM seen from the origin, so (motor - CoM) is the vector
                    //           from the CoM to the motor: p x f below is then the moment about the
                    //           CoM, not about the vehicle origin. (It was `+= p_com` until bug #11,
                    //           harmless only while p_com was 0.)
                    motor_location -= com;
                }
                // ORIENTATION      R_j = R_j^rel * Rot(e_j, theta_j)
                if let Some(m_orientation)= motor_model.relative_orientation {
                    // R_j^rel : rest orientation of the root motor in the body frame
                    motor_orientation= UnitQuaternion::from(m_orientation);
                }
                // * Rot(e_j, theta_j) : same as below, an angular joint adds its current angle
                if motor_model.is_joint() {
                    motor_orientation *= UnitQuaternion::from_axis_angle(&Unit::new_unchecked(motor_model.axis()), joint_angle(*motor_id));
                }
                motor_transforms.insert(*motor_id, Transform {location: motor_location, orientation: UnitQuat::from(motor_orientation)});
            }
            // ELSE a child: its parent comes before it in `order`, so its transform is resolved
            else {
                let parent_transform= motor_transforms[&motor_model.parent_id].clone();
                let mut motor_transform= parent_transform.clone();
                // ORIENTATION      R_j = R_parent * R_j^rel * Rot(e_j, theta_j)
                if let Some(m_orientation)= motor_model.relative_orientation {
                    // R_j^rel : rest orientation of j, in its parent's frame
                    let mut motor_orientation= UnitQuaternion::from(m_orientation);
                    if motor_model.is_joint() {
                        // R_j^rel * Rot(e_j, theta_j) : an angular joint adds its current angle about its
                        // own axis. Composed on the RIGHT because e_j is expressed in j's frame.
                        motor_orientation *= UnitQuaternion::from_axis_angle(&Unit::new_unchecked(motor_model.axis()), joint_angle(*motor_id));
                    }
                    // R_parent : parent orientation in the body frame, already resolved
                    let parent_orientation= UnitQuaternion::from(parent_transform.orientation);
                    // R_j = R_parent * ( R_j^rel * Rot(e_j, theta_j) )
                    motor_transform.orientation= UnitQuat::from(parent_orientation * motor_orientation);
                }
                // POSITION         p_j = p_parent + R_parent * p_j^rel
                if let Some(m_location)= motor_model.relative_location {
                    // p_j^rel : position of j relative to its parent, in the PARENT's frame
                    let mut motor_location= Vector3::from(m_location);
                    // R_parent : rotates p_j^rel from the parent's frame into the body frame.
                    // The parent's orientation, never the child's: p_j^rel is expressed in the parent.
                    let parent_orientation= UnitQuaternion::from(parent_transform.orientation);
                    // p_parent : parent position in the body frame, already resolved
                    let parent_location=    Vector3::from(parent_transform.location);
                    // p_j = p_parent + R_parent * p_j^rel
                    motor_location= parent_location + parent_orientation * motor_location;
                    motor_transform.location= Vec3::from(motor_location);
                }
                motor_transforms.insert(*motor_id, motor_transform);
            }
        }
        return motor_transforms;
    }

    /// S, derived from the tree AT REST (every joint angle at 0), in the body frame:
    ///   1. resolve the rest transform of every motor;
    ///   2. for each thruster, its rest thrust direction f_hat;
    ///   3. for each joint among its ancestors, of axis a, the direction it sweeps the thrust towards,
    ///      a x f_hat (a thruster hanging under a joint covers a plane, not a line);
    ///   4. S = the span of all these directions: the eigenvectors of their Gram matrix sum(d d^T) whose
    ///      eigenvalue is not negligible; k = how many there are;
    ///   5. v = the eigenvector of the largest eigenvalue (k = 1, the direction, oriented like the
    ///      thrust) or of the smallest one (k = 2, the normal of the plane).
    fn reachable_force_subspace(&self) -> ForceSubspace {
        // Step 1. every motor's rest transform, every joint angle at 0. The CoM does not matter:
        //         only directions are used
        let transforms= self.transforms(Vec3::new(0.0, 0.0, 0.0), |_| 0.0);
        // every direction d found below goes straight into the Gram matrix sum(d d^T), in the order of
        // the tree: the sum, hence the sign of its eigenvectors, is reproducible
        let mut gram= Matrix3::zeros();
        let mut thrust_sum= Vector3::zeros();
        for id in &self.order {
            let model= &self.models[id];
            // Step 2. a thruster and its rest thrust direction; joints are skipped
            if !model.is_thruster() {
                continue;
            }
            let f_hat= UnitQuaternion::from(transforms[id].orientation) * model.axis();
            gram += f_hat * f_hat.transpose();
            thrust_sum += f_hat;
            // Step 3. walk up its ancestors, up to the vehicle (parent 0): each joint sweeps the
            //         thrust towards a x f_hat. The joint axis seen from the body does not depend on
            //         the joint's own angle
            let mut parent_id= model.parent_id;
            while parent_id != 0 {
                let ancestor= &self.models[&parent_id];
                if ancestor.is_joint() {
                    let axis= UnitQuaternion::from(transforms[&parent_id].orientation) * ancestor.axis();
                    let swept= axis.cross(&f_hat);
                    if swept.norm() > 1e-9 {
                        let d= swept.normalize();
                        gram += d * d.transpose();
                    }
                }
                parent_id= ancestor.parent_id;
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
}

/// Trees shared by the tests of the stages that read a `MotorTree` (this module, the mixer, the
/// resolver).
#[cfg(test)]
pub(crate) mod test_trees {
    use nalgebra::UnitQuaternion;

    use crate::messages::{motor_messages::{MotorModel, WorkingAxis}, registered_message::{UnitQuat, Vec3}};

    use super::MotorTree;

    /// One motor: id, parent (0 = the vehicle), working axis, position in its parent's frame, at rest
    /// aligned with its parent; no bounds, no constants.
    pub fn motor(id: u32, parent_id: u32, axis: WorkingAxis, location: Vec3) -> MotorModel {
        return MotorModel { id, parent_id, working_axis: axis as i32, relative_location: Some(location),
                            relative_orientation: Some(UnitQuat::from(UnitQuaternion::identity())), ..Default::default() };
    }

    /// Four thrusters along z, nothing to tilt them: S is a line.
    pub fn quadrotor() -> MotorTree {
        return MotorTree::new(vec![motor(1, 0, WorkingAxis::LinearMotionALongZ, Vec3::new(0.2, 0.2, 0.0)),
                                   motor(2, 0, WorkingAxis::LinearMotionALongZ, Vec3::new(-0.2, 0.2, 0.0)),
                                   motor(3, 0, WorkingAxis::LinearMotionALongZ, Vec3::new(-0.2, -0.2, 0.0)),
                                   motor(4, 0, WorkingAxis::LinearMotionALongZ, Vec3::new(0.2, -0.2, 0.0))]).unwrap();
    }

    /// The OSPRAI as its Gazebo bridge describes it: on each side an arm (joint about y, root),
    /// carrying a rotor (thrust along z) placed relative to its arm (fix #9). Ids, bounds and
    /// constants are the bridge's: arm_left 74, rotor_left 64, arm_right 189, rotor_right 179.
    pub fn osprai() -> MotorTree {
        let mut models= Vec::new();
        for (arm_id, rotor_id, y, k_m) in [(74, 64, 1.0, -0.016), (189, 179, -1.0, 0.016)] {
            models.push(MotorModel { min_value: -1.57, max_value: 1.57, max_rot_speed: 10.0,
                                     ..motor(arm_id, 0, WorkingAxis::RotatingAroundY, Vec3::new(0.0, 0.18 * y, 0.10)) });
            models.push(MotorModel { min_value: 0.1, max_value: 17.28, max_rot_speed: 1200.0, moment_constant: k_m,
                                     effort_constant: 1.2e-5, exp_command_law: 2, transmission_factor: 1.0,
                                     ..motor(rotor_id, arm_id, WorkingAxis::LinearMotionALongZ, Vec3::new(0.0, 0.12 * y, 0.0)) });
        }
        return MotorTree::new(models).unwrap();
    }

    /// Thrusters along x, y and z: S is all of space.
    pub fn holonomic() -> MotorTree {
        return MotorTree::new(vec![motor(1, 0, WorkingAxis::LinearMotionALongX, Vec3::new(0.0, 0.0, 0.0)),
                                   motor(2, 0, WorkingAxis::LinearMotionALongY, Vec3::new(0.0, 0.0, 0.0)),
                                   motor(3, 0, WorkingAxis::LinearMotionALongZ, Vec3::new(0.0, 0.0, 0.0))]).unwrap();
    }
}

/// Hand-predicted values: the trees of `test_trees`, and the validation tables of `resolver.mdx`.
#[cfg(test)]
mod tests {
    use crate::messages::motor_messages::WorkingAxis;

    use super::{test_trees::{holonomic, motor, osprai, quadrotor}, *};

    #[test]
    fn a_faulty_description_is_refused() {
        // each case is one fault in an otherwise valid one-motor tree, and the message names it
        let rotor= |id, parent_id| motor(id, parent_id, WorkingAxis::LinearMotionALongZ, Vec3::new(0.0, 0.0, 0.0));
        let refused= |models: Vec<MotorModel>, fault: &str| match MotorTree::new(models) {
            Ok(_) => panic!("accepted, expected: {}", fault),
            Err(message) => assert!(message.contains(fault), "{} (expected: {})", message, fault),
        };
        // 1. the id 0 is the vehicle's
        refused(vec![rotor(0, 0)], "id 0");
        // 2. two motors with the same id
        refused(vec![rotor(1, 0), rotor(1, 0)], "used twice");
        // 3. a parent that is not in the tree
        refused(vec![rotor(1, 7)], "unknown parent");
        // 4. a parent chain that never reaches the vehicle
        refused(vec![rotor(1, 2), rotor(2, 1)], "cycle");
        // 5. an axis that is neither a joint nor a thruster
        refused(vec![motor(1, 0, WorkingAxis::Unknown, Vec3::new(0.0, 0.0, 0.0))], "unknown working axis");
        // 6. bounds the wrong way round (clamping on them would panic)
        refused(vec![MotorModel { min_value: 1.0, max_value: 0.0, ..rotor(1, 0) }], "above max_value");
    }

    #[test]
    fn the_tree_derives_its_order_and_its_children() {
        let tree= osprai();
        // 1. parents first: the two arms (depth 0), then the two rotors (depth 1), each pair by id
        assert_eq!(tree.order(), &[74, 189, 64, 179]);
        // 2. each arm carries its rotor, a rotor carries nothing
        assert_eq!((tree.children(74), tree.children(189), tree.children(64)), (&[64][..], &[179][..], &[][..]));
    }

    #[test]
    fn lever_arms_are_measured_from_the_com() {
        // Geometry of the reduction point (bug #11, see `lessons.mdx`). An arm sits at (0, 0.18, 0.10)
        // from the vehicle origin and its rotor 0.12 m further along y, given in the ARM's frame:
        // measured from the origin, the rotor is at (0, 0.30, 0.10). The CoM is 1/60 m above the
        // origin (the OSPRAI's), so the lever arms about the CoM are the origin-based positions minus
        // (0, 0, 1/60): z = 0.10 - 0.0167 = 0.0833. Seen red with the former `+= p_com`.
        let transforms= osprai().transforms(Vec3::new(0.0, 0.0, 1.0 / 60.0), |_| 0.0);
        let (arm, rotor)= (Vector3::from(transforms[&74].location), Vector3::from(transforms[&64].location));
        assert!((arm - Vector3::new(0.0, 0.18, 0.10 - 1.0 / 60.0)).norm() < 1e-12, "arm at {:?}", arm);
        assert!((rotor - Vector3::new(0.0, 0.30, 0.10 - 1.0 / 60.0)).norm() < 1e-12, "rotor at {:?}", rotor);
    }

    #[test]
    fn the_subspace_comes_from_the_motor_tree() {
        // 1. quadrotor: four thrusts along z, nothing tilts them: a line, v = +z
        let ForceSubspace::Line(v)= quadrotor().force_subspace().clone() else { panic!("not a line") };
        assert!((v - Vector3::z()).norm() < 1e-12, "{:?}", v);
        // 2. OSPRAI: thrusts along z, swept about y by the arms: the xz plane, normal +/- y
        let ForceSubspace::Plane(n)= osprai().force_subspace().clone() else { panic!("not a plane") };
        assert!((n.dot(&Vector3::y()).abs() - 1.0).abs() < 1e-12, "{:?}", n);
        // 3. holonomic: all of space, no characteristic vector
        assert_eq!(holonomic().force_subspace(), &ForceSubspace::Space);
        // 4. no motor: nothing reachable
        assert_eq!(MotorTree::new(vec![]).unwrap().force_subspace(), &ForceSubspace::Empty);
    }

    #[test]
    fn a_tree_reads_from_a_json_file() {
        // 1. a one-arm, one-rotor tree as a configuration file writes it: vectors as [x, y, z], the
        //    axis by its name, the orientation and every constant left out (identity, 0)
        let json= r#"{ "motors": [
            { "id": 1, "working_axis": "RotatingAroundY", "relative_location": [0.0, 0.18, 0.10],
              "min_value": -1.57, "max_value": 1.57 },
            { "id": 2, "parent_id": 1, "working_axis": "LinearMotionALongZ", "relative_location": [0.0, 0.12, 0.0],
              "min_value": 0.1, "max_value": 17.28, "moment_constant": -0.016 } ] }"#;
        let path= std::env::temp_dir().join(format!("robomorph_motor_tree_{}.json", std::process::id()));
        std::fs::write(&path, json).unwrap();
        let tree= MotorTree::from_json(&path);
        let _= std::fs::remove_file(&path);
        let tree= tree.unwrap();
        // 2. every field where it belongs
        let rotor= tree.get(2).unwrap();
        assert!(rotor.is_thruster() && rotor.parent_id == 1 && rotor.moment_constant == -0.016 && rotor.relative_orientation.is_none());
        assert_eq!(tree.children(1), &[2]);
        // 3. the missing orientation is the identity: the rotor sits at (0, 0.30, 0.10), and S is the
        //    OSPRAI's half, the xz plane
        let rotor_location= Vector3::from(tree.transforms(Vec3::new(0.0, 0.0, 0.0), |_| 0.0)[&2].location);
        assert!((rotor_location - Vector3::new(0.0, 0.30, 0.10)).norm() < 1e-12, "{:?}", rotor_location);
        assert!(matches!(tree.force_subspace(), ForceSubspace::Plane(_)));
        // 4. a file that does not describe a tree says why
        assert!(MotorTree::from_json(Path::new("/nonexistent/motors.json")).is_err());
    }
}
