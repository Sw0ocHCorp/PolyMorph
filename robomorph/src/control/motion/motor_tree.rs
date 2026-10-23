//! Geometry of the motor tree, shared by the stages that need it: the mixer (lever arms and thrust
//! directions at the CURRENT joint angles, to allocate) and the force -> attitude resolver (thrust
//! directions and joint axes AT REST, to derive the reachable force subspace). Both receive the
//! tree the same way, on the motor configuration channel (`receive_motors`).
//!
//! A motor is described at rest, in its parent's frame (`MotorModel::relative_location`,
//! `relative_orientation`); a root motor (`parent_id == 0`) hangs directly under the vehicle.
//! Theory: `docs/docs/pipelines/motor-control/motor-model.mdx`.

use std::collections::HashMap;

use nalgebra::{UnitQuaternion, Vector3};
use tokio::sync::broadcast::Receiver;

use crate::{control::motion::motor_controller::{MotorController, working_axis_i32_to_vec3}, messages::{motor_messages::WorkingAxis, pose_messages::Transform, registered_message::{UnitQuat, Vec3}}};

/// Absorb the motor descriptions waiting on the configuration channel into `motors` (keyed by id, a
/// newer description replacing an older one), without blocking. Returns whether any arrived, so a
/// stage can derive again what depends on the tree.
pub fn receive_motors(receiver: &mut Receiver<MotorController>, motors: &mut HashMap<u32, MotorController>) -> bool {
    let mut received= false;
    // 1. only what is already queued: a pass never waits for the tree
    for _ in 0..receiver.len() {
        // 2. a lagged or closed channel yields nothing: the descriptions already held stay
        if let Ok(motor)= receiver.try_recv() {
            motors.insert(motor.get_motor_model().id, motor);
            received= true;
        }
    }
    return received;
}

/// The ids of `motors`, ordered so a parent always comes before its children: depth ascending (the
/// length of the parent_id chain up to a root, parent_id == 0), ties broken by id. Forward =
/// root -> leaves, reversed = leaves -> root. The order never depends on the HashMap iteration
/// order, which is randomised per map: what is computed by sweeping the tree in this order (the
/// mixer's allocation, the resolver's subspace) is reproducible from one run to the next.
pub fn parents_first_order(motors: &HashMap<u32, MotorController>) -> Vec<u32> {
    // 1. every motor id, in whatever order the HashMap gives them
    let mut processing_order: Vec<u32>= motors.keys().copied().collect();
    // 2. sort by a key computed for each id: the function passed to sort_by_key(...) takes an id
    //    and returns its sorting key, here (depth, id)
    processing_order.sort_by_key(|id| {
        // 2a. depth: walk up the parent_id chain until a root, counting the steps
        let mut depth= 0usize;
        let mut current= *id;
        while let Some(mc)= motors.get(&current) {
            if mc.get_motor_model().parent_id == 0 { break; }
            current= mc.get_motor_model().parent_id;
            depth += 1;
            if depth > motors.len() { break; } // guard against a malformed cyclic chain
        }
        // 2b. the key: depth first (parents before children), then id, so that motors of equal
        //     depth do not follow the HashMap order
        (depth, *id)
    });
    return processing_order;
}

/// Resolve every motor's absolute transform in the BODY frame, by walking the kinematic chain.
/// `motor_ids` MUST be ordered parents first: a child's pose is expressed relative to its parent's.
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
/// `joint_angle` gives theta_j for an angular joint: the mixer passes the measured angle (its
/// feedback), the force -> attitude resolver passes 0 to get the tree AT REST. `com` is the CoM
/// in the vehicle frame (the reduction point of the positions; directions do not depend on it).
pub fn motor_transforms(motors: &HashMap<u32, MotorController>, motor_ids: &[u32], com: Vec3, joint_angle: impl Fn(&MotorController) -> f64) -> HashMap<u32, Transform> {
    let mut motor_transforms: HashMap<u32, Transform>= HashMap::new();
    for motor_id in motor_ids {
        let Some(motor_controller)= motors.get(motor_id) else {
            continue
        };
        let motor_model= motor_controller.get_motor_model();
        // IF the parent transform has already been computed
        if let Some(parent_transform) = motor_transforms.get(&motor_model.parent_id) {
            let mut motor_transform= parent_transform.clone();
            // ORIENTATION      R_j = R_parent * R_j^rel * Rot(e_j, theta_j)
            if let Some(m_orientation)= motor_model.relative_orientation {
                // R_j^rel : rest orientation of j, in its parent's frame
                let mut motor_orientation= UnitQuaternion::from(m_orientation);
                if motor_model.working_axis <= WorkingAxis::RotatingAroundZ as i32 {
                    // R_j^rel * Rot(e_j, theta_j) : an angular joint adds its current angle about its
                    // own axis. Composed on the RIGHT because e_j is expressed in j's frame.
                    motor_orientation *= UnitQuaternion::from_axis_angle(&working_axis_i32_to_vec3(motor_model.working_axis), joint_angle(motor_controller));
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
        //ELSE IF the motor is the first motor of the kinematic chain
        else if motor_model.parent_id == 0 {
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
            // * Rot(e_j, theta_j) : same as above, an angular joint adds its current angle
            if motor_model.working_axis <= WorkingAxis::RotatingAroundZ as i32 {
                motor_orientation *= UnitQuaternion::from_axis_angle(&working_axis_i32_to_vec3(motor_model.working_axis), joint_angle(motor_controller));
            }
            motor_transforms.insert(*motor_id, Transform {location: motor_location, orientation: UnitQuat::from(motor_orientation)});
        }
        // a non-root motor whose parent is not resolved yet means motor_ids is not parents-first
        else {
            println!("[ERROR] -> Motor#{} has a parent (Motor#{}) but it's transform is not computed", motor_id, motor_model.parent_id);
        }
    }
    return motor_transforms;
}
