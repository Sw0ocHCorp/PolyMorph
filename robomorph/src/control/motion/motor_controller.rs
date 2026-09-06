//! Per-motor building block of the control allocation: for every motor of the kinematic tree,
//! how much wrench it contributes NOW (its "work vector") and how much wrench the vehicle would gain
//! per unit of command on it (its "effectiveness column").
//!
//! The two are different objects and the distinction is the key to understanding the mixer:
//!   - the COLUMN `a_j` is a SLOPE (a sensitivity): "if I add one newton / one radian on this motor,
//!     what does the vehicle gain?". For a thruster it is pure geometry and exists even when the
//!     rotor is stopped;
//!   - the WORK VECTOR `w_j` is a VALUE: "what does this motor produce right now?". For a thruster
//!     `w_j = T * a_j` where `T` is the current thrust (N) - zero when the rotor is stopped, while
//!     the column is not. A lever keeps its lever arm when nobody pushes on it.
//!
//! An angular joint produces no wrench of its own: it reorients its children's. Its column AND its
//! work vector are proportional to its children's thrust, so at start-up (rotors stopped) a joint has
//! no authority at all and sits out of the allocation until the rotors' feedback reports thrust.
//!
//! Full derivation and notation: the NOTATION block in `motors_mixer.rs` and
//! `docs/src/controle/modele-moteur.md`.

use std::{collections::HashMap, fmt::{Display, Formatter}};

use chrono::Utc;
use nalgebra::{UnitQuaternion, UnitVector3, Vector3};
use prost_types::Timestamp;

use crate::{control::{pid_controller::PIDController}, messages::{motor_messages::{MotorFeedBack, MotorModel, MotorStatus, WorkVec, WorkingAxis}, pose_messages::Transform, registered_message::Vec3}};

/// Roll / pitch / yaw (rad, ZYX convention) of a quaternion. DISPLAY ONLY: the control law never
/// uses Euler angles (singular at +/-90 deg of pitch, and their axes are not the body axes on
/// which the mixer produces moments). Useful to read a log.
pub fn quaternion_to_euler(quat: &UnitQuaternion<f64>) -> [f64; 3] {
    let w = quat.w;
    let x = quat.i;
    let y = quat.j;
    let z = quat.k;

    // Roll (x-axis rotation)
    let sinr_cosp = 2.0 * (w * x + y * z);
    let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    let roll = sinr_cosp.atan2(cosr_cosp);

    // Pitch (y-axis rotation) - handle gimbal lock
    let sinp = 2.0 * (w * y - z * x);
    let pitch = if sinp.abs() >= 1.0 {
        std::f64::consts::FRAC_PI_2.copysign(sinp)
    } else {
        sinp.asin()
    };

    // Yaw (z-axis rotation)
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    let yaw = siny_cosp.atan2(cosy_cosp);

    return [roll, pitch, yaw];
}

/// 6D selector `[fx fy fz mx my mz]` of the single wrench component a motor acts on directly, from
/// its `WorkingAxis`. Not used by the allocation (which builds full geometric columns), kept as a
/// helper.
/// Get the vector6 that map the axes of rotation and translation on which the motor exerts an influence(Expressed in the Body frame of the vehicle)
pub fn working_axis_to_vec6(axis: WorkingAxis) -> [f64; 6] {
    let mut motor_axis= [0.0; 6];
    motor_axis= match axis {
        WorkingAxis::Unknown => [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        WorkingAxis::LinearMotionALongX => [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        WorkingAxis::LinearMotionALongY => [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
        WorkingAxis::LinearMotionALongZ => [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        WorkingAxis::RotatingAroundX    => [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
        WorkingAxis::RotatingAroundY    => [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        WorkingAxis::RotatingAroundZ    => [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    };
    return motor_axis;
}

/// Get the vector6 that map the axes of rotation and translation on which the motor exerts an influence(Expressed in the Body frame of the vehicle)
pub fn working_axis_i32_to_vec6(axis: i32) -> [f64; 6] {
    let mut motor_axis= [0.0; 6];
    if let Ok(working_axis) = WorkingAxis::try_from(axis) {
        motor_axis= match working_axis {
            WorkingAxis::Unknown => [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            WorkingAxis::LinearMotionALongX => [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            WorkingAxis::LinearMotionALongY => [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            WorkingAxis::LinearMotionALongZ => [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            WorkingAxis::RotatingAroundX    => [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            WorkingAxis::RotatingAroundY    => [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            WorkingAxis::RotatingAroundZ    => [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        }
    }
    return motor_axis;
}

/// The motor's own working axis `e_j` as a unit vector in ITS OWN frame: thrust direction for a
/// thruster, rotation axis for a joint. Rotated by the resolved motor orientation it becomes
/// `fhat` (thruster) or `ahat` (joint) in the body frame.
pub fn working_axis_to_vec3(axis: WorkingAxis) -> UnitVector3<f64> {
    let mut motor_axis= Vector3::zeros();
    //the thruster effort axis of the motor 
    motor_axis= match axis {
        WorkingAxis::LinearMotionALongX => Vector3::x(),
        WorkingAxis::RotatingAroundX => Vector3::x(),
        WorkingAxis::LinearMotionALongY => Vector3::y(),
        WorkingAxis::RotatingAroundY => Vector3::y(),
        WorkingAxis::LinearMotionALongZ => Vector3::z(),
        WorkingAxis::RotatingAroundZ => Vector3::z(),
        _ => Vector3::zeros(),
    };
    return UnitVector3::new_normalize(motor_axis);
}

/// Same as `working_axis_to_vec3` from the raw `i32` stored in the protobuf `MotorModel`.
/// An unknown value yields a zero vector, normalised - callers must pass a valid axis.
pub fn working_axis_i32_to_vec3(axis: i32) -> UnitVector3<f64> {
    let mut motor_axis= Vector3::zeros();
    if let Ok(working_axis) = WorkingAxis::try_from(axis) {
        //the thruster effort axis of the motor 
        motor_axis= match working_axis {
            WorkingAxis::LinearMotionALongX => Vector3::x(),
            WorkingAxis::RotatingAroundX => Vector3::x(),
            WorkingAxis::LinearMotionALongY => Vector3::y(),
            WorkingAxis::RotatingAroundY => Vector3::y(),
            WorkingAxis::LinearMotionALongZ => Vector3::z(),
            WorkingAxis::RotatingAroundZ => Vector3::z(),
            _ => Vector3::zeros(),
        };
    }
    return UnitVector3::new_normalize(motor_axis);
}

/// `Clone` is required to travel through a `broadcast` channel: the channel owns the value
/// and hands a clone to every subscriber.
/// One node of the vehicle's motor tree as the mixer sees it: its static description
/// (`model`), its last known state (`feedback`) and a per-motor PID (reserved for local servoing,
/// unused by the allocation).
///
/// The mixer keeps one `MotorController` per motor id, refreshed by the feedback channel. The
/// feedback's `current_value` is the motor's CURRENT EFFORT in the mixer's own units:
///   - thruster: the thrust `T` it produces right now, in newtons (reconstructed at the hardware
///     boundary from the measured rotor speed through the effort law `T = k * w^n`);
///   - angular joint: its current angle `theta_j`, in radians.
#[derive(Clone)]
pub struct MotorController {
    model: MotorModel,
    feedback: MotorFeedBack,
    pid: PIDController,
}

impl Display for MotorController {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        return write!(
            f,
            "MotorController {{\n  .model= {:?}\n  .feedback= {:?}\n  .pid= {:?}\n}}",
            self.model, self.feedback, self.pid,
        );
    }
}

impl MotorController {
    /// Bundle a model, its initial feedback and a PID. The feedback is expected to carry the
    /// same `id` and `command_type` as the model's family (see `update_motor_feedback`).
    pub fn new(motor_model: MotorModel, motor_fbbk: MotorFeedBack, motor_pid: PIDController) -> Self {
        return Self { model: motor_model, feedback: motor_fbbk, pid: motor_pid }
    }


    /// Compute this motor's contribution to the vehicle's control-allocation matrix:
    /// a single 6D wrench column `[fx, fy, fz, mx, my, mz]` in the body frame, moment
    /// taken about the CoM.
    ///
    /// - A **linear motor / thruster** returns its per-unit-effort wrench: force is the
    ///   thrust direction, moment is `position x direction + moment_constant * direction`.
    /// - An **angular motor / joint** returns the gradient of its subtree's wrench with
    ///   respect to its own angle - how the wrench changes as the joint turns.
    ///
    /// The two returns are different objects and must not be confused:
    ///   - the COLUMN says what the vehicle would gain per unit of command on this motor.
    ///     Its unit follows the decision variable: per newton for a thruster, per radian for a joint.
    ///   - the WORK VECTOR is the wrench the motor produces right now, with its current command.
    ///     Summed over the roots it gives the vehicle's current wrench, i.e. the linearisation point.
    ///
    /// Two properties of the joint branch are worth keeping in mind:
    ///   - a joint produces no wrench of its own, it only REORIENTS what its children produce.
    ///     Its column is therefore proportional to its children's thrust: at zero thrust it is
    ///     exactly zero and the joint disappears from the allocation.
    ///   - the column is a derivative, so it is only valid for a small angle. The caller is
    ///     responsible for keeping the commanded increment inside that domain.
    ///
    /// Arguments:

    pub fn compute_motor_efforts(&self, motor_transforms: &HashMap<u32, Transform>, childs_motor_works: &HashMap<u32, WorkVec>,
                                    motor_models: &HashMap<u32, MotorModel>) -> ([f64; 6], WorkVec) {
        if let Some(motor_transform)= motor_transforms.get(&self.model.id) {
                //IF the motor is a linear motor / thruster
                //the effort motor vector depends on the parent motor
                if self.model.working_axis >= WorkingAxis::LinearMotionALongX as i32{
                    // THRUSTER - a leaf force producer. Notation:
                    //      e     = its effort axis, in its own frame (unit)
                    //      fhat  = R_j * e            thrust DIRECTION in the body frame (unit)
                    //      p     = p_j                its position in the body frame, from the reduction point
                    //      k_m   = moment_constant    reaction moment per unit thrust, signed by the spin direction
                    //      T     = current_value      the thrust it produces right now (N)
                    //
                    // COLUMN, per unit of thrust:
                    //      a = [ fhat ; p x fhat + k_m * fhat ]
                    //          p x fhat   = lever arm about the reduction point
                    //          k_m * fhat = intrinsic reaction moment (propeller drag on the airframe),
                    //                       carried along the thrust axis. Its sign is the spin direction:
                    //                       two counter rotating rotors must have opposite k_m, otherwise
                    //                       the model sees a yaw torque the vehicle does not actually produce.
                    // WORK VECTOR, what it produces right now:
                    //      w = T * a           strictly linear in T, which is why the column is EXACT
                    // fhat = R_j * e            thrust direction in the body frame, unit vector
                    let thrust_force= (UnitQuaternion::from(motor_transform.orientation) * working_axis_i32_to_vec3(self.model.working_axis)).into_inner();
                    // moment = p x fhat + k_m * fhat
                    //      p x fhat    lever arm of the thrust about the reduction point
                    //      k_m * fhat  reaction moment of the propeller, along the thrust axis
                    let moments= Vector3::from(motor_transform.location).cross(&thrust_force) + self.model.moment_constant * thrust_force;
                    // A thruster is a leaf force producer: its column is its own per-unit-effort
                    // wrench(work vector), returned directly. Child motors are irrelevant here.
                    // current_value is ALREADY the effort of the motor (a thrust, in N): the command law
                    // (thrust <-> rotor speed) lives in the hardware interface, not here. Applying
                    // k * w^exp again would square a force and shrink the wrench by several decades.
                    // T = current_value        the thrust produced right now (N)
                    let motor_factor= self.feedback.current_value;
                    // w = T * a                 work vector = column scaled by the current thrust
                    let work_vec= WorkVec {fx: motor_factor * thrust_force.x, fy: motor_factor * thrust_force.y, fz: motor_factor * thrust_force.z, 
                                                        mx: motor_factor * moments.x, my: motor_factor * moments.y, mz: motor_factor * moments.z};
                    return ([thrust_force.x, thrust_force.y, thrust_force.z, moments.x, moments.y, moments.z], work_vec);
                } 
                //ELSE the motor is an angular motor
                //the effort vector depends on his childs
                else {
                    // ANGULAR JOINT - produces no wrench of its own, it reorients its subtree.
                    // Its column is the DERIVATIVE of the subtree wrench with respect to its own angle,
                    // summed over its children ci. Notation:
                    //      ahat = R_j * e             joint rotation axis in the body frame
                    //      q    = p_j                 joint pivot, body frame
                    //      p_ci = child position      body frame
                    //      f_ci = child force         its current force, from the child work vector
                    //      m_ci = k_m,ci * f_ci       child intrinsic moment
                    //
                    // Under an infinitesimal rotation dtheta about ahat through q, every point of the
                    // subtree moves by  dtheta * (ahat x (p - q))  and every vector rotates by
                    // dtheta * (ahat x v). Differentiating the subtree wrench gives:
                    //
                    //      COLUMN force part :   dF/dtheta  = ahat x f_ci
                    //      WORK VECTOR       :   w = sum of the children work vectors (the joint adds nothing)

                    //   force  : ahat x f_ci
                    // depends on the child's type.
                    //      COLUMN moment part, depends on the child type:
                    //
                    //   child is a THRUSTER - differentiate  M_ci = p_ci x f_ci + m_ci  term by term:
                    //      dM/dtheta = (ahat x (p_ci - q)) x f_ci      the child SWINGS about the pivot,
                    //                                                  its lever arm changes
                    //                + p_ci x (ahat x f_ci)            its force DIRECTION turns,
                    //                                                  applied at the same lever arm
                    //                + ahat x m_ci                     its intrinsic moment turns too
                    //
                    //   child is another JOINT - its wrench already aggregates its own subtree, so use it
                    //   as a block. Taking its moment back to the pivot q:
                    //      m_ci      = M_ci - q x f_ci                 subtree moment about q
                    //      dM/dtheta = ahat x m_ci + q x (ahat x f_ci)
                    // The two child cases are the SAME formula written two ways, they were checked to be
                    // identical: under a rigid rotation about ahat through q, the subtree wrench taken at q
                    // simply rotates, so  dM_O/dtheta = ahat x (M_O - q x F) + q x (ahat x F). Expanding it
                    // with the identity  ahat x (u x f) = (ahat x u) x f + u x (ahat x f)  gives back the
                    // a + b + c terms of the thruster case. No need to re-derive either branch.
                    //
                    // Every term above is LINEAR in f_ci: that is the algebraic reason why a joint's column
                    // vanishes when its children produce no thrust.
                    // a = dw/dtheta, accumulated over the children
                    let mut wrench_gradient= [0.0; 6];
                    // ahat = R_j * e            joint rotation axis in the body frame.
                    // Rot(e, theta) leaves e unchanged, so this axis does not depend on the joint angle.
                    let rotating_axis= UnitQuaternion::from(motor_transform.orientation) * working_axis_i32_to_vec3(self.model.working_axis);
                    // w = sum of the children work vectors: a joint adds no wrench of its own
                    let mut work_vec= WorkVec::default();
                    for child_id in &self.model.child_ids {
                        //the wrench of an angular joint is the sum of it's childs wrenchs
                        if let Some(child_work_vec) = childs_motor_works.get(child_id) && let Some(child_model)= motor_models.get(child_id) &&
                                let Some(child_transform)= motor_transforms.get(child_id) {
                            // f_ci                      child's current force, from its work vector
                            let child_thrust= Vector3::new(child_work_vec.fx, child_work_vec.fy, child_work_vec.fz);
                            // dF/dtheta = ahat x f_ci    force part of the column: the child's force
                            //                            direction turns with the joint
                            let thrust_force= rotating_axis.cross(&child_thrust);
                            // w += w_ci                  the joint's work vector is the sum of its children's
                            work_vec += *child_work_vec;
                            wrench_gradient[0]+= thrust_force.x;
                            wrench_gradient[1]+= thrust_force.y;
                            wrench_gradient[2]+= thrust_force.z;
                            //IF the child is a linear motor (wheel / thruster / etc...)
                            //build it's moments based on it's geometry, relative to it's parent
                            if child_model.working_axis >= WorkingAxis::LinearMotionALongX as i32  { 
                                // CHILD IS A THRUSTER - differentiate  M_ci = p_ci x f_ci + m_ci  term by term.
                                //
                                // p_ci                      child position in the body frame
                                let child_location= Vector3::from(child_transform.location);
                                // m_ci = k_m,ci * f_ci      child's intrinsic reaction moment at its current force
                                let child_moments= child_model.moment_constant * child_thrust;
                                //
                                // a = (ahat x (p_ci - q)) x f_ci     the LEVER ARM changes: the child swings
                                //                                    about the pivot along an arc
                                //      p_ci - q               lever arm from the pivot to the child
                                //      ahat x (p_ci - q)      velocity of the child when the joint turns
                                //      (...) x f_ci           the moment that motion creates
                                let a= (rotating_axis.cross(&(child_location - Vector3::from(motor_transform.location)))).cross(&child_thrust);
                                // b = p_ci x (ahat x f_ci)           the FORCE DIRECTION turns, applied at the
                                //                                    same lever arm
                                //      ahat x f_ci            how the child's force rotates with the joint
                                //      p_ci x (...)           its moment about the reduction point
                                let b= child_location.cross(&rotating_axis.cross(&child_thrust));
                                // c = ahat x m_ci                    the child's INTRINSIC MOMENT turns too
                                let c= rotating_axis.cross(&child_moments);
                                // dM/dtheta = a + b + c              moment part of the column
                                let moments= a + b + c;
                                    
                                wrench_gradient[3]+= moments.x;
                                wrench_gradient[4]+= moments.y;
                                wrench_gradient[5]+= moments.z;
                            } 
                            //ELSE the child is an angular motor
                            //it's wrench is already the sum of the wrenchs of it's childs, use it directly
                            else {
                                // CHILD IS ANOTHER JOINT - its work vector already aggregates its whole
                                // subtree, so it is handled as one rigid block rotating about the pivot.
                                //
                                // M_ci                      subtree moment, taken about the reduction point
                                let child_moments= Vector3::new(child_work_vec.mx, child_work_vec.my, child_work_vec.mz);
                                // q = p_j                   the pivot, i.e. this joint's own position
                                let motor_location= Vector3::from(motor_transform.location);
                                // m_ci = M_ci - q x f_ci    same moment, moved to the pivot. Rotating a rigid
                                //                           body about q only rotates its moment taken AT q.
                                let motor_moments= child_moments - motor_location.cross(&child_thrust);
                                // dM/dtheta = ahat x m_ci + q x (ahat x f_ci)
                                //      ahat x m_ci          the subtree moment at the pivot rotates
                                //      q x (ahat x f_ci)    and the rotated force is brought back to the
                                //                           reduction point through the pivot lever arm
                                let derived_moments= rotating_axis.cross(&motor_moments) + motor_location.cross(&thrust_force);
                                wrench_gradient[3]+= derived_moments.x;
                                wrench_gradient[4]+= derived_moments.y;
                                wrench_gradient[5]+= derived_moments.z;
                            }
                        }
                    }
                    return (wrench_gradient, work_vec);
                }
        } else {
            return ([0.0; 6], WorkVec::default());
        }
    }

    pub fn get_motor_model(&self) -> &MotorModel {
        return &self.model;
    }

    pub fn get_motor_feedback(&self) -> &MotorFeedBack {
        return &self.feedback;
    }

    pub fn get_motor_pid(&self) -> &PIDController {
        return &self.pid;
    }

    /// Replace the feedback with a fresher one - ONLY if it belongs to this motor (same id) and
    /// speaks the same unit (same `command_type`): a THRUST feedback must never land on a joint.
    pub fn update_motor_feedback(&mut self, motor_feedback: MotorFeedBack) {
        if self.feedback.id == motor_feedback.id && 
                self.feedback.command_type == motor_feedback.command_type {
            self.feedback= motor_feedback;
        }
    }

    pub fn set_motor_status(&mut self, new_status: MotorStatus) {
        self.feedback.status= new_status as i32;
    }

    pub fn set_motor_min_value(&mut self, motor_min_value: f64) {
        self.model.min_value= motor_min_value;
    }

    /// Set the current effort (N or rad) measured on the hardware / simulator. Non-finite values
    /// are refused: a NaN here would become the mixer's linearisation point and poison every
    /// column of the allocation. Out-of-bounds values are accepted but reported.
    pub fn set_motor_current_value(&mut self, new_current_value: f64) {
        if (new_current_value < self.model.min_value || new_current_value > self.model.max_value) && new_current_value > 1e-4{
            println!("[WARNING] -> Motor #{}: current value {} is out of bounds [{}, {}]", self.model.id, new_current_value, self.model.min_value, self.model.max_value);
        }
        if !new_current_value.is_finite() {
            println!("[WARNING] ->  Unable to update Motor#{} current value because the specified value is NaN or +/-INFINITY", self.model.id);
        } else {
            self.feedback.current_value= new_current_value;
        }
    }

    /// Record the effort the mixer just commanded (N or rad). The value is expected to be clamped
    /// to `[min_value, max_value]` by the caller; a violation is only reported.
    pub fn set_motor_setpoint_value(&mut self, new_setpoint_value: f64) {
        if new_setpoint_value < self.model.min_value || new_setpoint_value > self.model.max_value {
            println!("[WARNING] -> Motor #{}: setpoint value {} is out of bounds [{}, {}]", self.model.id, new_setpoint_value, self.model.min_value, self.model.max_value);
        }
        self.feedback.setpoint_value= new_setpoint_value;
    }

    pub fn set_motor_max_value(&mut self, motor_max_value: f64) {
        self.model.max_value= motor_max_value;
    }

    pub fn set_parent_id(&mut self, parent_id: u32) {
        self.model.parent_id= parent_id;
    }

    pub fn set_child_ids(&mut self, child_ids: Vec<u32>) {
        self.model.child_ids= child_ids
    }

    pub fn set_motor_relative_location(&mut self, relative_location: Vec3) {
        self.model.relative_location= Some(relative_location);
    }

    /// Speed limit of the motor: rad/s of rotor for a thruster (command ceiling), joint angular
    /// speed for a joint - in which case it is also the TRUST REGION of the allocation
    /// (`max_rot_speed * dt` per cycle). Negative values are refused.
    pub fn set_max_rot_speed(&mut self, limit_velocity: f64) {
        if limit_velocity >= 0.0 {        
            self.model.max_rot_speed= limit_velocity;
        } else {
            println!("[WARNING] ->  Unable to update Motor#{} max_rot_speed value because the specified value is negative: {}", self.model.id, limit_velocity);
        }
    }

    pub fn set_feedback_timestamp(&mut self, timestamp: Timestamp) {
        self.feedback.timestamp= Some(timestamp);
    }

    pub fn get_timestamp(&self) -> Option<Timestamp> {
        return self.feedback.timestamp;
    }

    pub fn get_working_axis(&self) -> WorkingAxis {
        if let Ok(working_axis) = WorkingAxis::try_from(self.model.working_axis) {
            return working_axis;
        }
        return WorkingAxis::Unknown;
    }

    /// Attach a child motor (a motor mounted on this one) in the kinematic tree. Idempotent.
    pub fn add_child(&mut self, child_id: u32) {
        if self.model.child_ids.contains(&child_id) {
            println!("[WARNING] -> Motor #{} already chld of Motor #{}", child_id, self.model.id);
        } else {
            self.model.child_ids.push(child_id);
        }
    }
}