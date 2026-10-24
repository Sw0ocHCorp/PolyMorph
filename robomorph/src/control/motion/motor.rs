//! What a motor knows how to do, split by role:
//!   - `impl MotorModel`, on its DESCRIPTION (fixed): its family, its axis, its bounds, and the
//!     wrench it contributes to the vehicle at a given operating point (`efforts`);
//!   - `impl MotorState`, on its STATE (runtime): recording a measurement, issuing a command.
//!
//! The wrench a motor contributes comes in two objects, and the distinction is the key to
//! understanding the mixer:
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
//! `docs/docs/pipelines/motor-control/motor-model.mdx`.

use std::collections::HashMap;

use nalgebra::{UnitQuaternion, Vector3};
use prost_types::Timestamp;

use crate::{control::motion::motor_tree::MotorTree, messages::{motor_messages::{MotorCommand, MotorCommandType, MotorFeedBack, MotorModel, MotorState, MotorStatus, WorkVec, WorkingAxis}, pose_messages::Transform}};

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

impl MotorModel {
    /// An angular joint (`RotatingAround{X,Y,Z}`): its effort is an angle (rad).
    pub fn is_joint(&self) -> bool {
        return matches!(WorkingAxis::try_from(self.working_axis),
                        Ok(WorkingAxis::RotatingAroundX | WorkingAxis::RotatingAroundY | WorkingAxis::RotatingAroundZ));
    }

    /// A thruster (`LinearMotionALong{X,Y,Z}`): its effort is a thrust (N).
    pub fn is_thruster(&self) -> bool {
        return matches!(WorkingAxis::try_from(self.working_axis),
                        Ok(WorkingAxis::LinearMotionALongX | WorkingAxis::LinearMotionALongY | WorkingAxis::LinearMotionALongZ));
    }

    /// The motor's own working axis `e_j`, a unit vector in ITS OWN frame: thrust direction for a
    /// thruster, rotation axis for a joint. Rotated by the resolved motor orientation it becomes
    /// `fhat` (thruster) or `ahat` (joint) in the body frame. Zero for `Unknown`, which a
    /// `MotorTree` never holds.
    pub fn axis(&self) -> Vector3<f64> {
        return match WorkingAxis::try_from(self.working_axis) {
            Ok(WorkingAxis::RotatingAroundX | WorkingAxis::LinearMotionALongX) => Vector3::x(),
            Ok(WorkingAxis::RotatingAroundY | WorkingAxis::LinearMotionALongY) => Vector3::y(),
            Ok(WorkingAxis::RotatingAroundZ | WorkingAxis::LinearMotionALongZ) => Vector3::z(),
            _ => Vector3::zeros(),
        };
    }

    /// `value` (N or rad) brought back inside the motor's bounds `[min_value, max_value]`.
    pub fn clamp(&self, value: f64) -> f64 {
        return f64::clamp(value, self.min_value, self.max_value);
    }

    /// This motor's contribution to the vehicle's control-allocation matrix, at the operating point
    /// `measured` (its current effort, N or rad): a single 6D wrench column `[fx, fy, fz, mx, my, mz]`
    /// in the body frame, moment taken about the CoM, and its work vector.
    ///
    /// - A **linear motor / thruster** returns its per-unit-effort wrench: force is the
    ///   thrust direction, moment is `position x direction + moment_constant * direction`.
    /// - An **angular motor / joint** returns the gradient of its subtree's wrench with
    ///   respect to its own angle - how the wrench changes as the joint turns. Its children come
    ///   from `tree`, their work vectors from `child_works` (already computed: leaves first).
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
    pub fn efforts(&self, measured: f64, motor_transforms: &HashMap<u32, Transform>, child_works: &HashMap<u32, WorkVec>,
                    tree: &MotorTree) -> ([f64; 6], WorkVec) {
        if let Some(motor_transform)= motor_transforms.get(&self.id) {
                //IF the motor is a linear motor / thruster
                //the effort motor vector depends on the parent motor
                if self.is_thruster() {
                    // THRUSTER - a leaf force producer. Notation:
                    //      e     = its effort axis, in its own frame (unit)
                    //      fhat  = R_j * e            thrust DIRECTION in the body frame (unit)
                    //      p     = p_j                its position in the body frame, from the reduction point
                    //      k_m   = moment_constant    reaction moment per unit thrust, signed by the spin direction
                    //      T     = measured           the thrust it produces right now (N)
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
                    let thrust_force= UnitQuaternion::from(motor_transform.orientation) * self.axis();
                    // moment = p x fhat + k_m * fhat
                    //      p x fhat    lever arm of the thrust about the reduction point
                    //      k_m * fhat  reaction moment of the propeller, along the thrust axis
                    let moments= Vector3::from(motor_transform.location).cross(&thrust_force) + self.moment_constant * thrust_force;
                    // A thruster is a leaf force producer: its column is its own per-unit-effort
                    // wrench(work vector), returned directly. Child motors are irrelevant here.
                    // `measured` is ALREADY the effort of the motor (a thrust, in N): the command law
                    // (thrust <-> rotor speed) lives in the hardware interface, not here. Applying
                    // k * w^exp again would square a force and shrink the wrench by several decades.
                    // T = measured             the thrust produced right now (N)
                    let motor_factor= measured;
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
                    let rotating_axis= UnitQuaternion::from(motor_transform.orientation) * self.axis();
                    // w = sum of the children work vectors: a joint adds no wrench of its own
                    let mut work_vec= WorkVec::default();
                    for child_id in tree.children(self.id) {
                        //the wrench of an angular joint is the sum of it's childs wrenchs
                        if let Some(child_work_vec) = child_works.get(child_id) && let Some(child_model)= tree.get(*child_id) &&
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
                            if child_model.is_thruster() {
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
}

impl MotorState {
    /// A motor that has neither been measured nor commanded yet.
    pub fn new(id: u32) -> Self {
        return Self { id, feedback: None, command: None };
    }

    /// The last measured effort (N or rad), 0 before the first measurement.
    pub fn get_measured_value(&self) -> f64 {
        return self.feedback.as_ref().map_or(0.0, |feedback| feedback.current_value);
    }

    /// Record a measurement of the motor described by `model` (N or rad), with the motor's status as
    /// the source of the measurement reports it:
    ///   1. out of the motor's bounds: accepted but reported (a rotor slightly below its floor while
    ///      it spins up is normal, hence the 1e-4 tolerance near zero);
    ///   2. not finite: refused, the previous measurement is kept. A NaN here would become the
    ///      mixer's linearisation point and poison every column of the allocation;
    ///   3. otherwise it becomes the motor's feedback, with its status.
    pub fn record_measurement(&mut self, model: &MotorModel, value: f64, status: MotorStatus, timestamp: Option<Timestamp>) {
        // 1. reported only
        if (value < model.min_value || value > model.max_value) && value > 1e-4 {
            println!("[WARNING] -> Motor #{}: current value {} is out of bounds [{}, {}]", self.id, value, model.min_value, model.max_value);
        }
        // 2. refused
        if !value.is_finite() {
            println!("[WARNING] ->  Unable to update Motor#{} current value because the specified value is NaN or +/-INFINITY", self.id);
            return;
        }
        // 3. the new measurement
        self.feedback= Some(MotorFeedBack { timestamp, id: self.id, status: status as i32, current_value: value });
    }

    /// The command of the motor described by `model`, kept as its last command:
    ///   1. `value` (N or rad) brought back inside the motor's bounds;
    ///   2. its unit from the motor's family: `THRUST` (N) for a thruster, `ANGULARPOSITION` (rad)
    ///      for a joint.
    pub fn get_command(&mut self, model: &MotorModel, value: f64, timestamp: Option<Timestamp>) -> MotorCommand {
        // 1. and 2.
        let command_type= if model.is_thruster() { MotorCommandType::THRUST } else { MotorCommandType::ANGULARPOSITION };
        let command= MotorCommand { timestamp, id: self.id, command_type: command_type as i32, setpoint_value: model.clamp(value) };
        self.command= Some(command.clone());
        return command;
    }
}
