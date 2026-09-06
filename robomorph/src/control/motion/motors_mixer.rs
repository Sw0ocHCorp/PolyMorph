//! Control allocation: given the 6D wrench the vehicle must produce, find the command of every motor
//! of its kinematic tree. Generic over the vehicle: the tree, the motor families and the geometry are
//! data (`MotorModel`), the solver is the same for a multirotor, a tilt-rotor or a wheeled rover.
//!
//! The THEORY block below is the reference for the notation and the six principles the solver rests
//! on (incremental formulation, meaning of a column, row normalisation, projected coordinate descent,
//! bounds as part of the problem, unreachable demands). `docs/src/controle/mixer.md` explains it in
//! prose with the lessons learnt during validation.

use std::{collections::HashMap, time::Duration};

use chrono::Utc;
use nalgebra::{DVector, Matrix3, Matrix6xX, UnitQuaternion, Vector3, Vector6};
use prost_types::Timestamp;
use tokio::sync::broadcast::{Receiver, Sender};

use crate::{control::{motion::{motion_controller::{MotionController, VehicleKinematicConfig}, motor_controller::{MotorController, working_axis_i32_to_vec3}}}, core::scheduler::Process, messages::{motor_messages::{MotorCommand, MotorCommandType, MotorFeedBack, MotorModel, WorkVec, WorkingAxis}, pose_messages::{Pose, Transform}, registered_message::{AnyMessage, UnitQuat, Vec3}}};

/// The control-allocation stage: turns one 6D wrench setpoint into one command per motor.
///
/// Inputs: the motor tree (`motor_config_receiver`, once), the motor feedbacks
/// (`motor_feedback_receiver`, every tick, published by the Gazebo `joint_state` callback) and the
/// wrench setpoint (scheduler pipe from the attitude stage). Output: `AnyMessage::MotorCommands`
/// on `cmd_sender` - efforts in the motor's own unit (N for a thruster, rad for a joint), the
/// conversion to actuator units happens at the hardware boundary.
///
/// The only state besides the motor table is `setpoint`, the last wrench received: it is HELD when
/// nothing new arrives (zero-order hold), which is what lets the mixer keep flying while the stage
/// above abstains.
pub struct MotorsMixer {
    name: String,
    cmd_sender: Option<Sender<AnyMessage>>,
    motor_config_receiver: Option<Receiver<MotorController>>,
    motor_feedback_receiver: Option<Receiver<MotorFeedBack>>,

    motors: HashMap<u32, MotorController>,
    vehicle_config: VehicleKinematicConfig,
    period: Duration,
    setpoint: WorkVec,
}

impl Process for MotorsMixer {
    fn set_name(&mut self, name: String) {
        self.name= name;
    }

    /// One cycle: (1) absorb new motor descriptions, (2) absorb fresh feedbacks (the measured motor
    /// state is the linearisation point of the solve), (3) take the wrench setpoint from the pipe
    /// if the previous process handed one over - otherwise keep the last one -, (4) allocate and
    /// publish the commands. Publishing goes through `cmd_sender` when someone listens, else the
    /// commands are returned into the pipe.
    fn exec(&mut self, input: &Option<AnyMessage>, dt: std::time::Duration) -> Option<AnyMessage> {
        //get the motors being controlled by the mixer
        if let Some(motor_config_rcvr)= &mut self.motor_config_receiver  {
            let sz= motor_config_rcvr.len();
            for _ in 0..sz {
                if let Ok(motor_controller)= motor_config_rcvr.try_recv() {
                    self.motors.insert(motor_controller.get_motor_model().id, motor_controller);
                }
            }
        }
        //get the current
        if let Some(feedbacks_rcvr)= &mut self.motor_feedback_receiver {
            let sz= feedbacks_rcvr.len();
            for _ in 0..sz {
                if let Ok(motor_feedback)= feedbacks_rcvr.try_recv() {
                    if let Some(mc)= self.motors.get_mut(&motor_feedback.id) {
                        mc.update_motor_feedback(motor_feedback);
                    }
                }
            }
        }
        if let Some(msg)= input && let AnyMessage::VehicleWrench(setpoint_msg)= msg {
            self.setpoint= *setpoint_msg;
        }
        //compute the motor commands using the motor controllers and the setpoint from previous process or asynchronous receiver
        let motor_cmds= self.compute_command_law(None, Some(AnyMessage::VehicleWrench(self.setpoint.clone())), dt, false);
        if let Some(cmds)= motor_cmds {
            if let Some(cmd_sender)= &mut self.cmd_sender && cmd_sender.receiver_count() > 0{
                let _= cmd_sender.send(cmds);
                return None;
            } else {
                return Some(cmds);
            }
        } else {
            return None;
        }
    }

    fn set_receiver(&mut self, _receiver: Receiver<AnyMessage>) {
        println!("[MotorsMixer:INFO] -> {} don't need input data to compute the motor's command law because it compute current vehicle wrench from controlled motors directly", self.name)
    }

    fn set_sender(&mut self, sender: Sender<AnyMessage>) {
        self.cmd_sender= Some(sender);
    }
    
    fn set_period_from_freq(&mut self, frequency: u64) {
        self.period= Duration::from_nanos(1_000_000_000 / frequency);
    }
    
    fn get_period(&self) -> Duration {
        return self.period;
    }
    
    fn get_name(&self) ->String {
        return self.name.clone();
    }

}

// ============================ THEORY OF THE MIXER ============================
//
// The problem solved here is CONTROL ALLOCATION: the vehicle needs one 6D wrench
// (3 forces + 3 moments, body frame, moments taken about the reduction point), and it
// has n motors. Find the n motor commands that produce it.
//
// ---------------------------- NOTATION ----------------------------
// Every symbol used below and in the per line comments of this file and of motor_controller.rs.
//
//   FRAMES AND INDICES
//     body frame        frame attached to the vehicle. Everything below lives in it unless stated.
//     reduction point   the point all moments are taken about: the CoM. Root motor positions are
//                       shifted by p_com so that every lever arm is measured from it.
//     j                 index of a motor
//     ci                index of a CHILD of motor j in the kinematic tree
//     n                 number of motors
//
//   VEHICLE
//     m                 vehicle mass                                            kg      (vehicle_config.weight)
//     I_k               moment of inertia about body axis k, k in {x,y,z}       kg.m2   (vehicle_config.moments_matrix)
//     p_com             position of the CoM in the vehicle frame                m       (com_relative_location)
//
//   KINEMATICS  (resolved by compute_motor_transforms)
//     e_j               working axis of motor j, in ITS OWN frame, unit. Thrust direction for a
//                       thruster, rotation axis for a joint.                            (working_axis)
//     theta_j           current angle of joint j, 0 for a thruster                rad   (current_value)
//     R_j^rel, p_j^rel  pose of j AT REST, expressed in its PARENT's frame              (relative_orientation
//                                                                                        / relative_location)
//     Rot(e, theta)     rotation of angle theta about axis e
//     R_j               orientation of j in the body frame                              (transform.orientation)
//     p_j               position    of j in the body frame                        m     (transform.location)
//     R_parent, p_parent   same two quantities for j's parent, already resolved
//     q                 pivot of a joint = p_j of that joint                       m
//
//   EFFORTS  (computed by compute_motor_efforts)
//     fhat              thrust DIRECTION of a thruster in the body frame, unit = R_j * e
//     ahat              rotation AXIS of a joint in the body frame, unit = R_j * e
//     T                 thrust a thruster produces right now                       N    (current_value)
//     k_m               reaction moment per unit thrust, along the thrust axis      m    (moment_constant)
//                       SIGNED by the spin direction: counter rotating rotors need opposite signs.
//     f_ci, M_ci        current force / moment of child ci, read from its work vector
//     m_ci              same moment moved to the pivot q:  m_ci = M_ci - q x f_ci
//
//   WRENCH AND ALLOCATION
//     wrench            6D vector [fx fy fz mx my mz]: 3 forces + 3 moments about the reduction point
//     w_j               WORK VECTOR of motor j: the wrench it produces right now       (WorkVec)
//     w_current         vehicle wrench right now = sum of the ROOT motors' work vectors (current_global_work)
//     w_setpoint        wrench the control law is asking for                            (compute_wrench_setpoint)
//     b                 what is MISSING = w_setpoint - w_current, row normalised        (delta_wrench)
//     a_j               EFFECTIVENESS COLUMN of motor j: the wrench gained per unit of command on j.
//                       Per newton for a thruster, per radian for a joint.              (column of effectivness_mat)
//     A                 6 x n matrix whose columns are the a_j                          (effectivness_mat)
//     dx_j              increment to apply on motor j's command. Unit = that of the motor's own
//                       effort: N for a thruster, rad for a joint.                      (additional_work)
//     r                 residual still to be served = b - A.dx                          (remaining_wrench)
//     1/||a_j||^2       exact step along column j                                       (motor_work_steps)
//     x . y             dot product        u x v   cross product        dz/dt  derivative
// ------------------------------------------------------------------
//
// 1. INCREMENTAL FORMULATION
//    We do not solve for the absolute commands, we solve for the INCREMENT to apply on
//    top of the measured motor state:
//              A . dx  ~=  w_setpoint - w_current
//    Two reasons:
//      - an angular joint's column is a tangent (see 2.), only valid near the current
//        configuration, so the problem has to be re-linearised every cycle anyway;
//      - w_current is rebuilt from the motor FEEDBACK, so the measured motor state is
//        the linearisation point. A wrong feedback is therefore a wrong plant model
//
// 2. WHAT A COLUMN OF `A` MEANS - and why the two motor families differ
//    Column j is "the wrench the vehicle gains per unit of command on motor j".
//      - THRUSTER: its wrench is T * [fhat ; p x fhat + k_m*fhat], strictly LINEAR in
//        the thrust T. The column is therefore EXACT for any increment, and the decision
//        variable dx_j is a force (N).
//      - ANGULAR JOINT: it produces no wrench of its own, it REORIENTS what its children
//        produce. Its column is the derivative dw/dtheta of its whole subtree, i.e. a
//        TANGENT: only valid for a small angle. The decision variable dx_j is an angle (rad).
//    Direct consequence, and the source of several failure modes: a joint's column is
//    proportional to its children's thrust. At zero thrust the column is exactly zero,
//    the joint has no authority at all, and the allocation loses rank.
//
// 3. WHY THE ROWS ARE NORMALISED (see STEP#2 and delta_wrench below)
//    Minimising ||A.dx - b||^2 adds newtons and newton-metres in the same norm, which is
//    dimensionally meaningless: the answer would change if lengths were expressed in mm.
//    Every row is therefore divided so that the residual is expressed in ACCELERATION:
//              force  row     / m
//              moment row k   / sqrt(m * I_k)      (m/s^2 seen at the radius of gyration)
//    This is a correctness requirement, not tuning: it is what decides, physically rather
//    than by unit accident, how the solver arbitrates between a force error and a moment error.
//
// 4. THE SOLVER: projected coordinate descent (Gauss-Seidel)
//    Motor j is moved along its own column by the amount that best reduces the residual:
//              dx_j += (a_j . r) / ||a_j||^2
//    That step is the EXACT minimiser along a_j, provided the other motors do not move at
//    the same time. Hence the two rules below:
//      - the step is PER COLUMN (1/||a_j||^2), never global: with mixed units a single step
//        is either divergent for the big columns or paralysing for the small ones;
//      - the residual is refreshed after EVERY motor, not once per sweep. Updating all
//        motors from the same residual (Jacobi) makes correlated columns correct the same
//        error twice and overshoot - on a bicopter the two rotor columns are ~83% correlated.
//    Each coordinate update can only lower the objective, so the sweep converges monotonically.
//
// 5. THE BOUNDS ARE PART OF THE PROBLEM
//    After each coordinate step the increment is clipped to what the motor can actually do.
//    This is a projection onto the feasible box, which keeps the descent valid.
//      - both families: min_value / max_value, the bounds of the motor's own effort
//        (N for a thruster, rad for a joint);
//      - joints only: additionally the TRUST REGION, max_rot_speed * dt, i.e. what the joint
//        can physically travel in one cycle. It keeps the increment inside the domain where
//        the tangent of point 2 still describes reality. A thruster needs none: its column
//        is exact.
//
// 6. WHAT THE SOLVER CANNOT DO
//    n motors can serve at most n wrench components. Any demand outside the column space
//    is simply unreachable and stays in the residual for ever - on a bicopter, the lateral
//    force fy, since no column has an fy entry. The residual norm is therefore NOT a
//    convergence indicator on its own: it converges to the unreachable part, not to zero.
// =============================================================================

impl MotionController for MotorsMixer {
    // Compute the commands to apply to the motors to reach the setpoint
    // based on their geometries, positions, and the forces they produces.
    // the goal is to apply the motors commands that produce the exact wrenches, for each motors that allow to reach the setpoint
    // 3 steps are needed to achieve that goal:
    //      STEP#1:
    //          Compute the transform(position, orientation) of each motors, based on their position / orientation at rest and their current orientation
    //          For that STEP, the motors tree is iterated from root to leaves because parent's transform is needed to compute child's transforms(especially if the parent is an angular joint)
    //      STEP#2:
    //          Compute the effectiveness column and current wrench of each motors. 
    //          The effectiveness columns will be used in the next STEP, to solve the motor efforts <=> motor commands problem.
    //              It allow to know on what dimensions, the motor has an effect
    //          The motor wrench vector will be used in 2 different places:
    //              1. Build the child motor tree to compute the angular joint grandient(his contribution)
    //              2. The global vehicle wrench is the sum of all root motors wrenches. This global wrench will be the current state of the motion controller 
    //                    the error e to minimize is e= setpoint_wench - current_wrench
    //          For that STEP, the motors tree is iterated from the leaves to the root because of the angular joints.
    //          The angular joint's wrench is computed, based on the child's wrenches 
    //      STEP#3:
    //          Build the control effectiveness matrix(solver) to get the motor commands to apply 
    //          
    fn compute_command_law(&mut self, _input_data: Option<AnyMessage>, setpoint: Option<AnyMessage>, dt: Duration, _verbose: bool)  -> Option<AnyMessage> {
        let mut motors_commands= Vec::new();
        let mut setpoint_wrench= self.setpoint;
        if let Some(msg) = setpoint && let AnyMessage::VehicleWrench(setpoint_msg)= msg {
            setpoint_wrench= setpoint_msg;
        }
        println!("[INFO] -> Setpoint wrench= {:?}", setpoint_wrench);
        let mut motor_transforms_map: HashMap<u32, Transform>= HashMap::new();
        let mut motors_wrench_map= HashMap::new();
        // the global wrench of the wehicle. Used in the STEP#3 (sum of the root motors wrenches)
        let mut current_global_work= WorkVec::default();
        // order the motors so a parent is always processed before its children (depth ascending).
        // depth is the length of the parent_id chain up to a root (parent_id == 0), so the order stays
        // correct whatever the HashMap iteration order. Forward = root->leaves (STEP#1), reversed = leaves->root (STEP#2).
        let mut processing_order: Vec<u32>= self.motors.keys().copied().collect();
        // the function passed to sort_by_key(...) use keys as input and return a value, used to sort the element
        // this returned value is like the weight of a key in the list, used to order elements in list 
        processing_order.sort_by_key(|id| {
            let mut depth= 0usize;
            let mut current= *id;
            while let Some(mc)= self.motors.get(&current) {
                if mc.get_motor_model().parent_id == 0 { break; }
                current= mc.get_motor_model().parent_id;
                depth += 1;
                if depth > self.motors.len() { break; } // guard against a malformed cyclic chain
            }
            // the ordering criteria
            depth
        });
            
        //STEP#1:
        //      Compute the current motor transform based on the potential parent motor, for each motor (parents first)
        motor_transforms_map= self.compute_motor_transforms(&processing_order);
        // the thruster effectivness mat (Used in STEP#3)
        // it map, the dimension were the vehicle can produce force, thanks to all these motors, given their transform (location / orientation)
        let mut effectivness_mat= Matrix6xX::zeros(self.motors.len());

        //STEP#2:
        //      Compute each motor's wrench column (thruster) or wrench gradient (angular joint),
        //      traversing leaves -> root: an angular joint's gradient is built from its children's
        //      already-resolved wrenches, so children must be computed first (reverse of STEP#1).
        // Snapshot of every motor model, so a joint can look up its children's geometry by id.
        let motor_models_map: HashMap<u32, MotorModel>= self.motors.iter()
                                                        .map(|(id, mc)| (*id, mc.get_motor_model().clone()))
                                                        .collect();
        // now i is used to fill the effectivness matrix with the effectivness vector of each thrusters
        let mut i= 0;
        let mut max_norm= 0.0;
        for motor_id in processing_order.iter().rev() {
            if let Some(motor_controller)= self.motors.get(motor_id) {
                println!("Motor #{}: parent_id= {}, child_ids= {:?}, min_value= {:?}, max_value= {:?}", motor_controller.get_motor_model().id, motor_controller.get_motor_model().parent_id, motor_controller.get_motor_model().child_ids, motor_controller.get_motor_model().min_value, motor_controller.get_motor_model().max_value);
                // get the parent_transform from the motor_transforms_map computed in STEP#1 None for a root motor.
                let (output_vec, output_wrench)= motor_controller.compute_motor_efforts(&motor_transforms_map, &motors_wrench_map, &motor_models_map);
                //IF it's a root motor, add it's wrench to the current vehicle wrench
                //the current wrecnh of the vehicle is the sum of the root motors wrenches
                if motor_controller.get_motor_model().parent_id == 0 {
                    current_global_work += output_wrench;
                }
                motors_wrench_map.insert(*motor_id, output_wrench);
                // row normalisation (theory 3): the column is turned into "acceleration produced per
                // unit of command". Forces / m, moment of axis k / sqrt(m * I_k). Without it the
                // least squares would compare newtons to newton-metres and the arbitration between
                // holding the attitude and holding the force would depend on the choice of length unit.
                let normalized_output_vec= Vector6::new(output_vec[0] / self.vehicle_config.weight,
                                                                                            output_vec[1] / self.vehicle_config.weight, 
                                                                                            output_vec[2] / self.vehicle_config.weight, 
                                                                                            output_vec[3] / f64::sqrt(self.vehicle_config.weight * self.vehicle_config.moments_matrix[(0, 0)]),
                                                                                            output_vec[4] / f64::sqrt(self.vehicle_config.weight * self.vehicle_config.moments_matrix[(1, 1)]),
                                                                                            output_vec[5] / f64::sqrt(self.vehicle_config.weight * self.vehicle_config.moments_matrix[(2, 2)]),);
                effectivness_mat.set_column(i, &Vector6::new(normalized_output_vec[0], normalized_output_vec[1], normalized_output_vec[2], 
                                                                        normalized_output_vec[3], normalized_output_vec[4], normalized_output_vec[5]));
                max_norm= f64::max(max_norm, effectivness_mat.column(i).norm());
            } 
            i+= 1;
        }

        println!("[INFO] -> Current vehicle wrench: {:?}", Vector6::from(current_global_work));
        
        //STEP#3:
        let mut additionnal_works= HashMap::new();
        let mut motor_work_steps= HashMap::new();
        //      per column step (theory 4): 1 / ||a_j||^2 is the EXACT minimiser of the residual
        //      along column j. Because it is per column, the solver becomes invariant to the unit
        //      each motor is commanded in - a single global step would be dictated by the largest
        //      column and would leave the smallest ones almost frozen.
        i= 0;
        for motor_id in processing_order.iter().rev() {
            let mut accum= 0.0;
            for ele in effectivness_mat.column(i).iter() {
                accum += ele * ele;
            }
            let step= 1.0 / accum;
            // RELATIVE guard on top of the exact-zero one: a column that is tiny but not zero (a joint
            // whose child rotor has just started spinning) gives an astronomic step (1e117 observed),
            // and any 1e-17 residual noise then saturates its increment at the trust region. It once
            // flicked the arms by +/-5 deg at every take-off, kicking the roll by ~1 rad/s. Such a
            // motor sits out exactly like an exactly-zero column.
            if step.is_finite() && effectivness_mat.column(i).norm() > 1e-6 * max_norm {
                motor_work_steps.insert(*motor_id, step);
            } else {
                // zero column: this motor has no effect on the wrench at the current operating point
                // (typically a joint whose child produces no thrust). It simply sits out this cycle.
                motor_work_steps.insert(*motor_id, 0.0);
            }
            i+= 1;
        }
        for motor_id in processing_order.iter().rev() {
            if let Some(_) = self.motors.get(motor_id) {
                additionnal_works.insert(*motor_id, 0.0);
            }
        }
        println!("[INFO] -> Wrench Setpoint: {:?}", Vector6::from(setpoint_wrench));
        // the target of the solve is the wrench that is MISSING, not the wrench that is wanted
        // (theory 1): the motors already produce current_global_work, so only the difference has to
        // be allocated. Same row normalisation as the columns - the two sides of A.dx ~= b must be
        // expressed in the same units, otherwise the scores are meaningless.
        let mut delta_wrench= setpoint_wrench - current_global_work;
        delta_wrench.fx /= self.vehicle_config.weight;
        delta_wrench.fy /= self.vehicle_config.weight;
        delta_wrench.fz /= self.vehicle_config.weight;
        delta_wrench.mx /= f64::sqrt(self.vehicle_config.weight * self.vehicle_config.moments_matrix[(0, 0)]);
        delta_wrench.my /= f64::sqrt(self.vehicle_config.weight * self.vehicle_config.moments_matrix[(1, 1)]);
        delta_wrench.mz /= f64::sqrt(self.vehicle_config.weight * self.vehicle_config.moments_matrix[(2, 2)]);
            
        println!("[INFO] -> Execution of BVLS to find the best motor works combinaison to reach the setpoint");
        //if let Ok(base_thrusts)= svd.solve(&Vector6::from(wrench_setpoint), 1e-3 * svd.singular_values.max()) {
            
        let mut prev_wrench_norm= Vector6::from(delta_wrench).norm();
        for s in 0..10 {
            //remaining_wrench = delta_wrench − effectivness_mat * x
            let mut current_commands= DVector::zeros(processing_order.len());
            let mut j= 0;
            //build the motor increment commands vector, used to compute the remaining wrench to reach the setpoint
            for motor_id in processing_order.iter().rev() {
                if let Some(additional_work) = additionnal_works.get(motor_id) {
                    current_commands[j]= *additional_work;
                }
                j+= 1;
            }
            let mut idx= 0;
            let mut wrench_norm= 0.0;
            // one sweep of projected coordinate descent (theory 4). The residual is recomputed
            // INSIDE this loop, so each motor sees what the previous ones have already corrected.
            // Refreshing it only once per sweep would make correlated columns - here the two rotors,
            // which both push mostly along +z - correct the same error twice and overshoot.
            for motor_id in processing_order.iter().rev() {
                let remaining_wrench= Vector6::from(delta_wrench) - &effectivness_mat * &current_commands;
                println!("[INFO] -> Remaining wrench to reach the setpoint: {:?}", remaining_wrench);
                // score of a motor = a_j . r, how much of what is still missing this motor can
                // produce. Sign included: a negative score means the motor must back off.
                let motor_scores= effectivness_mat.transpose() * remaining_wrench;
                if let Some(additional_work) = additionnal_works.get_mut(motor_id) && motor_scores.len() > idx &&
                        let Some(mc) = self.motors.get(motor_id) && let Some(motor_work_step) = motor_work_steps.get(motor_id) {
                    // exact coordinate step: score / ||a_j||^2 is the amount that minimises the
                    // residual along this column alone (theory 4)
                    *additional_work += motor_scores[idx] * motor_work_step;
                    let current_value= f64::clamp(mc.get_motor_feedback().current_value, mc.get_motor_model().min_value, mc.get_motor_model().max_value);
                    // projection on the feasible box (theory 5). Bounds are on the RESULT
                    // current_value + increment, hence the shift by current_value.
                    if mc.get_motor_model().working_axis >= WorkingAxis::LinearMotionALongX as i32 {
                        // thruster: physical bounds only. Its column is exact whatever the increment,
                        // so no trust region is needed. min_value is not zero on purpose: a joint
                        // borrows its authority from its child's thrust (theory 2), so letting a
                        // rotor stop would silently remove the arm from the allocation.
                        *additional_work= additional_work.clamp(mc.get_motor_model().min_value - current_value,
                                                                mc.get_motor_model().max_value - current_value);
                    } else {
                        // angular joint: physical bounds INTERSECTED with the trust region
                        // max_rot_speed * dt, what the joint can actually travel in one cycle.
                        // Its column is a tangent (theory 2): beyond a small angle it stops
                        // describing reality, so the increment must stay small enough for the
                        // linearisation to hold. The joint speed is a hardware spec, not a gain.
                        *additional_work= additional_work.clamp(f64::max(mc.get_motor_model().min_value - current_value, -mc.get_motor_model().max_rot_speed * dt.as_secs_f64()),
                                                                f64::min(mc.get_motor_model().max_value - current_value, mc.get_motor_model().max_rot_speed * dt.as_secs_f64()));
                    }
                    current_commands[idx]= *additional_work
                }
                idx+= 1;
            }
            // convergence is judged on the RELATIVE improvement, never on an absolute threshold:
            // part of the residual is structurally unreachable (theory 6) and never goes away, so
            // the norm converges to that floor, not to zero. When the sweep stops improving, the
            // solver has extracted everything the motors can give.
            wrench_norm= (Vector6::from(delta_wrench) - &effectivness_mat * &current_commands).norm();
            // NOTE: this criterion is measured on the TOTAL residual, which includes the structurally
            // unreachable part (theory 6). As soon as the vehicle is tilted, the unreachable `fy` of the
            // gravity feedforward dominates the norm and never moves, so the relative improvement of the
            // first sweep looks negligible and the solver exits after ONE sweep (vs 9-10 when level),
            // leaving reachable moments under-served. The least-squares optimality condition is
            // A^T r = 0, not r = 0, and the unreachable part lies in the null space of A^T: judging
            // convergence on ||A^T r|| (the `motor_scores` vector, already computed) would remove the
            // unreachable part from the criterion automatically.
            if (prev_wrench_norm - wrench_norm) / prev_wrench_norm < 0.1 {
                break;  
            } else {
                prev_wrench_norm= wrench_norm;
                if s == 9 {
                    println!("[WARNING] -> BVLS did not converge after {} iterations. Remaining wrench norm: {}", s + 1, wrench_norm);
                }
            }
        }
        let now= Timestamp {
            seconds: Utc::now().timestamp(),
            nanos: Utc::now().timestamp_subsec_nanos() as i32,
        };
        for id in processing_order {
            let motor_id= id as u32;
            if let Some(motor_work_step) = motor_work_steps.get(&motor_id) {
                println!("[INFO] -> Motor #{} with work_step= {}: additional work to reach the setpoint: {:?}", motor_id, motor_work_step, additionnal_works.get(&motor_id));
            }
            if let Some(motor_controller) = self.motors.get_mut(&motor_id) && let Some(additional_work) = additionnal_works.get(&motor_id) {
                motor_controller.set_motor_setpoint_value(f64::clamp(motor_controller.get_motor_feedback().current_value + additional_work, motor_controller.get_motor_model().min_value, motor_controller.get_motor_model().max_value));
                println!("[INFO] -> Command for Motor #{}: type= {}, setpoint= {}", motor_id, motor_controller.get_motor_model().working_axis, motor_controller.get_motor_feedback().setpoint_value);
                if motor_controller.get_motor_model().working_axis >= WorkingAxis::LinearMotionALongX as i32 {
                    motors_commands.push(MotorCommand {timestamp: Some(now), id: motor_id, command_type: MotorCommandType::THRUST as i32, setpoint_value: motor_controller.get_motor_feedback().setpoint_value });
                } else {
                    motors_commands.push(MotorCommand {timestamp: Some(now), id: motor_id, command_type: MotorCommandType::ANGULARPOSITION as i32, setpoint_value: motor_controller.get_motor_feedback().setpoint_value });
                }
            }
        } 
        return Some(AnyMessage::MotorCommands(motors_commands));
    }
    
    fn set_setpoint_receiver(&mut self, _receiver: Receiver<Pose>) {
        println!("[MotorsMixer:INFO] -> this Controller don't receive any external setpoints")
    }
}

impl MotorsMixer {
    /// Mixer with a placeholder vehicle (unit mass, zero inertia): only for wiring tests, the row
    /// normalisation divides by the inertia and needs the real config (`new`).
    pub fn new_default(name: String) -> Self {
        return Self { name, vehicle_config: VehicleKinematicConfig { error_linear_factor: 1.0, error_angular_factor: 1.0, error_attitude_factor: 1.0,
                                                                        weight: 1.0, com_relative_location: Vec3::default(), moments_matrix: Matrix3::zeros(),
                                                                    },
                        cmd_sender: None,
                        motors: HashMap::new(), period: Duration::from_millis(0),
                        setpoint: WorkVec { fx: 0.0, fy: 0.0, fz: 0.0, mx: 0.0, my: 0.0, mz: 0.0 },
                        motor_config_receiver: None,
                        motor_feedback_receiver: None,
                    };
    }
    /// Mixer for a given vehicle. The motor tree is not passed here: it arrives through the
    /// configuration channel (`set_motor_config_receiver`) once the hardware / simulator has
    /// discovered it.
    pub fn new(name: String, vehicle_config: VehicleKinematicConfig) -> Self {
        return Self { name, vehicle_config, cmd_sender: None,
                        motors: HashMap::new(), period: Duration::from_millis(0),
                        setpoint: WorkVec { fx: 0.0, fy: 0.0, fz: 0.0, mx: 0.0, my: 0.0, mz: 0.0 },
                        motor_config_receiver: None,
                        motor_feedback_receiver: None,
                    };
    }

    /// Channel on which the vehicle controller publishes the motor tree (one `MotorController`
    /// per motor, ids resolved, parents and children linked).
    pub fn set_motor_config_receiver(&mut self, motor_config_receiver: Receiver<MotorController>) {
        self.motor_config_receiver= Some(motor_config_receiver);
    }

    /// Channel on which the vehicle controller publishes the measured motor state (thrust in N /
    /// angle in rad) - the linearisation point of every solve.
    pub fn set_motor_feedback_receiver(&mut self, motor_feedback_receiver: Receiver<MotorFeedBack>) {
        self.motor_feedback_receiver= Some(motor_feedback_receiver);
    }

    /// Resolve every motor's absolute transform in the body frame.
    /// `motor_ids` MUST be ordered parents-first (depth ascending): a child's transform is expressed
    /// relative to its parent's, so the parent must already be in the map when the child is processed.
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
    /// frame is R_parent * R_j^rel * e_j: it does not depend on the joint's own angle. That is why
    /// `compute_motor_efforts` can read the rotation axis straight from R_j.
    ///
    /// For a root motor (parent_id == 0) the parent is the vehicle itself:
    ///     R_j = R_j^rel * Rot(e_j, theta_j)          p_j = p_j^rel + p_com
    /// the CoM offset being what makes every lever arm below be taken about the reduction point.
    fn compute_motor_transforms(&self, motor_ids : &Vec<u32>) -> HashMap<u32, Transform> {
        let mut motor_transforms: HashMap<u32, Transform>= HashMap::new();
        for motor_id in motor_ids {
            let Some(motor_controller)= self.motors.get(motor_id) else {
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
                        motor_orientation *= UnitQuaternion::from_axis_angle(&working_axis_i32_to_vec3(motor_model.working_axis), motor_controller.get_motor_feedback().current_value);
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
                // ROOT MOTOR: the "parent" is the vehicle itself, so R_parent = identity and
                //             p_parent = p_com, the reduction point every lever arm is taken about.
                // POSITION         p_j = p_j^rel + p_com
                if let Some(m_location)= motor_model.relative_location {
                    // p_j^rel : position of the root motor, given relative to the vehicle origin
                    motor_location= m_location;
                    // + p_com : shifts it to the reduction point, so that p x f below is the moment
                    //           about the CoM and not about the vehicle origin
                    motor_location += self.vehicle_config.com_relative_location;
                }
                // ORIENTATION      R_j = R_j^rel * Rot(e_j, theta_j)
                if let Some(m_orientation)= motor_model.relative_orientation {
                    // R_j^rel : rest orientation of the root motor in the body frame
                    motor_orientation= UnitQuaternion::from(m_orientation);
                }
                // * Rot(e_j, theta_j) : same as above, an angular joint adds its current angle
                if motor_model.working_axis <= WorkingAxis::RotatingAroundZ as i32 {
                    motor_orientation *= UnitQuaternion::from_axis_angle(&working_axis_i32_to_vec3(motor_model.working_axis), motor_controller.get_motor_feedback().current_value);
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

    /// Insert or replace a motor in the table by id (alternative to the configuration channel).
    pub fn add_or_update_motor(&mut self, new_motor: MotorController) {
        let motor_description= new_motor.to_string();

        let motor_id= new_motor.get_motor_model().id;
        if self.motors.insert(motor_id, new_motor).is_none() {
            println!("[INFO] -> Add motor {}", motor_description);
        } else {
            println!("[INFO] -> Motor with id= {} already exist. Motor {} updated", motor_id, motor_description);
        }
    }

    /// Replace the vehicle description (mass, inertia, CoM) used by the row normalisation.
    pub fn set_vehicle_config(&mut self, vehicle_config: VehicleKinematicConfig) {
        self.vehicle_config= vehicle_config;
    }

    pub fn get_motors(&self) -> &HashMap<u32, MotorController> {
        return &self.motors;
    }

    pub fn get_motors_mut(&mut self) -> &mut HashMap<u32, MotorController> {
        return &mut self.motors;
    }
}


