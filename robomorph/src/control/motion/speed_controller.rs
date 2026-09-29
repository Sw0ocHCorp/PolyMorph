use std::time::Duration;
use tokio::sync::broadcast::{Receiver, Sender};
use crate::{control::{motion::motion_controller::MotionController, pid_controller::PIDController}, core::scheduler::Process, messages::{pose_messages::Pose, registered_message::{AnyMessage::{self, ForceVec}, Vec3}}};

pub struct SpeedController {
    name: String,
    vehicle_weight: f64,
    period: Duration,
    setpoint: Pose,
    setpoint_receiver: Option<Receiver<Pose>>,
    telemetry_receiver: Option<Receiver<AnyMessage>>,
    linear_force_sender: Option<Sender<AnyMessage>>,
    accel_pid: PIDController<Vec3>,
}

impl Process for SpeedController {
    fn set_name(&mut self, name: String) {
        self.name = name;
    }

    fn exec(&mut self, input: &Option<crate::messages::registered_message::AnyMessage>, dt: std::time::Duration) -> Option<crate::messages::registered_message::AnyMessage> {
        let mut current_pose= Pose::default();
        if let Some(current_pose_rcvr)= &mut self.telemetry_receiver {
            let sz= current_pose_rcvr.len();
            for _ in 0..sz {
                if let Ok(pose_msg)= current_pose_rcvr.try_recv() && 
                        let AnyMessage::PoseState(pose) = pose_msg {
                    current_pose= pose;
                }
            }
        }
        if let Some(setpoint)= input && let AnyMessage::PoseState(setpoint_pose)= setpoint {
            self.setpoint= setpoint_pose.clone();
        } 
        else if let Some(setpoint_rcvr) = self.setpoint_receiver.as_mut() {
            for _ in 0..setpoint_rcvr.len() {
                if let Ok(setpoint)= setpoint_rcvr.try_recv() {
                    self.setpoint= setpoint;
                }
            }
        }
        let output= self.compute_command_law(Some(AnyMessage::PoseState(current_pose)), Some(AnyMessage::PoseState(self.setpoint.clone())), dt, true);
        if let Some(res) = output && let AnyMessage::ForceVec(linear_force)= res {
            if let Some(sender)= &mut self.linear_force_sender && sender.receiver_count() > 0 {
                let _= sender.send(AnyMessage::ForceVec(linear_force.clone()));
                return None;
            } else {
                return Some(AnyMessage::ForceVec(linear_force.clone()));
            }
        }
        return None;
    }

    fn set_receiver(&mut self, receiver: tokio::sync::broadcast::Receiver<AnyMessage>) {
        self.telemetry_receiver= Some(receiver);
    }

    fn set_sender(&mut self, sender: tokio::sync::broadcast::Sender<AnyMessage>) {
        self.linear_force_sender= Some(sender);
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

impl MotionController for SpeedController {
    fn compute_command_law(&mut self, input_data: Option<AnyMessage>, setpoint: Option<AnyMessage>, dt: Duration, verbose: bool) -> Option<AnyMessage> {
        if let Some(input) = input_data && let AnyMessage::PoseState(current_pose)= input &&
                let Some(target) = setpoint && let AnyMessage::PoseState(setpoint_pose)= target && 
                let Some(current_l_vel)= current_pose.l_velocity && let Some(mut setpoint_l_vel)= setpoint_pose.l_velocity &&
                                                                    let Some(current_orientation)= current_pose.orientation  {
            if verbose {
                println!("[SpeedController::INFO] -> current_pose: {:?} && setpoint_pose: {:?}", current_pose, setpoint_pose);
            }
            let mut rotated_gravity_vec= Vec3::new(0.0, 0.0, -self.vehicle_weight * 9.81);
            rotated_gravity_vec= current_orientation * rotated_gravity_vec;
            setpoint_l_vel.x-= rotated_gravity_vec.x / self.vehicle_weight;
            setpoint_l_vel.y-= rotated_gravity_vec.y / self.vehicle_weight;
            setpoint_l_vel.z-= rotated_gravity_vec.z / self.vehicle_weight;
            let accel_output= self.accel_pid.compute_output(current_l_vel, setpoint_l_vel, dt);
            let force_setpoint= Some(ForceVec(Vec3::new(self.vehicle_weight * accel_output.x + rotated_gravity_vec.x, 
                                            self.vehicle_weight * accel_output.y + rotated_gravity_vec.y, 
                                                self.vehicle_weight * accel_output.z + rotated_gravity_vec.z)));
            println!("[SpeedController::INFO] -> accel_output: {:?} && force_setpoint: {:?}", accel_output, Vec3::new(self.vehicle_weight * accel_output.x + rotated_gravity_vec.x, 
                                            self.vehicle_weight * accel_output.y + rotated_gravity_vec.y, 
                                                self.vehicle_weight * accel_output.z + rotated_gravity_vec.z));
            return force_setpoint;
        }
        
        //let accel_output= self.accel_pid.
        return None;
    }

    fn set_setpoint_receiver(&mut self, receiver: Receiver<Pose>) {
        self.setpoint_receiver= Some(receiver);
    }
}

impl SpeedController {
    pub fn new(name: String, vehicle_weight: f64, accel_pid: PIDController<Vec3>) -> Self {
        return Self {
            name,
            vehicle_weight,
            period: Duration::from_millis(10),
            setpoint: Pose::default(),
            setpoint_receiver: None,
            telemetry_receiver: None,
            linear_force_sender: None,
            accel_pid,
        };
    }
    
}