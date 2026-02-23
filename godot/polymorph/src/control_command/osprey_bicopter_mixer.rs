use godot::global::godot_print;
use robomorph::{actuators::mixer_model::MixerModel, control::{self, pid::PIDController}, core::utils::{self, modulo_2pi}, positionning::pose::Pose};

const WINGL_ANGLE_OFFSET: f64= 90.0;
const WINGR_ANGLE_OFFSET: f64= -90.0;

pub struct OspreyBicopterMixer {
    wings_command: [f64; 2],
    thrusters_command: [f64; 2],
    //Wings PIDs will control the rotation speed of the wings to reach the setpoint
    wing_pids: [PIDController; 2],
    //Thruster PIDs will control the rotation speed to move the drone
    thruster_pids: [PIDController; 2],
    dist_threshold: f64,
    drone_pose: Pose
}

impl OspreyBicopterMixer {
    pub fn new(init_pose: Pose, wing_pid_params: [f64; 3], wing_max_angle_error: f64, wing_angle_threshold: f64, thruster_pid_params: [f64; 3], thruster_max_speed_error: f64, dist_threshold: f64) -> Self {
        let wing_pid= PIDController::new(wing_pid_params[0], wing_pid_params[1], wing_pid_params[2], wing_max_angle_error, wing_angle_threshold, 0.99);
        let wing_pid_copy= control::pid::copy(&wing_pid);
        let thruster_pid= PIDController::new(thruster_pid_params[0], thruster_pid_params[1], thruster_pid_params[2], thruster_max_speed_error, dist_threshold, 0.99);
        let thruster_pid_copy= control::pid::copy(&thruster_pid);
        let wings_pids= [wing_pid, wing_pid_copy];
        let thrusters_pids= [thruster_pid, thruster_pid_copy];
        return Self {drone_pose: init_pose, wings_command: [0.0, 0.0], thrusters_command: [0.0, 0.0], wing_pids: wings_pids, thruster_pids: thrusters_pids, dist_threshold: dist_threshold };
    }
}

impl Default for OspreyBicopterMixer {
    fn default() -> Self {
        Self { wings_command: Default::default(), thrusters_command: Default::default(), 
            wing_pids: [PIDController::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), 
                            PIDController::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)], 
            thruster_pids: [PIDController::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), 
                                PIDController::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)], dist_threshold: Default::default(), drone_pose: Default::default() }
    }
}

impl MixerModel for OspreyBicopterMixer {
    //Apply the command law to the system based on the system dynamics
    //  State is composed of the orientations of the Wings and the location of the robot as follow: 
    //      WingLAngle, WingRAngle, LocX, LocY, LocZ
    //  Setpoint is the Waypoint location to target:
    //      WayPX, WayPY, WayPZ
    fn apply_command_law(&mut self, current_state: Vec<f64>, setpoint: Vec<f64>, dt: f64) -> Vec<f64> {
        let mut commands= Vec::new();
        let current_orientation= self.drone_pose.get_euler_orientation();
        let mut motion_direction= [0.0, 0.0, 0.0];
        if utils::euclidean_distance(&current_orientation.to_vec(), &setpoint) > self.dist_threshold {
            motion_direction= [setpoint[0] - current_orientation[0], setpoint[1] - current_orientation[1], setpoint[2] - current_orientation[2]]
        }
        motion_direction= utils::compute_direction_vector(motion_direction);
        let wing_angle_setpoint= f64::atan2(motion_direction[1], motion_direction[0]).to_degrees();
        for j in 0..2 {
            if j == 0{
                //apply - to the setpoint because the wing is 180 flip around the Y axis
                self.wings_command[j] = self.wing_pids[j].compute_output_value(-wing_angle_setpoint + WINGL_ANGLE_OFFSET, current_state[j], dt);
            } else {
                //godot_print!("Right wing:\n    current angle= {} | setpoint angle= {}\n    angle error= {}", current_state[j], wing_angle_setpoint + WINGR_ANGLE_OFFSET, wing_angle_setpoint + WINGR_ANGLE_OFFSET - current_state[j]);
                self.wings_command[j] = self.wing_pids[j].compute_output_value(wing_angle_setpoint + WINGR_ANGLE_OFFSET, current_state[j], dt);
            }
            commands.push(self.wings_command[j]);
        }
        
        return commands;
    }
}