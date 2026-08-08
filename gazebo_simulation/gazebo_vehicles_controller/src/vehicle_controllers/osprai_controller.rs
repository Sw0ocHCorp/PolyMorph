use std::{collections::HashMap, f64::consts::PI, sync::{atomic::{AtomicBool, Ordering}, Arc, Mutex}, time::Duration};

use chrono::Utc;
use gz::{msgs::{actuators::Actuators, empty::Empty, image::Image, imu::IMU, laserscan::LaserScan, model::Model, navsat::NavSat, odometry::Odometry, scene::Scene}, transport::{Node, Publisher}};
use nalgebra::{UnitQuaternion, Vector3};
use robomorph::{communications::interface::HardwareInterface, control::{motion::{motion_controller::{MotionController, VehicleKinematicConfig}, motor_controller::{MotorController, quaternion_to_euler, working_axis_to_vec3}, motors_mixer::MotorsMixer}, pid_controller::PIDController}, core::scheduler::Process, messages::{lidar_messages::{LidarMeasurements, Ray}, motor_messages::{MotorCommandType, MotorFeedBack, MotorModel, MotorStatus, PIDConfig, WorkingAxis}, pose_messages::{GNSSMeasurement, IMUMeasurements, Pose}, registered_message::{AnyMessage, UnitQuat, Vec3}}};
use tokio::sync::{broadcast, mpsc};

use crate::vehicle_controllers::vehicle_controller::VehicleController;

pub fn gz_quaternion_to_euler(quat: &gz::msgs::quaternion::Quaternion) -> [f64; 3] {
    let w = quat.w;
    let x = quat.x;
    let y = quat.y;
    let z = quat.z;

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

/// Struct that represent the camera sensor specs and the pixels in the image
pub struct ImageState {
    pub timestamp: i64,
    pub cam_label: String,
    pub width: u32,
    pub height: u32,
    pub channel: u8,
    pub format: String,
    pub data: Vec<u8>,
}

/// Struct that represent the telemetry telemetry_state of the osprai vehicle
pub struct OsprAiState {
    /// Lidar sensor specs (vertical / horizontal range, ray max range, etc...) & measurements
    pub lidar_measurements: LidarMeasurements,
    pub pose_state: Pose,
    /// Camera sources specs and image
    pub cameras: Vec<ImageState>,
}

impl OsprAiState {
    pub fn new() -> Self {
        return OsprAiState {
            pose_state: Pose { gnss_measurement: Some(GNSSMeasurement { latitude: 0.0, longitude: 0.0, altitude: 0.0, fix_status: 1 }), 
                                relative_location: Some(Vec3::default()), 
                                orientation: Some(UnitQuat { w: 1.0, x: 0.0, y: 0.0, z: 0.0 }), 
                                imu_measurement: Some(IMUMeasurements::default()), l_velocity: Some(Vec3::default())},
            lidar_measurements: LidarMeasurements {
                vertical_min_angle: 0.0, vertical_angle_resolution: 0.0, vertical_max_angle: 0.0,
                horizontal_min_angle: 0.0, horizontal_angle_resolution: 0.0, horizontal_max_angle: 0.0,
                rays: vec![],
            },
            cameras: vec![
                ImageState { timestamp: 0, cam_label: "camera_left".to_string(), width: 0, height: 0, channel: 0, format: "rgb".to_string(), data: vec![] },
                ImageState { timestamp: 0, cam_label: "camera_right".to_string(), width: 0, height: 0, channel: 0, format: "rgb".to_string(), data: vec![] },
            ],
        };
    }

    pub fn new_with_lidar() -> Self {
        return OsprAiState {
            pose_state: Pose { gnss_measurement: Some(GNSSMeasurement { latitude: 0.0, longitude: 0.0, altitude: 0.0, fix_status: 1 }), 
                                relative_location: Some(Vec3::default()), 
                                // identity, NOT UnitQuat::default(): the wire default is (0,0,0,0), whose normalisation
                                // divides by a zero norm and yields NaN. Until the first IMU message arrives this
                                // value is used as the measured attitude, and a NaN here poisons the whole chain.
                                orientation: Some(UnitQuat { w: 1.0, x: 0.0, y: 0.0, z: 0.0 }), 
                                imu_measurement: Some(IMUMeasurements::default()), l_velocity: Some(Vec3::default())},
            lidar_measurements: LidarMeasurements {
                vertical_min_angle: 0.0, vertical_angle_resolution: 0.0, vertical_max_angle: 0.0,
                horizontal_min_angle: 0.0, horizontal_angle_resolution: 0.0, horizontal_max_angle: 0.0,
                rays: vec![],
            },
            cameras: vec![
                ImageState { timestamp: 0, cam_label: "camera_left".to_string(), width: 0, height: 0, channel: 0, format: "rgb".to_string(), data: vec![] },
                ImageState { timestamp: 0, cam_label: "camera_right".to_string(), width: 0, height: 0, channel: 0, format: "rgb".to_string(), data: vec![] },
            ],
        };
    }
}

/// Struct that represent the controller's of the osprai vehicle
/// 
/// can send telemetry data to an Hardware Interface to communicate with other softwares, receive command frames from the hardware interface
/// 
/// can apply commands on the hardware actuators or Gazebo simulated vehicle
pub struct OspraiController {
    name: String,
    /// Telemetry state that aggregate all the telemtry data
    pub telemetry_state: Arc<Mutex<OsprAiState>>,
    /// The node that allow the controller to connect itself to Gazebo simulation environment
    controller_node: Option<Node>,
    /// Receiver of the cmd from Hardware Interface
    interface_cmd_receiver: Option<broadcast::Receiver<AnyMessage>>,
    /// Telemetry sender to Hardware Interface
    telemetry_sender: Option<mpsc::Sender<AnyMessage>>,
    controller: Arc<Mutex<MotorsMixer>>,
    motors_command_sequence: Vec<u8>,
    motors_command_publisher: Option<Publisher<Actuators>>,
    /// Shared with the Gazebo subscription callbacks, which run outside of `&self`
    motors_initialized: Arc<AtomicBool>,
}

impl OspraiController {
    pub fn new(name: String, cmd_receiver: broadcast::Receiver<AnyMessage>, vehicle_config: VehicleKinematicConfig, pid: PIDController) -> Self {
        return Self {name, telemetry_state: Arc::new(Mutex::new(OsprAiState::new())),
                controller_node: None,
                interface_cmd_receiver: Some(cmd_receiver),
                telemetry_sender: None,
                controller: Arc::new(
                    Mutex::new(
                        MotorsMixer::new("osprai_controller".to_string(), vehicle_config, pid),
                    )
                ),
                motors_command_publisher: None,
                motors_command_sequence: vec![0, 0, 0, 0],
                motors_initialized: Arc::new(AtomicBool::new(false)),
        };
    }
    /// Constructor
    /// 
    /// Arguments:
    /// 
    /// cmd_receiver: A vehicle commands receiver
    pub fn new_with_lidar(name: String, cmd_receiver: broadcast::Receiver<AnyMessage>, vehicle_config: VehicleKinematicConfig, pid: PIDController) -> Self {
        return Self {name, telemetry_state: Arc::new(Mutex::new(OsprAiState::new_with_lidar())),
                controller_node: None,
                interface_cmd_receiver: Some(cmd_receiver),
                telemetry_sender: None,
                controller: Arc::new(
                    Mutex::new(
                        MotorsMixer::new("osprai_controller".to_string(), vehicle_config, pid),
                    )
                ),
                motors_command_publisher: None,
                motors_command_sequence: vec![0, 0, 0, 0],
                motors_initialized: Arc::new(AtomicBool::new(false)),
        };
    }

    pub fn set_vehicle_params(&mut self, vehicle_config: VehicleKinematicConfig) {
        if let Ok(mut controller)= self.controller.lock() {
            controller.set_vehicle_config(vehicle_config);
        }
    }
}

impl Default for OspraiController {
    /// Default constructor
    fn default() -> Self {
        Self { name: String::new(), telemetry_state: Arc::new(Mutex::new(OsprAiState::new())),
                controller_node: None,
                interface_cmd_receiver: None,
                telemetry_sender: None,
                controller: Arc::new(Mutex::new(MotorsMixer::new_default("osprai_controller".to_string()))),
                motors_command_publisher: None,
                motors_command_sequence: vec![0, 0, 0, 0],
                motors_initialized: Arc::new(AtomicBool::new(false)),
        }
    }
}

impl VehicleController for OspraiController {
    /// Connect the simulation node to all the required topics to be able to control the vehicle
    /// 
    /// Notes:
    /// 
    /// Controller must be to be mutable because the topics will update the state of the vehicle
    fn start_listening_topics(&mut self) {
        self.controller_node= Node::new();
        if let Some(node)=  &mut self.controller_node {
            self.motors_command_publisher = node.advertise::<Actuators>("/osprai/command/motor_setpoints");
            let _= node.subscribe("/model/osprai/odometry", move |msg: Odometry| {
                println!("[INFO] -> True pose= {:?}", msg.pose);
            });
            let osprai_state_imu= self.telemetry_state.clone();
            let _= node.subscribe("osprai/imu", move |msg : IMU| {
                if let Ok(mut telemetry_state) = osprai_state_imu.clone().lock() &&
                        let Some(imu)= telemetry_state.pose_state.imu_measurement.as_mut() {
                    imu.l_accel= Some(Vec3::new(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z));
                    imu.a_velocity= Some(Vec3::new(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z));
                    telemetry_state.pose_state.orientation= Some(UnitQuat { w: msg.orientation.w, x: msg.orientation.x, y: msg.orientation.y, z: msg.orientation.z });
                    if let Some(quat)= telemetry_state.pose_state.orientation {
                        let euler= quaternion_to_euler(&UnitQuaternion::from(quat));
                        println!("IMU orientation: roll: {}, pitch: {}, yaw: {}", euler[0], euler[1], euler[2]);
                    }
                }
                
            });
            let osprai_state_gnss= self.telemetry_state.clone();
            let _= node.subscribe("osprai/gps", move |msg : NavSat| {
                if let Ok(mut telemetry_state) = osprai_state_gnss.clone().lock() && 
                        let Some(gnss)= telemetry_state.pose_state.gnss_measurement.as_mut() {
                            *gnss= GNSSMeasurement { longitude: msg.longitude_deg, latitude: msg.latitude_deg,
                                                            altitude: msg.altitude, fix_status: 1 };
                }
            });
            let osprai_state_lidar= self.telemetry_state.clone();
            let _= node.subscribe("osprai/lidar", move |msg: LaserScan| {
                if let Ok(mut telemetry_state) = osprai_state_lidar.clone().lock() {
                    telemetry_state.lidar_measurements.rays.clear();

                    telemetry_state.lidar_measurements.vertical_min_angle = msg.vertical_angle_min;
                    telemetry_state.lidar_measurements.vertical_angle_resolution = msg.vertical_angle_step;
                    telemetry_state.lidar_measurements.vertical_max_angle = msg.vertical_angle_max;
                    telemetry_state.lidar_measurements.horizontal_min_angle = msg.angle_min;
                    telemetry_state.lidar_measurements.horizontal_angle_resolution = msg.angle_step;
                    telemetry_state.lidar_measurements.horizontal_max_angle = msg.angle_max;
                    let vertical_angle= telemetry_state.lidar_measurements.vertical_min_angle;
                    let mut offset_idx= 0;
                    for i in 0..msg.ranges.len() {
                        let mut current_angle= telemetry_state.lidar_measurements.horizontal_min_angle + ((i - offset_idx) as f64)*telemetry_state.lidar_measurements.horizontal_angle_resolution;
                        if current_angle > telemetry_state.lidar_measurements.horizontal_max_angle {
                            offset_idx= i;
                            current_angle= 0.0;
                        }
                        telemetry_state.lidar_measurements.rays.push(Ray{vertical_angle: vertical_angle, horizontal_angle: current_angle, length: msg.ranges[i]});
                    }
                }
            });
            let osprai_state_lc= self.telemetry_state.clone();
            let _= node.subscribe("osprai/cameras/left", move |msg: Image| {
                if let Ok(mut telemetry_state) = osprai_state_lc.clone().lock() {
                    telemetry_state.cameras[0]= ImageState { timestamp: Utc::now().timestamp_millis(), cam_label: "camera_left".to_string(), 
                                                    width: msg.width, height: msg.height, channel: 3, format: "rgb".to_string(), data: msg.data.clone() };
                }
            });
            let osprai_state_rc= self.telemetry_state.clone();
            let _= node.subscribe("osprai/cameras/right", move |msg: Image| {
                if let Ok(mut telemetry_state) = osprai_state_rc.clone().lock() {
                    telemetry_state.cameras[0]= ImageState { timestamp: Utc::now().timestamp_millis(), cam_label: "camera_right".to_string(), 
                                                    width: msg.width, height: msg.height, channel: 3, format: "rgb".to_string(), data: msg.data.clone() };
                }
            });

            let controller_cl= self.controller.clone();
            if let Some((scene, true)) = node.request::<Empty, Scene>(
                "/world/my_environment/scene/info",
                &Empty::default(),
                Duration::from_secs(2),
            ) {
                for model in &scene.model {
                    if model.name == "osprai" {
                        for link in &model.link {
                            if let Ok(controller) = controller_cl.lock().as_mut() {
                                let link_id: u8= link.name.bytes().fold(0u8, |acc, b| acc.wrapping_add(b));

                                if link.name.contains("rotor_left") {
                                    self.motors_command_sequence[0]= link_id;
                                } else if link.name.contains("rotor_right") {
                                    self.motors_command_sequence[1]= link_id;
                                } else if link.name.contains("arm_left") {
                                    self.motors_command_sequence[2]= link_id;
                                } else if link.name.contains("arm_right") {
                                    self.motors_command_sequence[3]= link_id;
                                }
                                if link.name.contains("arm"){
                                    let motor_model= MotorModel {
                                        id: link_id as u32,
                                        parent_id: 0,
                                        child_ids: Vec::new(),
                                        relative_location:  Some(Vec3::new(link.pose.position.x, 
                                                                        link.pose.position.y, 
                                                                        link.pose.position.z)),
                                        working_axis: WorkingAxis::RotatingAroundY as i32,
                                        min_value: 0.0,
                                        max_value: 0.0,
                                        relative_orientation: Some(UnitQuat::from(UnitQuaternion::from_axis_angle(&working_axis_to_vec3(WorkingAxis::RotatingAroundY), 0.0))),
                                        pid_config: Some(PIDConfig { p: 0.2, i: 1.0, d: 0.0, min_correction_error: 0.0, max_error_accum: PI }),
                                        exp_command_law: 1,
                                        max_rot_speed: 0.0,
                                        transmission_factor: 1.0,
                                        effort_constant: 1.2e-5,
                                        moment_constant: 0.016,
                                        motor_kv: 0.0
                                    };
                                    let motor_feedback= MotorFeedBack {
                                        status: MotorStatus::IDLE as i32,
                                        command_type: MotorCommandType::ANGULARPOSITION as i32,
                                        current_value: 0.0,
                                        setpoint_value: 0.0,
                                        pid_config: Some(PIDConfig { p: 1.0, i: 0.0, d: 0.0, 
                                                            min_correction_error: 0.0, max_error_accum: PI}),
                                        control_frequency: 50,
                                    };
                                    let pid= PIDController::new(0.2, 1.0, 0.0, 0.0, PI);

                                    controller.add_or_update_motor(MotorController::new(motor_model, motor_feedback, pid));
                                } else if link.name.contains("rotor") {
                                    let mut moment_constant= 0.016;
                                    if link.name.contains("right") {
                                        moment_constant= -0.016;
                                    }
                                    let motor_model= MotorModel {
                                        id: link_id as u32,
                                        parent_id: 0,
                                        child_ids: Vec::new(),
                                        relative_location:  Some(Vec3::new(link.pose.position.x, 
                                                                        link.pose.position.y, 
                                                                        link.pose.position.z)),
                                        working_axis: WorkingAxis::LinearMotionALongZ as i32,
                                        min_value: 0.0,
                                        max_value: 0.0,
                                        relative_orientation: Some(UnitQuat::from(UnitQuaternion::from_axis_angle(&working_axis_to_vec3(WorkingAxis::RotatingAroundY), 0.0))),
                                        pid_config: Some(PIDConfig { p: 0.2, i: 1.0, d: 0.0, min_correction_error: 0.0, max_error_accum: PI }),
                                        exp_command_law: 2,
                                        max_rot_speed: 1200.0,
                                        transmission_factor: 1.0,
                                        effort_constant: 1.2e-5,
                                        moment_constant: moment_constant,
                                        motor_kv: 1200.0
                                    };
                                    let motor_feedback= MotorFeedBack {
                                        status: MotorStatus::IDLE as i32,
                                        command_type: MotorCommandType::THRUST as i32,
                                        current_value: 0.0,
                                        setpoint_value: 0.0,
                                        pid_config: Some(PIDConfig { p: 1.0, i: 0.0, d: 0.0, 
                                                            min_correction_error: 0.0, max_error_accum: f64::INFINITY}),
                                        control_frequency: 50,
                                    };
                                    let pid= PIDController::new(1.0, 0.0, 0.0, 0.0, f64::INFINITY);

                                    controller.add_or_update_motor(MotorController::new(motor_model, motor_feedback, pid));
                                }
                            }
                            println!("{} pose: {:?}", link.name, link.pose);
                        }
                        println!("Model {} pose: {:?}", model.name, model.pose);
                    }
                }
            }

            let controller_cl= self.controller.clone();
            let motors_initialized_cl= self.motors_initialized.clone();
            let _= node.subscribe("/osprai/joint_state", move |msg: Model| {
                for j in msg.joint {
                    if let Ok(controller) = controller_cl.lock().as_mut() {
                        let mut parent_id: u8= j.parent.bytes().fold(0u8, |acc, b| acc.wrapping_add(b));
                        let mut name= j.name.clone();
                        let mut id= 0;
                        let mut relative_location= Vec3::default();
                        let mut is_initialized= true;
                        if j.name.contains("tilt") {
                            name= name.replace("tilt", "arm");
                            id= name.bytes().fold(0u8, |acc, b| acc.wrapping_add(b));
                            if let Some(motor_controller)= controller.get_motors_mut().get_mut(&(id as u32)) {
                                let motor_model= motor_controller.get_motor_model();
                                if motor_model.max_value != j.axis1.limit_upper {
                                    if let Some(relative_loc)= motor_model.relative_location {
                                        relative_location= relative_loc;
                                    }
                                    else {
                                        relative_location= Vec3::new(0.0, 0.0, 0.0);
                                    }
                                    motor_controller.set_parent_id(parent_id as u32);
                                    motor_controller.set_motor_max_value(j.axis1.limit_upper);
                                    motor_controller.set_motor_min_value(j.axis1.limit_lower);
                                    motor_controller.set_max_rot_speed(10.0);
                                    is_initialized= false;
                                }
                            }
                        } else if j.name.contains("spin") {
                            name= name.replace("spin", "rotor");
                            id= name.bytes().fold(0u8, |acc, b| acc.wrapping_add(b));
                            if let Some(motor_controller)= controller.get_motors_mut().get_mut(&(id as u32)) {
                                let motor_model= motor_controller.get_motor_model();
                                let (transmission_factor, effort_constant, exp_command_law)= (motor_model.transmission_factor, motor_model.effort_constant ,motor_model.exp_command_law);
                                if motor_model.max_value == 0.0 {
                                    if let Some(relative_loc)= motor_controller.get_motor_model().relative_location {
                                        relative_location= relative_loc;
                                    }
                                    else {
                                        relative_location= Vec3::new(0.0, 0.0, 0.0);
                                    }
                                    motor_controller.set_parent_id(parent_id as u32);
                                    motor_controller.set_motor_min_value(0.1);
                                    let max_thrust= transmission_factor * effort_constant * 
                                                    f64::powf(1200.0, exp_command_law as f64);
                                    motor_controller.set_motor_max_value(max_thrust);
                                    is_initialized= false;
                                }
                            }
                        }
                        if is_initialized == false && motors_initialized_cl.load(Ordering::Relaxed) == false {
                            if let Some(parent_motor_controller)= controller.get_motors_mut().get_mut(&(parent_id as u32)) {
                                    let parent_model= parent_motor_controller.get_motor_model();
                                    let mut parent_relative_location= Vec3::new(0.0, 0.0, 0.0);
                                    if let Some(relative_loc)= parent_model.relative_location {
                                        parent_relative_location= relative_loc;
                                    }
                                    relative_location -= parent_relative_location;
                                    
                                    parent_motor_controller.add_child(id as u32);
                                    
                            } else {
                                if let Some(motor_controller)= controller.get_motors_mut().get_mut(&(id as u32)) {
                                    motor_controller.set_parent_id(0);
                                }
                            }
                            if let Some(motor_controller)= controller.get_motors_mut().get_mut(&(id as u32)) {
                                motor_controller.set_motor_relative_location(Vec3::new(relative_location.x, relative_location.y, relative_location.z));
                            }
                        }
                        if let Some(motor_controller)= controller.get_motors_mut().get_mut(&(id as u32)) {
                            
                            if motor_controller.get_working_axis() as i32 >= WorkingAxis::LinearMotionALongX as i32 && motor_controller.get_working_axis() as i32 <= WorkingAxis::LinearMotionALongZ as i32 {
                                println!("Received motor measurements: id: {}, name: {}, type: {}, value: {}", id, name, motor_controller.get_working_axis() as i32, j.axis1.velocity);
                                let motor_model= motor_controller.get_motor_model();
                                let current_thrust= motor_model.transmission_factor * motor_model.effort_constant * 
                                                    j.axis1.velocity.powf(motor_model.exp_command_law as f64);
                                motor_controller.set_motor_current_value(current_thrust);
                            }
                            else if motor_controller.get_working_axis() as i32 >= WorkingAxis::RotatingAroundX as i32 && motor_controller.get_working_axis() as i32 <= WorkingAxis::RotatingAroundZ as i32 {
                                println!("Received motor measurements: id: {}, name: {}, type: {}, value: {}", id, name, motor_controller.get_working_axis() as i32, j.axis1.position);
                                motor_controller.set_motor_current_value(j.axis1.position);
                            }
                        }
                    }
                }
                motors_initialized_cl.store(true, Ordering::Relaxed);
            });
        }
    }

    /// Apply the setpoints values for all the actuators
    /// 
    /// Arguments:
    /// 
    /// setpoints: list of the setpoints to apply
    /// 
    /// Notes:
    /// 
    /// Not used in this implementation because setpoint to apply lived in the motor states in the vehicle state
    fn apply_actuator_setpoints(&mut self, _: Vec<AnyMessage>, dt: Duration) {
        let mut current_orientation= UnitQuat { w: 1.0, x: 0.0, y: 0.0, z: 0.0 };
        let mut imu_measurements= IMUMeasurements { l_accel: Some(Vec3::new(0.0, 0.0, 0.0)), 
                                                                    a_velocity: Some(Vec3::new(0.0, 0.0, 0.0)), 
                                                                    magnetic_field: Some(Vec3::new(0.0, 0.0, 0.0))
                                                                };
        if let Ok(state)= self.telemetry_state.lock() {
            if let Some(orientation) = state.pose_state.orientation && 
                    let Some(imu)= state.pose_state.imu_measurement.clone() {
                current_orientation= orientation;
                imu_measurements= imu;
            }
        } 
        let current_pose= AnyMessage::PoseState(Pose { relative_location: Some(Vec3::new(0.0, 0.0, 0.0)),
                                                                    gnss_measurement: Some(GNSSMeasurement { longitude: 0.0, 
                                                                                                                latitude: 0.0, 
                                                                                                                altitude: 0.0, 
                                                                                                                fix_status: 0, 
                                                                    }), 
                                                                    imu_measurement: Some(imu_measurements),
                                                                    orientation: Some(current_orientation),
                                                                    l_velocity: Some(Vec3::new(0.0, 0.0, 0.0)) });
        
        let pose_setpoint= AnyMessage::PoseState(Pose { relative_location: Some(Vec3::new(0.0, 0.0, 0.0)),
                                                                    gnss_measurement: Some(GNSSMeasurement { longitude: 0.0, 
                                                                                                                latitude: 0.0, 
                                                                                                                altitude: 0.0, 
                                                                                                                fix_status: 0, 
                                                                    }), 
                                                                    imu_measurement: Some(IMUMeasurements { l_accel: Some(Vec3::new(0.0, 0.0, 0.0)), 
                                                                                                                a_velocity: Some(Vec3::new(0.0, 0.0, 0.0)), 
                                                                                                                magnetic_field: Some(Vec3::new(0.0, 0.0, 0.0)) 
                                                                    }),
                                                                    orientation: Some(UnitQuat::from(UnitQuaternion::from_euler_angles(f64::to_radians(-5.0), 0.0, 0.0))),
                                                                    l_velocity: Some(Vec3::new(0.0, 0.0, 0.5)) });
        if let Ok(mut controller)= self.controller.lock() && 
                let Some(motor_pub)= self.motors_command_publisher.as_mut() &&
                let AnyMessage::MotorCommands(motors_command)= controller.compute_command_law( current_pose, pose_setpoint, dt) &&
                motors_command.len() > 0 {
            let mut actuator_setpoints= Actuators::new();
            for id in self.motors_command_sequence.clone() {
                for cmd in motors_command.clone() {
                    if cmd.id == id as u32 && let Some(motor_controller)= controller.get_motors().get(&cmd.id) {
                        // The controller works in EFFORT units: newtons for a thruster, radians for a joint.
                        // Gazebo speaks the actuator's own unit, so the conversion happens here, at the
                        // hardware boundary, and only for the motors that need it.
                        if cmd.command_type == MotorCommandType::THRUST as i32 {
                            // thrust -> rotor speed: the MulticopterMotorModel is driven in rad/s
                            //      T = transmission * effort_constant * w^exp   =>   w = (T / (transmission * effort_constant))^(1/exp)
                            let motor_model= motor_controller.get_motor_model();
                            let setpoint_rot_speed= (cmd.setpoint_value / (motor_model.transmission_factor * motor_model.effort_constant)).powf(1.0 / motor_model.exp_command_law as f64);
                            actuator_setpoints.velocity.push(setpoint_rot_speed);
                        } else {
                            // a position-controlled joint is already commanded in radians: no conversion
                            actuator_setpoints.position.push(cmd.setpoint_value);
                        }
                    }
                }
            }
            // NEVER publish a non finite command. Gazebo rejects it ("Invalid joint velocity value [nan]")
            // but the MulticopterMotorModel keeps it in its first order filter state, which stays NaN
            // for the rest of the run: the rotor never spins again and the whole chain reads zero thrust.
            // Same rule applies to real hardware: an ESC fed garbage does not come back on its own.
            if actuator_setpoints.velocity.iter().chain(actuator_setpoints.position.iter()).any(|v| !v.is_finite()) {
                println!("[ERROR] -> Non finite actuator setpoint, command dropped: {:?}", actuator_setpoints);
                return;
            }
            println!("Publishing actuator setpoints: {:?}", actuator_setpoints);
            let _= motor_pub.publish(&actuator_setpoints);
        }
    }


    /// Send telemetry data to the telemetry receiver linked to telemetry_sender
    /// 
    /// Most likely to an interface
    fn send_telemetry(&mut self) {
        if let Some(sender) = &mut self.telemetry_sender &&
            let Ok(telemetry_state)= self.telemetry_state.lock() {
            if let Some(imu)= &telemetry_state.pose_state.imu_measurement {
                let _ = sender.try_send(AnyMessage::ImuState(imu.clone()));        
            }
            if let Some(gnss)= &telemetry_state.pose_state.gnss_measurement {
                let _ = sender.try_send(AnyMessage::GnssState(gnss.clone()));
            }
            if let Ok(controller)= self.controller.lock() {
                for (_, motor_controller) in controller.get_motors() {
                    //let _ = sender.try_send(AnyMessage::MotorState(motor.clone()));
                }
            }
        }
    }

    /// Connect the controller to an interface
    /// 
    /// To receive commands that acto on the osprai and return telemetry to other softwares like Remote Controller
    /// 
    /// Arguments:
    /// 
    /// interface: The hardware interface the process must be to be connected to
    fn connect_interface(&mut self, interface: &mut dyn HardwareInterface) {
        interface.connect_process(self);
    }
}

impl Process for OspraiController {
    /// Execute process task
    /// 
    /// Send setpoint to the actuators
    /// 
    /// Send the telemetry data (vehicle state) to interface / process
    fn exec(&mut self, inputs: Option<AnyMessage>, dt: Duration) -> Option<AnyMessage> {
        self.send_telemetry();
        let mut motors_initialized= true;
        if let Ok(controller) = self.controller.lock() {
            for (_, motor_controller) in controller.get_motors() {
                /*if motor.max_value == 0.0 && motor.min_value == 0.0 {
                    motors_initialized= false;
                }*/
            }
        }
        if motors_initialized {
            self.apply_actuator_setpoints(inputs.into_iter().collect(), dt);
        }
        return None;
    }

    /// Helper function that force the Controller to have receiver object to receive cmd from other process or interface
    fn set_inbound_receiver(&mut self, receiver: broadcast::Receiver<AnyMessage>) {
        self.interface_cmd_receiver = Some(receiver);
    }

    /// Helper function that force the Controller to have sender object to send data to other process or interface
    fn set_outbound_sender(&mut self, sender: mpsc::Sender<AnyMessage>) {
        self.telemetry_sender = Some(sender);
    }
    
    fn set_name(&mut self, name: String) {
        self.name= name;
    }
}








