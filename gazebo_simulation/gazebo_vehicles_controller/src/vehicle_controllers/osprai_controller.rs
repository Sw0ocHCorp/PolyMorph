//! Gazebo bridge for the OSPRAI tilt-rotor bicopter: the only component that knows Gazebo. It
//! remounts the measurements (IMU, GNSS, lidar, cameras, odometry), discovers the motor tree from the
//! scene, converts efforts to actuator units and publishes the commands. Vehicle-specific by design:
//! everything generic lives in `robomorph`. See `docs/src/simulation/gazebo.md`.

use std::{collections::HashMap, f64::consts::PI, sync::{atomic::{AtomicBool, Ordering}, Arc, Mutex}, time::Duration};

use chrono::Utc;
use gz::{msgs::{actuators::Actuators, empty::Empty, image::Image, imu::IMU, laserscan::LaserScan, model::Model, navsat::NavSat, odometry::Odometry, scene::Scene}, transport::{Node, Publisher}};
use nalgebra::{UnitQuaternion, Vector3};
use prost_types::Timestamp;
use robomorph::{communications::interface::HardwareInterface, control::{motion::{motion_controller::{MotionController, VehicleKinematicConfig}, motor_controller::{MotorController, quaternion_to_euler, working_axis_to_vec3}, motors_mixer::MotorsMixer}, pid_controller::PIDController}, core::scheduler::Process, messages::{lidar_messages::{LidarMeasurements, Ray}, motor_messages::{MotorCommandType, MotorFeedBack, MotorModel, MotorStatus, PIDConfig, WorkingAxis}, pose_messages::{GNSSMeasurement, IMUMeasurements, Pose}, registered_message::{AnyMessage, UnitQuat, Vec3}}};
use tokio::sync::{broadcast, mpsc};

use crate::vehicle_controllers::vehicle_controller::VehicleController;

/// Roll / pitch / yaw of a Gazebo quaternion, for display only (see `quaternion_to_euler`).
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

/// One camera frame as received from Gazebo (RGB bytes), stamped on reception.
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
/// Everything the Gazebo callbacks write and the process reads. Shared behind `Arc<Mutex>` because
/// the callbacks run on the gz-transport threads. `pose_state.orientation` is the attitude
/// body->world published by the IMU sensor, which MUST be configured with
/// `<orientation_reference_frame>` / `parent_frame="world"` in the SDF: by default Gazebo references
/// the orientation to the sensor's pose at spawn ("local frame on boot"), and a vehicle spawned
/// tilted would read the identity.
pub struct OsprAiState {
    /// Lidar sensor specs (vertical / horizontal range, ray max range, etc...) & measurements
    pub lidar_measurements: LidarMeasurements,
    pub pose_state: Pose,
    /// Camera sources specs and image
    pub cameras: Vec<ImageState>,
}

impl OsprAiState {
    /// State before the first message: identity attitude (NOT the wire default, whose zero norm
    /// yields NaN when normalised), zero measurements, empty lidar, two empty camera slots.
    pub fn new() -> Self {
        let now= Timestamp {
            seconds: Utc::now().timestamp(),
            nanos: Utc::now().timestamp_subsec_nanos() as i32,
        };
        return OsprAiState {
            pose_state: Pose {timestamp: Some(now) , gnss_measurement: Some(GNSSMeasurement { timestamp: Some(now), latitude: 0.0, longitude: 0.0, altitude: 0.0, fix_status: 1 }), 
                                relative_location: Some(Vec3::default()), 
                                orientation: Some(UnitQuat { w: 1.0, x: 0.0, y: 0.0, z: 0.0 }), 
                                imu_measurement: Some(IMUMeasurements::default()), l_velocity: Some(Vec3::default())},
            lidar_measurements: LidarMeasurements { timestamp: Some(now),
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
        let now= Timestamp {
            seconds: Utc::now().timestamp(),
            nanos: Utc::now().timestamp_subsec_nanos() as i32,
        };
        return OsprAiState {
            pose_state: Pose {timestamp: Some(now),  gnss_measurement: Some(GNSSMeasurement { timestamp: Some(now), latitude: 0.0, longitude: 0.0, altitude: 0.0, fix_status: 1 }), 
                                relative_location: Some(Vec3::default()), 
                                // identity, NOT UnitQuat::default(): the wire default is (0,0,0,0), whose normalisation
                                // divides by a zero norm and yields NaN. Until the first IMU message arrives this
                                // value is used as the measured attitude, and a NaN here poisons the whole chain.
                                orientation: Some(UnitQuat { w: 1.0, x: 0.0, y: 0.0, z: 0.0 }), 
                                imu_measurement: Some(IMUMeasurements::default()), l_velocity: Some(Vec3::default())},
            lidar_measurements: LidarMeasurements { timestamp: Some(now),
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

/// Struct that represent the motor_mixer's of the osprai vehicle
/// 
/// can send telemetry data to an Hardware Interface to communicate with other softwares, receive command frames from the hardware interface
/// 
/// can apply commands on the hardware actuators or Gazebo simulated vehicle
pub struct OspraiController {
    name: String,
    /// Written by the IMU / GNSS / lidar / camera callbacks (250 Hz for the IMU), read once per
    /// process cycle by `send_telemetry`.
    /// Telemetry state that aggregate all the telemtry data
    pub telemetry_state: Arc<Mutex<OsprAiState>>,
    /// The node that allow the motor_mixer to connect itself to Gazebo simulation environment
    controller_node: Option<Node>,
    /// Receiver of the cmd from Hardware Interface
    interface_cmd_receiver: Option<broadcast::Receiver<AnyMessage>>,
    motor_config_sender: Option<broadcast::Sender<MotorController>>,
    /// Telemetry sender to Hardware Interface
    telemetry_sender: Option<broadcast::Sender<AnyMessage>>,
    motor_feedback_sender: Option<broadcast::Sender<MotorFeedBack>>,
    /// Attitude setpoint channel: today a fixed identity attitude with omega_d = 0 (stabilize).
    motion_setpoint_sender: Option<broadcast::Sender<Pose>>,
    /// Motor ids in the order expected by the Gazebo `Actuators` message:
    /// [rotor_left, rotor_right, arm_left, arm_right]. Ids are the byte-sum of the link name.
    motors_command_sequence: Vec<u8>,
    /// Publisher of `/osprai/command/motor_setpoints` (rotor speeds in rad/s, arm positions in rad).
    motors_command_publisher: Option<Publisher<Actuators>>,
    /// Shared with the Gazebo subscription callbacks, which run outside of `&self`
    motors_initialized: Arc<AtomicBool>,
    period: Duration,
    motors: Arc<Mutex<Vec<MotorController>>>,
    verbose: bool,
}

impl OspraiController {
    pub fn new(name: String, mixer: MotorsMixer, cmd_receiver: broadcast::Receiver<AnyMessage>, verbose: bool) -> Self {
        return Self {name, telemetry_state: Arc::new(Mutex::new(OsprAiState::new())),
                controller_node: None,
                interface_cmd_receiver: Some(cmd_receiver),
                telemetry_sender: None,
                motors_command_publisher: None,
                motor_config_sender: None,
                motor_feedback_sender: None,
                motion_setpoint_sender: None,
                motors_command_sequence: vec![0, 0, 0, 0],
                motors_initialized: Arc::new(AtomicBool::new(false)),
                period: Duration::from_millis(0),
                motors: Arc::new(Mutex::new(Vec::new())),
                verbose,
        };
    }
    /// Constructor
    /// 
    /// Arguments:
    /// 
    /// cmd_receiver: A vehicle commands receiver
    pub fn new_with_lidar(name: String, cmd_receiver: broadcast::Receiver<AnyMessage>, vehicle_config: VehicleKinematicConfig, verbose: bool) -> Self {
        return Self {name, telemetry_state: Arc::new(Mutex::new(OsprAiState::new_with_lidar())),
                controller_node: None,
                interface_cmd_receiver: Some(cmd_receiver),
                telemetry_sender: None,
                motors_command_publisher: None,
                motor_config_sender: None,
                motor_feedback_sender: None,
                motion_setpoint_sender: None,
                motors_command_sequence: vec![0, 0, 0, 0],
                motors_initialized: Arc::new(AtomicBool::new(false)),
                period: Duration::from_millis(0),
                motors: Arc::new(Mutex::new(Vec::new())),
                verbose
        };
    }

    pub fn set_motor_config_sender(&mut self, motor_config_sender: broadcast::Sender<MotorController>) {
        self.motor_config_sender= Some(motor_config_sender);
    }

    pub fn set_motor_feedback_sender(&mut self, motor_feedback_sender: broadcast::Sender<MotorFeedBack>) {
        self.motor_feedback_sender= Some(motor_feedback_sender);
    }

    pub fn set_motion_setpoint_sender(&mut self, setpoint_sender: broadcast::Sender<Pose>) {
        self.motion_setpoint_sender= Some(setpoint_sender);
    }

}

impl Default for OspraiController {
    /// Default constructor
    fn default() -> Self {
        Self { name: "osprai_controller".to_string(), telemetry_state: Arc::new(Mutex::new(OsprAiState::new())),
                controller_node: None,
                interface_cmd_receiver: None,
                telemetry_sender: None,
                motors_command_publisher: None,
                motors_command_sequence: vec![0, 0, 0, 0],
                motor_config_sender: None,
                motor_feedback_sender: None,
                motion_setpoint_sender: None,
                motors_initialized: Arc::new(AtomicBool::new(false)),
                period: Duration::from_millis(0),
                motors: Arc::new(Mutex::new(Vec::new())), 
                verbose: true,
        }
    }
}

impl VehicleController for OspraiController {
    /// Connect the simulation node to all the required topics to be able to control the vehicle
    /// 
    /// Notes:
    /// 
    /// Controller must be to be mutable because the topics will update the state of the vehicle
    /// Subscribe to every sensor topic, discover the motor tree from the scene, and wire the
    /// `joint_state` callback that (a) finishes the tree on its first message (parents, bounds,
    /// relative positions) and (b) publishes the motor feedbacks at every message.
    ///
    /// Frame / unit conventions established here and relied upon by the whole chain:
    ///   - IMU orientation = attitude body->world (see `OsprAiState`), gyro in the body frame;
    ///   - a rotor's feedback `current_value` is its THRUST in newtons, `T = transmission * k * w^n`
    ///     from the measured joint velocity - the effort law lives here, at the hardware boundary,
    ///     never in the mixer;
    ///   - an arm's feedback is its joint position in radians;
    ///   - rotor bounds: `min_value = 0.1 N` (never zero: an arm borrows its authority from its
    ///     rotor's thrust), `max_value = k * maxRotVelocity^n`; arm bounds from the joint limits,
    ///     joint speed 10 rad/s (the mixer's trust region per cycle).
    fn start_listening_topics(&mut self) {
        self.controller_node= Node::new();
        let verbose= self.verbose.clone();
        if let Some(node)=  &mut self.controller_node {
            self.motors_command_publisher = node.advertise::<Actuators>("/osprai/command/motor_setpoints");
            // Ground-truth pose from the simulator. Not used by the chain, but it is the judge of
            // every frame-convention test: cross-check the IMU quaternion against it.
            let _= node.subscribe("/model/osprai/odometry", move |msg: Odometry| {
                if verbose.clone() {
                    println!("[INFO] -> True pose= {:?}", msg.pose);
                    print!("");
                }
            });
            // 250 Hz. Orientation, gyro and accelerometer are written together, so a reader that
            // takes the lock once sees a coherent (q, omega) pair. The accelerometer norm is a free
            // contact detector: ~9.81 when supported, ~0 in free fall.
            let osprai_state_imu= self.telemetry_state.clone();
            let _= node.subscribe("osprai/imu", move |msg : IMU| {
                if let Ok(mut telemetry_state) = osprai_state_imu.clone().lock() &&
                        let Some(imu)= telemetry_state.pose_state.imu_measurement.as_mut() {
                    let now= Timestamp {
                        seconds: Utc::now().timestamp(),
                        nanos: Utc::now().timestamp_subsec_nanos() as i32,
                    };
                    imu.timestamp= Some(now);
                    imu.l_accel= Some(Vec3::new(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z));
                    imu.a_velocity= Some(Vec3::new(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z));
                    telemetry_state.pose_state.orientation= Some(UnitQuat { w: msg.orientation.w, x: msg.orientation.x, y: msg.orientation.y, z: msg.orientation.z });
                    if let Some(quat)= telemetry_state.pose_state.orientation {
                        let euler= quaternion_to_euler(&UnitQuaternion::from(quat));
                        if verbose.clone() {
                            println!("IMU orientation: roll: {}, pitch: {}, yaw: {}", euler[0], euler[1], euler[2]);
                        }
                    }
                }
            });
            let osprai_state_gnss= self.telemetry_state.clone();
            let _= node.subscribe("osprai/gps", move |msg : NavSat| {
                if let Ok(mut telemetry_state) = osprai_state_gnss.clone().lock() && 
                        let Some(gnss)= telemetry_state.pose_state.gnss_measurement.as_mut() {
                            let now= Timestamp {
                                seconds: Utc::now().timestamp(),
                                nanos: Utc::now().timestamp_subsec_nanos() as i32,
                            };
                            *gnss= GNSSMeasurement {timestamp: Some(now), longitude: msg.longitude_deg, latitude: msg.latitude_deg,
                                                            altitude: msg.altitude, fix_status: 1 };
                }
            });
            let osprai_state_lidar= self.telemetry_state.clone();
            let _= node.subscribe("osprai/lidar", move |msg: LaserScan| {
                if let Ok(mut telemetry_state) = osprai_state_lidar.clone().lock() {
                    let now= Timestamp {
                        seconds: Utc::now().timestamp(),
                        nanos: Utc::now().timestamp_subsec_nanos() as i32,
                    };
                    telemetry_state.lidar_measurements.timestamp= Some(now);
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

            if let Some((scene, true)) = node.request::<Empty, Scene>(
                "/world/my_environment/scene/info",
                &Empty::default(),
                Duration::from_secs(2),
            ) {
                for model in &scene.model {
                    if model.name == "osprai" {
                        for link in &model.link {
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
                                    let now= Timestamp {
                                        seconds: Utc::now().timestamp(),
                                        nanos: Utc::now().timestamp_subsec_nanos() as i32,
                                    };
                                    let motor_feedback= MotorFeedBack { timestamp: Some(now),
                                        id: link_id as u32,
                                        status: MotorStatus::IDLE as i32,
                                        command_type: MotorCommandType::ANGULARPOSITION as i32,
                                        current_value: 0.0,
                                        setpoint_value: 0.0,
                                        pid_config: Some(PIDConfig { p: 1.0, i: 0.0, d: 0.0, 
                                                            min_correction_error: 0.0, max_error_accum: PI}),
                                        control_frequency: 50,
                                    };
                                    let pid= PIDController::new(0.2, 1.0, 0.0, 0.0, PI);
                                    if let Ok(mut motors) = self.motors.clone().lock() {  
                                        motors.push(MotorController::new(motor_model, motor_feedback, pid));
                                    }
                                    //motor_mixer.add_or_update_motor(MotorController::new(motor_model, motor_feedback, pid));
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
                                    let now= Timestamp {
                                        seconds: Utc::now().timestamp(),
                                        nanos: Utc::now().timestamp_subsec_nanos() as i32,
                                    };
                                    let motor_feedback= MotorFeedBack { timestamp: Some(now),
                                        id: link_id as u32,
                                        status: MotorStatus::IDLE as i32,
                                        command_type: MotorCommandType::THRUST as i32,
                                        current_value: 0.0,
                                        setpoint_value: 0.0,
                                        pid_config: Some(PIDConfig { p: 1.0, i: 0.0, d: 0.0, 
                                                            min_correction_error: 0.0, max_error_accum: f64::INFINITY}),
                                        control_frequency: 50,
                                    };
                                    let pid= PIDController::new(1.0, 0.0, 0.0, 0.0, f64::INFINITY);
                                    if let Ok(mut motors) = self.motors.clone().lock() { 
                                        motors.push(MotorController::new(motor_model, motor_feedback, pid));
                                    }
                                }
                            if verbose.clone() {
                                println!("{} pose: {:?}", link.name, link.pose);
                            }
                        }
                        if verbose.clone() {
                            println!("Model {} pose: {:?}", model.name, model.pose);
                        }
                    }
                }
            }

            let motors_initialized_cl= self.motors_initialized.clone();
            // the callback runs on the Gazebo transport thread and must be 'static, so it cannot
            // borrow `self`: every piece of state it touches is captured as its own handle.
            let motor_config_sender_cl= self.motor_config_sender.clone();
            let motor_feedback_sender_cl= self.motor_feedback_sender.clone();
            let motors_cl= self.motors.clone();
            // Runs at the simulator's joint_state rate on a gz-transport thread. `tilt_*` joints map
            // to the `arm_*` motors (angle feedback), `spin_*` joints to the `rotor_*` motors (velocity
            // -> thrust feedback). The tree is finalised once (`motors_initialized`), then the
            // `MotorController`s are sent to the mixer through the configuration channel.
            let _= node.subscribe("/osprai/joint_state", move |msg: Model| {
                if let Some(mcs)= &motor_config_sender_cl && mcs.receiver_count() > 0  &&
                        let Some(mfs)= &motor_feedback_sender_cl && mfs.receiver_count() > 0 {
                    let now= Timestamp {
                        seconds: Utc::now().timestamp(),
                        nanos: Utc::now().timestamp_subsec_nanos() as i32,
                    };
                    for j in msg.joint {
                        let mut parent_id: u8= j.parent.bytes().fold(0u8, |acc, b| acc.wrapping_add(b));
                        let mut name= j.name.clone();
                        let mut id= 0;
                        if j.name.contains("tilt") {
                            name= name.replace("tilt", "arm");
                            id= name.bytes().fold(0u8, |acc, b| acc.wrapping_add(b));
                            if let Ok(mut motors) = motors_cl.lock() {
                                for i in 0..motors.len() {
                                    // `&mut`: the MotorController is mutated in place inside the Vec,
                                    // indexing by value would try to move it out
                                    let mc= &mut motors[i];
                                    if mc.get_motor_model().id == id as u32 {
                                        if motors_initialized_cl.load(Ordering::Relaxed) == false {
                                            mc.set_parent_id(parent_id as u32);
                                            mc.set_motor_max_value(j.axis1.limit_upper);
                                            mc.set_motor_min_value(j.axis1.limit_lower);
                                            mc.set_max_rot_speed(10.0);
                                        }

                                        if mc.get_working_axis() as i32 >= WorkingAxis::RotatingAroundX as i32 && mc.get_working_axis() as i32 <= WorkingAxis::RotatingAroundZ as i32 {
                                            if verbose.clone() {
                                                println!("Received motor measurements: id: {}, name: {}, type: {}, value: {}", id, name, mc.get_working_axis() as i32, j.axis1.position);
                                            }
                                            mc.set_feedback_timestamp(now);
                                            mc.set_motor_current_value(j.axis1.position);
                                            let _= mfs.send(mc.get_motor_feedback().clone());
                                        }
                                    }
                                }
                            }
                        } else if j.name.contains("spin") {
                            name= name.replace("spin", "rotor");
                            id= name.bytes().fold(0u8, |acc, b| acc.wrapping_add(b));
                            if let Ok(mut motors) = motors_cl.lock() {
                                for i in 0..motors.len() {
                                    let mc= &mut motors[i];
                                    if mc.get_motor_model().id == id as u32 {
                                        if motors_initialized_cl.load(Ordering::Relaxed) == false {
                                            let motor_model= mc.get_motor_model();
                                            let (transmission_factor, effort_constant, exp_command_law)= (motor_model.transmission_factor, motor_model.effort_constant ,motor_model.exp_command_law);
                                            mc.set_parent_id(parent_id as u32);
                                            mc.set_motor_min_value(0.1);
                                            let max_thrust= transmission_factor * effort_constant * 
                                                                f64::powf(1200.0, exp_command_law as f64);
                                            mc.set_motor_max_value(max_thrust);
                                        }
                                        
                                        if mc.get_working_axis() as i32 >= WorkingAxis::LinearMotionALongX as i32 && mc.get_working_axis() as i32 <= WorkingAxis::LinearMotionALongZ as i32 {
                                            if verbose.clone() {
                                                println!("Received motor measurements: id: {}, name: {}, type: {}, value: {}", id, name, mc.get_working_axis() as i32, j.axis1.velocity);
                                            }
                                            let motor_model= mc.get_motor_model();
                                            let current_thrust= motor_model.transmission_factor * motor_model.effort_constant * 
                                                                    j.axis1.velocity.powf(motor_model.exp_command_law as f64);
                                            mc.set_feedback_timestamp(now);
                                            mc.set_motor_current_value(current_thrust);
                                            let _= mfs.send(mc.get_motor_feedback().clone());
                                        }
                                    }
                                }
                            }
                        }
                    }
                    
                    if motors_initialized_cl.load(Ordering::Relaxed) == false {
                        if let Ok(mut motors) = motors_cl.lock() {
                            for i in 0..motors.len() {
                                // read only: this borrow of `motors` ends at the end of the line,
                                // so the Vec is free again for the inner loop
                                let parent_id= motors[i].get_motor_model().parent_id;
                                let mut has_parent= false;
                                for j in 0..motors.len() {
                                    if j != i && motors[j].get_motor_model().id == parent_id {
                                        // both elements are taken in ONE call: get_disjoint_mut checks that the
                                        // indices differ, which is what allows two &mut into the same Vec.
                                        // `j != i` above already guarantees the Ok branch.
                                        if let Ok([mc, parent_mc])= motors.get_disjoint_mut([i, j]) {
                                            if let Some(mut relative_location)= mc.get_motor_model().relative_location {
                                                let mut parent_relative_location= Vec3::new(0.0, 0.0, 0.0);
                                                if let Some(relative_loc)= parent_mc.get_motor_model().relative_location {
                                                    parent_relative_location= relative_loc;
                                                }
                                                relative_location -= parent_relative_location;
                                            }
                                            parent_mc.add_child(mc.get_motor_model().id);
                                            has_parent= true;
                                        }
                                    }
                                }
                                let mc= &mut motors[i];
                                if has_parent == false {
                                    mc.set_parent_id(0);
                                }
                                // the channel takes ownership of the value, and the motor has to stay
                                // in `motors`: a clone is sent instead of the element itself
                                
                            }
                            for i in 0..motors.len() {
                                let mc= &motors[i];
                                let _= mcs.send(mc.clone());
                            }
                        }
                        motors_initialized_cl.store(true, Ordering::Relaxed);
                    }
                }
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
    /*fn apply_actuator_setpoints(&mut self, setpoints: Vec<AnyMessage>, _dt: Duration) {
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
        // the commands are not computed here anymore: the MotorsMixer is a Process of its own and
        // publishes them on the command channel, so they reach this method through `setpoints`.
        let Some(motors_command)= setpoints.into_iter().find_map(|msg| match msg {
            AnyMessage::MotorCommands(commands) => Some(commands),
            _ => None,
        }) else {
            return;
        };
        if let Ok(motors)= self.motors.lock() &&
                let Some(motor_pub)= self.motors_command_publisher.as_mut() &&
                motors_command.len() > 0 {
            let mut actuator_setpoints= Actuators::new();
            for id in self.motors_command_sequence.clone() {
                for cmd in motors_command.clone() {
                    if cmd.id == id as u32 && let Some(motor_controller)= motors.iter().find(|mc| mc.get_motor_model().id == cmd.id) {
                        // The motor_mixer works in EFFORT units: newtons for a thruster, radians for a joint.
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
    }*/


    /// Send telemetry data to the telemetry receiver linked to telemetry_sender
    /// 
    /// Most likely to an interface
    /// Publish the latest pose (updated by the IMU callback at 250 Hz, hence < 4 ms old) on the
    /// telemetry channel. Called first in `exec`, i.e. right before the attitude controller runs in
    /// the same scheduler pass: the attitude always consumes the measurement of the current pass.
    /// NOTE: prints the whole pose every tick when reached (verbose is not checked here).
    fn send_telemetry(&mut self) {
        if let Some(sender) = &self.telemetry_sender &&
            let Ok(telemetry_state)= self.telemetry_state.lock() {
            // broadcast::Sender::send never blocks: it returns Err when nobody is subscribed,
            // which is not an error here
            println!("Telemetry Sent:\n{:?} to {} receivers", telemetry_state.pose_state.clone(), sender.receiver_count());
            let _ = sender.send(AnyMessage::PoseState(telemetry_state.pose_state.clone()));
            /*if let Ok(motors)= self.motors.lock() {
                for motor_controller in motors.iter() {
                    //let _ = sender.send(AnyMessage::MotorState(motor.clone()));
                }
            }*/
        }
    }

}

impl Process for OspraiController {
    /// Execute process task
    /// 
    /// Send setpoint to the actuators
    /// 
    /// Send the telemetry data (vehicle state) to interface / process
    /// One cycle: (1) publish telemetry, (2) publish the attitude setpoint (identity, omega_d = 0;
    /// the `l_accel` field of that setpoint is not read by the attitude controller), (3) drain the
    /// command channel: convert each `MotorCommand` from efforts to actuator units - THRUST (N) ->
    /// rotor speed `w = (T / (transmission * k))^(1/n)` (rad/s), ANGULARPOSITION (rad) unchanged -
    /// in the `motors_command_sequence` order, and publish the `Actuators` message only when it is
    /// complete. Commands computed at pass k are therefore applied at pass k+1 (one-cycle delay,
    /// covered by the tau >= 10 T margin).
    /// NOTE: never publish a non-finite value: the MulticopterMotorModel keeps a NaN in its
    /// first-order filter for the rest of the run.
    fn exec(&mut self, input: &Option<AnyMessage>, dt: Duration) -> Option<AnyMessage> {
        self.send_telemetry();
        if let Some(setpoint_sender)= self.motion_setpoint_sender.clone() {
            let mut pose_setpoint= Pose::default();
            pose_setpoint.orientation= Some(UnitQuat::from(UnitQuaternion::identity()));
            pose_setpoint.imu_measurement= Some(IMUMeasurements::default());
            if let Some(measurements)= &mut pose_setpoint.imu_measurement {
                measurements.a_velocity= Some(Vec3::default());
                measurements.l_accel= Some(Vec3::new(0.0, 0.0, 10.0));
            }
            let _= setpoint_sender.send(pose_setpoint);
        }
        if let Some(cmds_rcvr) = &mut self.interface_cmd_receiver {
            let mut actuator_setpoints= Actuators::new();
            let sz= cmds_rcvr.len();
            for _ in 0..sz {
                if let Ok(cmd)= cmds_rcvr.try_recv() {
                    if let AnyMessage::MotorCommands(motor_cmds)= cmd {
                        if let Ok(motors) = self.motors.clone().lock() {
                            for motor_cmd in &motor_cmds {
                                let mut idx= 0;
                                //iterate over the id of motor in the actuator command sequence
                                for id in self.motors_command_sequence.clone() {
                                    //get the motor model of the motor in the command sequence
                                    let mut model= None;
                                    for i in 0..motors.len() {
                                        let modl= motors[i].get_motor_model();
                                        if modl.id == id as u32 {
                                            model= Some(motors[i].get_motor_model());
                                            break;
                                        }
                                    }
                                    if let Some(motor_model) = model {
                                        if motor_cmd.command_type == MotorCommandType::THRUST as i32 &&  motor_model.working_axis >= WorkingAxis::LinearMotionALongX as i32 {
                                            if id == motor_cmd.id as u8 && id == motor_model.id as u8 {
                                                while idx >= actuator_setpoints.velocity.len() {
                                                    actuator_setpoints.velocity.push(0.0);
                                                }
                                                // thrust -> rotor speed: the MulticopterMotorModel is driven in rad/s
                                                //      T = transmission * effort_constant * w^exp   =>   w = (T / (transmission * effort_constant))^(1/exp)
                                                let setpoint_rot_speed= (motor_cmd.setpoint_value / (motor_model.transmission_factor * motor_model.effort_constant)).powf(1.0 / motor_model.exp_command_law as f64);
                                                actuator_setpoints.velocity[idx] = setpoint_rot_speed;
                                                break;
                                            } else {
                                                idx+= 1;
                                            }
                                        } else if motor_cmd.command_type == MotorCommandType::VELOCITY as i32 &&  motor_model.working_axis >= WorkingAxis::LinearMotionALongX as i32 {
                                            if id == motor_cmd.id as u8 && id == motor_model.id as u8 {
                                                while idx >= actuator_setpoints.velocity.len() {
                                                    actuator_setpoints.velocity.push(0.0);
                                                }
                                                // velocity of a motor that produce thrust is already commanded in rad/s or tr/s: no conversion
                                                actuator_setpoints.velocity[idx] = motor_cmd.setpoint_value;
                                                break;
                                            } else {
                                                idx+= 1;
                                            }
                                        } else if motor_cmd.command_type == MotorCommandType::ANGULARPOSITION as i32 &&  motor_model.working_axis <= WorkingAxis::RotatingAroundZ as i32 {
                                            if id == motor_cmd.id as u8 && id == motor_model.id as u8 {
                                                while idx >= actuator_setpoints.position.len() {
                                                    actuator_setpoints.position.push(0.0);
                                                }
                                                // a position-controlled joint is already commanded in radians: no conversion
                                                actuator_setpoints.position[idx] = motor_cmd.setpoint_value;
                                                break;
                                            } else {
                                                idx+= 1;
                                            }
                                        }
                                    }
                                }    
                            }
                        }
                        if let Some(motor_pub)= self.motors_command_publisher.as_mut() && 
                                (actuator_setpoints.position.len() + actuator_setpoints.velocity.len() == self.motors_command_sequence.len()){
                            let _= motor_pub.publish(&actuator_setpoints);
                        }
                    }
                }

            }
        }
        return None;
    }


    /// Helper function that force the Controller to have receiver object to receive cmd from other process or interface
    fn set_receiver(&mut self, receiver: broadcast::Receiver<AnyMessage>) {
        self.interface_cmd_receiver = Some(receiver);
    }

    /// Helper function that force the Controller to have sender object to send data to other process or interface
    fn set_sender(&mut self, sender: broadcast::Sender<AnyMessage>) {
        self.telemetry_sender = Some(sender);
    }
    
    fn set_name(&mut self, name: String) {
        self.name= name;
    }

    fn get_name(&self) ->String {
        return self.name.clone();
    }
    
    fn set_period_from_freq(&mut self, frequency: u64) {
        self.period= Duration::from_nanos(1_000_000_000 / frequency);
    }
    
    fn get_period(&self) -> Duration {
        return self.period;
    }
}








