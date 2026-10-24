//! Gazebo bridge for the OSPRAI tilt-rotor bicopter: the only component that knows Gazebo. It
//! remounts the measurements (IMU, GNSS, lidar, cameras, odometry), discovers the motor tree from the
//! scene, converts efforts to actuator units and publishes the commands. Vehicle-specific by design:
//! everything generic lives in `robomorph`. See `docs/docs/simulation/gazebo.mdx`.

use std::{collections::HashMap, sync::{Arc, Mutex, OnceLock}, time::Duration};

use chrono::Utc;
use gz::{msgs::{actuators::Actuators, empty::Empty, image::Image, imu::IMU, laserscan::LaserScan, model::Model, navsat::NavSat, odometry::Odometry, scene::Scene}, transport::{Node, Publisher}};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use prost_types::Timestamp;
use robomorph::{communications::interface::HardwareInterface, control::{motion::{motion_controller::{MotionController, VehicleKinematicConfig}, motor::quaternion_to_euler, motor_tree::{MotorTree, SharedMotorTree}, motors_mixer::MotorsMixer}, pid_controller::PIDController}, core::scheduler::Process, messages::{lidar_messages::{LidarMeasurements, Ray}, motor_messages::{MotorCommandType, MotorFeedBack, MotorModel, MotorState, MotorStatus, WorkingAxis}, pose_messages::{GNSSMeasurement, IMUMeasurements, Pose}, registered_message::{AnyMessage, UnitQuat, Vec3}}};
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

/// Simulated time carried by a Gazebo message (`header.stamp`, a gz.msgs.Time).
///
/// Every sample that goes into the `Pose` (IMU, odometry, GNSS) is stamped with it, never with the
/// wall clock: the scenarios are played and measured on the clock of the PHYSICS, and the two
/// clocks drift apart as soon as Gazebo runs slower or faster than real time.
pub fn gz_header_to_timestamp(header: &gz::msgs::header::Header) -> Timestamp {
    return Timestamp { seconds: header.stamp.sec, nanos: header.stamp.nsec };
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
            // [P3] no sample yet: no time, no position, no velocity (None, never a fake zero). The
            // consumers abstain until the first odometry / IMU message fills them.
            pose_state: Pose {timestamp: None , gnss_measurement: Some(GNSSMeasurement { timestamp: Some(now), latitude: 0.0, longitude: 0.0, altitude: 0.0, fix_status: 1 }), 
                                relative_location: None, 
                                orientation: Some(UnitQuat { w: 1.0, x: 0.0, y: 0.0, z: 0.0 }), 
                                imu_measurement: Some(IMUMeasurements::default()), l_velocity: None, force: None},
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
            // [P3] no sample yet: no time, no position, no velocity (None, never a fake zero)
            pose_state: Pose {timestamp: None,  gnss_measurement: Some(GNSSMeasurement { timestamp: Some(now), latitude: 0.0, longitude: 0.0, altitude: 0.0, fix_status: 1 }), 
                                relative_location: None, 
                                // identity, NOT UnitQuat::default(): the wire default is (0,0,0,0), whose normalisation
                                // divides by a zero norm and yields NaN. Until the first IMU message arrives this
                                // value is used as the measured attitude, and a NaN here poisons the whole chain.
                                orientation: Some(UnitQuat { w: 1.0, x: 0.0, y: 0.0, z: 0.0 }), 
                                imu_measurement: Some(IMUMeasurements::default()), l_velocity: None, force: None},
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
///
/// The bridge is the hardware boundary and nothing more: it publishes the measured state and applies
/// the motor commands. It does NOT produce setpoints: the mission setpoint comes from the scenario
/// player (`crate::scenarios::player`), the single source of setpoints of the binary.
pub struct OspraiController {
    name: String,
    /// Written by the IMU / odometry / GNSS / lidar / camera callbacks (250 Hz for the IMU and the
    /// odometry), read once per process cycle by `send_telemetry`.
    /// Telemetry state that aggregate all the telemtry data
    pub telemetry_state: Arc<Mutex<OsprAiState>>,
    /// The node that allow the motor_mixer to connect itself to Gazebo simulation environment
    controller_node: Option<Node>,
    /// Receiver of the cmd from Hardware Interface
    interface_cmd_receiver: Option<broadcast::Receiver<AnyMessage>>,
    /// The motor tree of the vehicle, shared with the control stages: written ONCE here, when the
    /// first `joint_state` message completes the discovery; read by `exec` to convert the commands.
    motor_tree: SharedMotorTree,
    /// Telemetry sender to Hardware Interface
    telemetry_sender: Option<broadcast::Sender<AnyMessage>>,
    motor_feedback_sender: Option<broadcast::Sender<MotorFeedBack>>,
    /// Motor ids in the order expected by the Gazebo `Actuators` message:
    /// [rotor_left, rotor_right, arm_left, arm_right]. Ids are the byte-sum of the link name.
    motors_command_sequence: Vec<u8>,
    /// Publisher of `/osprai/command/motor_setpoints` (rotor speeds in rad/s, arm positions in rad).
    motors_command_publisher: Option<Publisher<Actuators>>,
    period: Duration,
    /// The runtime state of each motor (the measurement it publishes, the command it applied), found
    /// by `MotorState::id`. Its description is in `motor_tree` only. Shared with the Gazebo
    /// subscription callbacks, which run outside of `&self`.
    motors: Arc<Mutex<Vec<MotorState>>>,
    verbose: bool,
}

impl OspraiController {
    pub fn new(name: String, mixer: MotorsMixer, cmd_receiver: broadcast::Receiver<AnyMessage>, verbose: bool) -> Self {
        return Self {name, telemetry_state: Arc::new(Mutex::new(OsprAiState::new())),
                controller_node: None,
                interface_cmd_receiver: Some(cmd_receiver),
                telemetry_sender: None,
                motors_command_publisher: None,
                motor_tree: Arc::new(OnceLock::new()),
                motor_feedback_sender: None,
                motors_command_sequence: vec![0, 0, 0, 0],
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
                motor_tree: Arc::new(OnceLock::new()),
                motor_feedback_sender: None,
                motors_command_sequence: vec![0, 0, 0, 0],
                period: Duration::from_millis(0),
                motors: Arc::new(Mutex::new(Vec::new())),
                verbose
        };
    }

    /// The motor tree shared with the control stages: the bridge writes it once, when it has
    /// discovered the motors.
    pub fn set_motor_tree(&mut self, motor_tree: SharedMotorTree) {
        self.motor_tree= motor_tree;
    }

    pub fn set_motor_feedback_sender(&mut self, motor_feedback_sender: broadcast::Sender<MotorFeedBack>) {
        self.motor_feedback_sender= Some(motor_feedback_sender);
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
                motor_tree: Arc::new(OnceLock::new()),
                motor_feedback_sender: None,
                period: Duration::from_millis(0),
                motors: Arc::new(Mutex::new(Vec::new())), 
                // quiet by default: the verbose prints (IMU, odometry and joint_state at up to 1 kHz)
                // load the machine and add jitter to the control loop
                verbose: false,
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
            let osprai_state_odom= self.telemetry_state.clone();
            // Ground-truth pose from the simulator (OdometryPublisher, 250 Hz, see the SDF). It fills
            // the TRANSLATION part of the state - position and linear velocity, both in the WORLD
            // frame [P7] - which the velocity loop will consume. The attitude stays the IMU's: the
            // odometry quaternion is only used here, to bring the velocity into the world, and it
            // remains the judge of the frame-convention tests (cross-check the IMU against it).
            let _= node.subscribe("/model/osprai/odometry", move |msg: Odometry| {
                if verbose {
                    println!("[INFO] -> True pose= {:?}", msg.pose);
                }
                if let Ok(mut telemetry_state) = osprai_state_odom.lock() {
                    // 1. q_odom, the attitude body->world reported by the odometry. nalgebra's
                    //    Quaternion::new takes (w, x, y, z), w FIRST; from_quaternion renormalises.
                    let q_odom= UnitQuaternion::from_quaternion(Quaternion::new(msg.pose.orientation.w, msg.pose.orientation.x,
                                                                                msg.pose.orientation.y, msg.pose.orientation.z));
                    // 2. Gazebo publishes twist.linear in the BODY frame (the odometry's child frame):
                    //    rotating it by q_odom gives the velocity in the WORLD frame [P7].
                    //    Bug #1 was here: the x/y/z components of the quaternion were used as three
                    //    Euler angles, which is not a rotation of the vehicle at all.
                    let v_body= Vector3::new(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z);
                    telemetry_state.pose_state.l_velocity= Some(Vec3(q_odom * v_body));
                    // 3. position of the model origin, already in the world frame (m)
                    telemetry_state.pose_state.relative_location= Some(Vec3::new(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z));
                    // 4. the state is now as recent as this sample: stamp it with its SIMULATED time
                    telemetry_state.pose_state.timestamp= Some(gz_header_to_timestamp(&msg.header));
                }
            });
            // 250 Hz. Orientation, gyro and accelerometer are written together, so a reader that
            // takes the lock once sees a coherent (q, omega) pair. The accelerometer norm is a free
            // contact detector: ~9.81 when supported, ~0 in free fall.
            let osprai_state_imu= self.telemetry_state.clone();
            let _= node.subscribe("osprai/imu", move |msg : IMU| {
                // what to print, computed under the lock but printed AFTER it is released: a print can
                // block on a slow terminal, and the control thread needs this lock at every pass
                let mut euler_to_print= None;
                if let Ok(mut telemetry_state) = osprai_state_imu.clone().lock() &&
                        let Some(imu)= telemetry_state.pose_state.imu_measurement.as_mut() {
                    // 1. the SIMULATED time of the sample (see `gz_header_to_timestamp`)
                    let stamp= gz_header_to_timestamp(&msg.header);
                    // 2. the raw sample: accelerometer and gyro, body frame
                    imu.timestamp= Some(stamp);
                    imu.l_accel= Some(Vec3::new(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z));
                    imu.a_velocity= Some(Vec3::new(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z));
                    // 3. the attitude body->world, and the state is now as recent as this sample
                    telemetry_state.pose_state.orientation= Some(UnitQuat { w: msg.orientation.w, x: msg.orientation.x, y: msg.orientation.y, z: msg.orientation.z });
                    telemetry_state.pose_state.timestamp= Some(stamp);
                    // 4. verbose only: the Euler angles, kept for printing once the lock is released
                    if verbose && let Some(quat)= telemetry_state.pose_state.orientation {
                        euler_to_print= Some(quaternion_to_euler(&UnitQuaternion::from(quat)));
                    }
                }
                // 5. the lock is released here: printing can no longer delay the control thread
                if let Some(euler)= euler_to_print {
                    println!("IMU orientation: roll: {}, pitch: {}, yaw: {}", euler[0], euler[1], euler[2]);
                }
            });
            let osprai_state_gnss= self.telemetry_state.clone();
            let _= node.subscribe("osprai/gps", move |msg : NavSat| {
                if let Ok(mut telemetry_state) = osprai_state_gnss.clone().lock() && 
                        let Some(gnss)= telemetry_state.pose_state.gnss_measurement.as_mut() {
                            // part of the Pose, hence stamped with the simulated time like the IMU
                            *gnss= GNSSMeasurement {timestamp: Some(gz_header_to_timestamp(&msg.header)), longitude: msg.longitude_deg, latitude: msg.latitude_deg,
                                                            altitude: msg.altitude, fix_status: 1 };
                }
            });
            // NOTE: the lidar, the cameras and the motor feedbacks are still stamped with the wall
            // clock (`Utc::now()`): nothing reads those stamps today. Align them on the simulated
            // time (`gz_header_to_timestamp`) when the perception starts using them.
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

            // the descriptions discovered in the scene, completed by the first joint_state message,
            // then consumed into the motor tree: no copy of them stays in the bridge
            let mut discovered: Vec<MotorModel>= Vec::new();
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
                                    // parent and bounds come with the first joint_state message
                                    let motor_model= MotorModel {
                                        id: link_id as u32,
                                        parent_id: 0,
                                        relative_location:  Some(Vec3::new(link.pose.position.x, 
                                                                        link.pose.position.y, 
                                                                        link.pose.position.z)),
                                        working_axis: WorkingAxis::RotatingAroundY as i32,
                                        min_value: 0.0,
                                        max_value: 0.0,
                                        relative_orientation: Some(UnitQuat::from(UnitQuaternion::identity())),
                                        exp_command_law: 1,
                                        max_rot_speed: 0.0,
                                        transmission_factor: 1.0,
                                        effort_constant: 1.2e-5,
                                        moment_constant: 0.016,
                                    };
                                    discovered.push(motor_model);
                                    if let Ok(mut motors) = self.motors.clone().lock() {  
                                        motors.push(MotorState::new(link_id as u32));
                                    }
                                } else if link.name.contains("rotor") {
                                    // `moment_constant` is the reaction moment the rotor exerts on the AIRFRAME, per
                                    // newton of thrust, along the thrust axis. A rotor spinning counter-clockwise
                                    // about that axis pushes the airframe clockwise: negative. The SDF spins the
                                    // left rotor ccw and the right one cw (`turningDirection`); measured in flight
                                    // by docs/scripts/identify_vehicle.py: k_m = 0.0158 with that sign.
                                    // Bug #12 (fixed in brick 2): the signs were swapped, so the mixer expected the
                                    // yaw torque of a differential thrust the wrong way round and its compensation
                                    // by the arms added to it (steady yaw error of 0.1 deg while a roll is held).
                                    let mut moment_constant= -0.016;
                                    if link.name.contains("right") {
                                        moment_constant= 0.016;
                                    }
                                    // parent and bounds come with the first joint_state message
                                    let motor_model= MotorModel {
                                        id: link_id as u32,
                                        parent_id: 0,
                                        relative_location:  Some(Vec3::new(link.pose.position.x, 
                                                                        link.pose.position.y, 
                                                                        link.pose.position.z)),
                                        working_axis: WorkingAxis::LinearMotionALongZ as i32,
                                        min_value: 0.0,
                                        max_value: 0.0,
                                        relative_orientation: Some(UnitQuat::from(UnitQuaternion::identity())),
                                        exp_command_law: 2,
                                        max_rot_speed: 1200.0,
                                        transmission_factor: 1.0,
                                        effort_constant: 1.2e-5,
                                        moment_constant: moment_constant,
                                    };
                                    discovered.push(motor_model);
                                    if let Ok(mut motors) = self.motors.clone().lock() { 
                                        motors.push(MotorState::new(link_id as u32));
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

            // the callback runs on the Gazebo transport thread and must be 'static, so it cannot
            // borrow `self`: every piece of state it touches is captured as its own handle. It OWNS
            // `discovered` (gz-transport takes an FnMut, which may mutate what it captured).
            let motor_tree_cl= self.motor_tree.clone();
            let motor_feedback_sender_cl= self.motor_feedback_sender.clone();
            let motors_cl= self.motors.clone();
            // Runs at the simulator's joint_state rate on a gz-transport thread. `tilt_*` joints map
            // to the `arm_*` motors (angle feedback), `spin_*` joints to the `rotor_*` motors (velocity
            // -> thrust feedback). Ids are the byte-sum of the motor's link name. Two steps:
            //   1. once, on the first message: the discovered descriptions are completed (parents,
            //      bounds, positions relative to the parent) and become the shared `MotorTree`, the
            //      very instance every control stage reads; `discovered` is emptied into it;
            //   2. on every message, the first one included: each joint's measurement, computed with
            //      the motor's description IN THE TREE, recorded in its state and published.
            let _= node.subscribe("/osprai/joint_state", move |msg: Model| {
                // verbose lines, collected while the `motors` lock is held and printed at the very end,
                // once every lock is released (the control thread needs `motors` at every pass)
                let mut verbose_lines: Vec<String>= Vec::new();
                let byte_sum= |name: &str| name.bytes().fold(0u8, |acc, b| acc.wrapping_add(b)) as u32;
                // 1. the tree, once: not written yet, and descriptions left to complete (a refused tree
                //    has emptied them, so it is not retried at every message)
                if motor_tree_cl.get().is_none() && !discovered.is_empty() {
                    // 1a. each joint gives its motor's parent and bounds
                    for j in &msg.joint {
                        let parent_id= byte_sum(&j.parent);
                        if j.name.contains("tilt") && let Some(model)= discovered.iter_mut().find(|model| model.id == byte_sum(&j.name.replace("tilt", "arm"))) {
                            // an arm: the joint limits, and its speed (the mixer's trust region per cycle)
                            model.parent_id= parent_id;
                            model.max_value= j.axis1.limit_upper;
                            model.min_value= j.axis1.limit_lower;
                            model.max_rot_speed= 10.0;
                        } else if j.name.contains("spin") && let Some(model)= discovered.iter_mut().find(|model| model.id == byte_sum(&j.name.replace("spin", "rotor"))) {
                            // a rotor: never below 0.1 N (an arm borrows its authority from its rotor's
                            // thrust), at most the thrust of the maximum rotor speed
                            model.parent_id= parent_id;
                            model.min_value= 0.1;
                            model.max_value= model.transmission_factor * model.effort_constant * 
                                                f64::powf(1200.0, model.exp_command_law as f64);
                        }
                    }
                    // 1b. the MODEL-frame position of every motor, read once before any is rewritten.
                    //     Each child is then placed relative to its parent from these values, so the
                    //     result does not depend on the order the motors are visited in, nor on how
                    //     deep the tree is.
                    let model_positions: Vec<(u32, Option<Vec3>)>= discovered.iter()
                        .map(|model| (model.id, model.relative_location)).collect();
                    let model_position= |id: u32| model_positions.iter().find(|(motor_id, _)| *motor_id == id).and_then(|(_, p)| *p);
                    for model in discovered.iter_mut() {
                        // 1c. a motor whose parent is another motor: its position becomes relative
                        // to its PARENT's origin, the frame the tree composes it with: p_child - p_parent,
                        // both in the model frame.
                        // Bug #9 (fixed in brick 2): the subtraction acted on a copy and was
                        // dropped, so a rotor kept its model-frame position as if it were relative
                        // to its arm. The mixer then placed the rotors at (0, +/-0.48, 0.20)
                        // instead of (0, +/-0.30, 0.10), and delivered only 0.30 / 0.48 = 62.5 % of
                        // the demanded roll moment (measured: 0.626, runs A and B).
                        if model.parent_id != model.id && model_positions.iter().any(|(motor_id, _)| *motor_id == model.parent_id) {
                            if let (Some(p_child), Some(p_parent))= (model_position(model.id), model_position(model.parent_id)) {
                                model.relative_location= Some(p_child - p_parent);
                            }
                        }
                        // 1d. otherwise it hangs under the body itself: the tilt joints' parent,
                        // base_link, is not a motor
                        else {
                            model.parent_id= 0;
                        }
                    }
                    // 1e. the tree, checked, written ONCE into the shared slot: from now on every
                    //     control stage reads this very instance, and the descriptions live there only
                    match MotorTree::new(std::mem::take(&mut discovered)) {
                        Ok(tree) => if motor_tree_cl.set(tree).is_err() {
                            println!("[WARNING] -> the motor tree was already configured: the discovered one is ignored");
                        },
                        Err(error) => println!("[ERROR] -> the discovered motor tree is refused: {}", error),
                    }
                }
                // 2. the measurements, with the descriptions of the tree (nothing before it exists)
                if let Some(tree)= motor_tree_cl.get() && let Ok(mut motors)= motors_cl.lock() {
                    let now= Timestamp {
                        seconds: Utc::now().timestamp(),
                        nanos: Utc::now().timestamp_subsec_nanos() as i32,
                    };
                    for j in &msg.joint {
                        // 2a. the motor of this joint: its description and its state
                        let name= if j.name.contains("tilt") { j.name.replace("tilt", "arm") }
                                  else if j.name.contains("spin") { j.name.replace("spin", "rotor") }
                                  else { continue };
                        let id= byte_sum(&name);
                        let (Some(model), Some(state))= (tree.get(id), motors.iter_mut().find(|state| state.id == id)) else {
                            continue;
                        };
                        // 2b. its effort in mixer units: an arm's angle (rad), a rotor's thrust (N) from
                        //     its measured speed through the effort law T = transmission * k * w^n
                        let value= if model.is_joint() { j.axis1.position }
                                   else { model.transmission_factor * model.effort_constant * j.axis1.velocity.powf(model.exp_command_law as f64) };
                        if verbose {
                            verbose_lines.push(format!("Received motor measurements: id: {}, name: {}, type: {}, value: {}", id, name, model.working_axis, value));
                        }
                        // 2c. recorded in its state (a non-finite value is refused), then published
                        state.record_measurement(model, value, MotorStatus::RUNNING, Some(now));
                        if let Some(mfs)= &motor_feedback_sender_cl && let Some(feedback)= &state.feedback {
                            let _= mfs.send(feedback.clone());
                        }
                    }
                }
                // every lock is released here: printing can no longer delay the control thread
                for line in verbose_lines {
                    println!("{}", line);
                }
            });
        }
    }

    /// Send telemetry data to the telemetry receiver linked to telemetry_sender
    /// 
    /// Most likely to an interface
    /// Publish the latest pose (updated by the IMU callback at 250 Hz, hence < 4 ms old) on the
    /// telemetry channel. Called first in `exec`, i.e. right before the attitude controller runs in
    /// the same scheduler pass: the attitude always consumes the measurement of the current pass.
    /// Prints the whole pose in verbose mode only, after the lock is released.
    fn send_telemetry(&mut self) {
        // 1. copy the pose and release the lock at once: the IMU / odometry callbacks wait on it
        let pose= match self.telemetry_state.lock() {
            Ok(telemetry_state) => telemetry_state.pose_state.clone(),
            Err(_) => return,
        };
        if let Some(sender) = &self.telemetry_sender {
            // 2. verbose only, and outside the lock
            if self.verbose {
                println!("Telemetry Sent:\n{:?} to {} receivers", pose, sender.receiver_count());
            }
            // 3. broadcast::Sender::send never blocks: it returns Err when nobody is subscribed,
            //    which is not an error here
            let _ = sender.send(AnyMessage::PoseState(pose));
        }
    }

}

impl Process for OspraiController {
    /// Execute process task
    /// 
    /// Send setpoint to the actuators
    /// 
    /// Send the telemetry data (vehicle state) to interface / process
    /// One cycle: (1) publish telemetry, (2) drain the command channel: convert each `MotorCommand`
    /// from efforts to actuator units - THRUST (N) -> rotor speed `w = (T / (transmission * k))^(1/n)`
    /// (rad/s), ANGULARPOSITION (rad) unchanged - in the `motors_command_sequence` order, and publish
    /// the `Actuators` message only when it is complete. Commands computed at pass k are therefore
    /// applied at pass k+1 (one-cycle delay, part of the actuation-delay budget).
    /// The setpoint is no longer published here: see `crate::scenarios::player`.
    /// NOTE: never publish a non-finite value: the MulticopterMotorModel keeps a NaN in its
    /// first-order filter for the rest of the run.
    fn exec(&mut self, input: &Option<AnyMessage>, dt: Duration) -> Option<AnyMessage> {
        // 1. the measured state of this pass, first, so every later process of the pass reads it
        self.send_telemetry();
        // 2. the motor commands of the previous pass, converted and published to Gazebo
        if let Some(cmds_rcvr) = &mut self.interface_cmd_receiver {
            let mut actuator_setpoints= Actuators::new();
            let sz= cmds_rcvr.len();
            for _ in 0..sz {
                if let Ok(cmd)= cmds_rcvr.try_recv() {
                    // the motors are read from the shared tree; not configured yet: nothing to convert
                    if let AnyMessage::MotorCommands(motor_cmds)= cmd && let Some(tree)= self.motor_tree.get() {
                        for motor_cmd in &motor_cmds {
                            let mut idx= 0;
                            //iterate over the id of motor in the actuator command sequence
                            for id in self.motors_command_sequence.clone() {
                                //get the motor model of the motor in the command sequence
                                if let Some(motor_model) = tree.get(id as u32) {
                                    if motor_cmd.command_type == MotorCommandType::THRUST as i32 &&  motor_model.is_thruster() {
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
                                    } else if motor_cmd.command_type == MotorCommandType::VELOCITY as i32 &&  motor_model.is_thruster() {
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
                                    } else if motor_cmd.command_type == MotorCommandType::ANGULARPOSITION as i32 &&  motor_model.is_joint() {
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
                        if let Some(motor_pub)= self.motors_command_publisher.as_mut() && 
                                (actuator_setpoints.position.len() + actuator_setpoints.velocity.len() == self.motors_command_sequence.len()){
                            let _= motor_pub.publish(&actuator_setpoints);
                            // the commands applied, kept in each motor's state
                            if let Ok(mut motors)= self.motors.lock() {
                                for motor_cmd in &motor_cmds {
                                    if let Some(state)= motors.iter_mut().find(|state| state.id == motor_cmd.id) {
                                        state.command= Some(motor_cmd.clone());
                                    }
                                }
                            }
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









/// The OSPRAI's reference configuration file, `vehicles/osprai/motors.json`: the form a real
/// robot's motor tree takes (`MotorTree::from_json`). Hand-predicted values: the tree this bridge
/// discovers from the scene and the first `joint_state` message (ids are the byte-sums of the link
/// names, positions come from the SDF, bounds and constants from `start_listening_topics`).
#[cfg(test)]
mod tests {
    use std::path::Path;

    use nalgebra::Vector3;
    use robomorph::control::motion::motor_tree::ForceSubspace;

    use super::*;

    #[test]
    fn the_reference_file_describes_the_discovered_osprai() {
        // 1. the file, read and checked like any robot configuration
        let path= Path::new(env!("CARGO_MANIFEST_DIR")).join("../vehicles/osprai/motors.json");
        let tree= MotorTree::from_json(&path).unwrap();
        // 2. the bridge's ids, parents first: the two arms, then the rotor each one carries
        assert_eq!(tree.order(), &[74, 189, 64, 179]);
        assert_eq!((tree.children(74), tree.children(189)), (&[64][..], &[179][..]));
        // 3. at rest the rotors are where the SDF puts them, (0, +/-0.30, 0.10) from the model origin:
        //    each one is given relative to its arm (fix #9)
        let transforms= tree.transforms(Vec3::new(0.0, 0.0, 0.0), |_| 0.0);
        for (id, y) in [(64, 0.30), (179, -0.30)] {
            let location= Vector3::from(transforms[&id].location);
            assert!((location - Vector3::new(0.0, y, 0.10)).norm() < 1e-12, "rotor {} at {:?}", id, location);
        }
        // 4. the rotors' bounds and constants: [0.1 N, k * 1200^2], spin signs of bug #12
        let (left, right)= (tree.get(64).unwrap(), tree.get(179).unwrap());
        assert_eq!((left.moment_constant, right.moment_constant), (-0.016, 0.016));
        assert!(left.min_value == 0.1 && (left.max_value - 1.2e-5 * 1200f64.powi(2)).abs() < 1e-9, "{:?}", left);
        // 5. S, the xz plane: a roll is refused, like in the scenario roll_refused
        assert!(matches!(tree.force_subspace(), ForceSubspace::Plane(_)));
    }
}
