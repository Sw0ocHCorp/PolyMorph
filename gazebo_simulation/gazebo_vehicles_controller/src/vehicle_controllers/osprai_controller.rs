use std::{sync::{Arc, Mutex}, time::Duration};

use chrono::Utc;
use gz::{msgs::{actuators::Actuators, double::Double, image::Image, imu::IMU, laserscan::LaserScan, model::Model, navsat::NavSat, quaternion::Quaternion}, transport::Node};
use robomorph::{communications::interface::HardwareInterface, core::scheduler::Process, messages::{lidar_messages::{LidarMeasurements, Ray}, motor_messages::{MotorCommandType, MotorFeedBack, MotorStatus}, pose_messages::{GNSSMeasurement, IMUMeasurements}, registered_message::AnyMessage}};
use tokio::sync::{broadcast, mpsc};

use crate::vehicle_controllers::vehicle_controller::VehicleController;

pub fn quaternion_to_euler(quat: &Quaternion) -> [f64; 3] {
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
    /// Imu 9 DOFs raw measurements
    pub imu_measurements: IMUMeasurements,
    /// GNSS raw measurements 
    pub gnss_measurements: GNSSMeasurement,
    /// Camera sources specs and image
    pub cameras: Vec<ImageState>,
    /// Arm's Motors data 
    pub arms: Vec<MotorFeedBack>,
    /// Propeller's Motors data 
    pub propellers: Vec<MotorFeedBack>
}

impl OsprAiState {
    pub fn new_with_lidar() -> Self {
        OsprAiState {
            gnss_measurements: GNSSMeasurement { latitude: 0.0, longitude: 0.0, altitude: 0.0, fix_status: 1 },
            imu_measurements: IMUMeasurements {
                l_accel_x: 0.0, l_accel_y: 0.0, l_accel_z: 0.0,
                a_velocity_x: 0.0, a_velocity_y: 0.0, a_velocity_z: 0.0,
                magnetic_field_x: 0.0, magnetic_field_y: 0.0, magnetic_field_z: 0.0,
            },
            lidar_measurements: LidarMeasurements {
                vertical_min_angle: 0.0, vertical_angle_resolution: 0.0, vertical_max_angle: 0.0,
                horizontal_min_angle: 0.0, horizontal_angle_resolution: 0.0, horizontal_max_angle: 0.0,
                rays: vec![],
            },
            cameras: vec![
                ImageState { timestamp: 0, cam_label: "camera_left".to_string(), width: 0, height: 0, channel: 0, format: "rgb".to_string(), data: vec![] },
                ImageState { timestamp: 0, cam_label: "camera_right".to_string(), width: 0, height: 0, channel: 0, format: "rgb".to_string(), data: vec![] },
            ],
            arms: vec![],
            propellers: vec![],
        }
    }
}

/// Struct that represent the controller's of the osprai vehicle
/// 
/// can send telemetry data to an Hardware Interface to communicate with other softwares, receive command frames from the hardware interface
/// 
/// can apply commands on the hardware actuators or Gazebo simulated vehicle
pub struct OspraiController {
    /// Telemetry state that aggregate all the telemtry data
    pub telemetry_state: Arc<Mutex<OsprAiState>>,
    /// The node that allow the controller to connect itself to Gazebo simulation environment
    controller_node: Option<Node>,
    /// Receiver of the cmd from Hardware Interface
    interface_cmd_receiver: Option<broadcast::Receiver<AnyMessage>>,
    /// Telemetry sender to Hardware Interface
    telemetry_sender: Option<mpsc::Sender<AnyMessage>>,
}

impl OspraiController {
    /// Constructor
    /// 
    /// Arguments:
    /// 
    /// cmd_receiver: A vehicle commands receiver
    pub fn new(cmd_receiver: broadcast::Receiver<AnyMessage>) -> Self {
        Self { telemetry_state: Arc::new(Mutex::new(OsprAiState::new_with_lidar())),
                controller_node: None,
                interface_cmd_receiver: Some(cmd_receiver),
                telemetry_sender: None,
        }
    }
}

impl Default for OspraiController {
    /// Default constructor
    fn default() -> Self {
        Self { telemetry_state: Arc::new(Mutex::new(OsprAiState::new_with_lidar())),
                controller_node: None,
                interface_cmd_receiver: None,
                telemetry_sender: None,
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
            let mut osprai_state_imu= self.telemetry_state.clone();
            let _= node.subscribe("osprai/imu", move |msg : IMU| {
                if let Ok(mut telemetry_state) = osprai_state_imu.clone().lock() {
                    telemetry_state.imu_measurements.l_accel_x= msg.linear_acceleration.x;
                    telemetry_state.imu_measurements.l_accel_y= msg.linear_acceleration.y;
                    telemetry_state.imu_measurements.l_accel_z= msg.linear_acceleration.z;
                    telemetry_state.imu_measurements.a_velocity_x= msg.angular_velocity.x;
                    telemetry_state.imu_measurements.a_velocity_y= msg.angular_velocity.y;
                    telemetry_state.imu_measurements.a_velocity_z= msg.angular_velocity.z;
                }
                
            });
            let mut osprai_state_gnss= self.telemetry_state.clone();
            let _= node.subscribe("osprai/gps", move |msg : NavSat| {
                if let Ok(mut telemetry_state) = osprai_state_gnss.clone().lock() {
                    telemetry_state.gnss_measurements.latitude = msg.latitude_deg;
                    telemetry_state.gnss_measurements.longitude = msg.longitude_deg;
                    telemetry_state.gnss_measurements.altitude = msg.altitude;
                }
            });
            let mut osprai_state_lidar= self.telemetry_state.clone();
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
            let mut osprai_state_lc= self.telemetry_state.clone();
            let _= node.subscribe("osprai/cameras/left", move |msg: Image| {
                if let Ok(mut telemetry_state) = osprai_state_lc.clone().lock() {
                    telemetry_state.cameras[0]= ImageState { timestamp: Utc::now().timestamp_millis(), cam_label: "camera_left".to_string(), 
                                                    width: msg.width, height: msg.height, channel: 3, format: "rgb".to_string(), data: msg.data.clone() };
                }
            });
            let mut osprai_state_rc= self.telemetry_state.clone();
            let _= node.subscribe("osprai/cameras/right", move |msg: Image| {
                if let Ok(mut telemetry_state) = osprai_state_rc.clone().lock() {
                    telemetry_state.cameras[0]= ImageState { timestamp: Utc::now().timestamp_millis(), cam_label: "camera_right".to_string(), 
                                                    width: msg.width, height: msg.height, channel: 3, format: "rgb".to_string(), data: msg.data.clone() };
                }
            });
            let mut osprai_state_motors= self.telemetry_state.clone();
            let _= node.subscribe("/osprai/joint_state", move |msg: Model| {
                if let Ok(mut telemetry_state) = osprai_state_motors.clone().lock() {
                    for i in 0..msg.joint.len() {
                        let j= &msg.joint[i];
                        if let Some(axis1) = j.axis1.clone().into_option() {
                            if j.name.contains("spin") {
                                let mut is_new= true;
                                for propeller in &mut telemetry_state.propellers {
                                    if propeller.id == j.id {
                                        is_new= false;
                                        propeller.current_value = axis1.velocity;
                                        break;
                                    }
                                }
                                if is_new {
                                    let feedback= MotorFeedBack {
                                        id: j.id,
                                        current_value: axis1.velocity,
                                        setpoint_value: axis1.velocity,
                                        min_value: 0.0,
                                        max_value: 1200.0,
                                        command_type: MotorCommandType::VELOCITY as i32,
                                        status: MotorStatus::IDLE as i32,
                                        p: 1.0,
                                        i: 0.0,
                                        d: 0.0, 
                                        control_frequency: 50
                                    };
                                    telemetry_state.propellers.push(feedback);
                                }
                            } else {
                                let mut is_new= true;
                                for arm in &mut telemetry_state.arms {
                                    if arm.id == j.id {
                                        is_new= false;
                                        arm.current_value = axis1.position;
                                        //println!("Joint {} Current Angle: {:.2}°", j.name, joint_state.current_value.to_degrees());
                                        break;
                                    }
                                }
                                if is_new {
                                    telemetry_state.arms.push(MotorFeedBack {
                                        id: j.id,
                                        current_value: axis1.position,
                                        setpoint_value: f64::to_radians(-90.0),
                                        min_value: axis1.limit_lower,
                                        max_value: axis1.limit_upper,
                                        command_type: MotorCommandType::ANGULARPOSITION as i32,
                                        status: MotorStatus::IDLE as i32,
                                        p: 1.0, // Proportional gain
                                        i: 0.0, // Integral gain
                                        d: 0.0, // Derivative gain
                                        control_frequency: 50,
                                    });
                                }
                            }
                        }
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
    fn apply_actuator_setpoints(&mut self, setpoints: Vec<AnyMessage>) {
        if let Ok(state) = self.telemetry_state.clone().lock() && 
                let Some(node) = & mut self.controller_node{
            if state.arms.len() == 2 && state.propellers.len() == 2 {
                if let Some(mut left_arm) = node.advertise::<Double>("/osprai/command/tilt_left") {
                    let mut msg = Double::new();
                    msg.data = state.arms[0].setpoint_value;
                    let _ = left_arm.publish(&msg);
                }
                if let Some(mut right_arm) = node.advertise::<Double>("/osprai/command/tilt_right") {
                    let mut msg = Double::new();
                    msg.data = state.arms[1].setpoint_value;
                    let _ = right_arm.publish(&msg);
                }
            }
        }
    }

    /// Send telemetry data to the telemetry receiver linked to telemetry_sender
    /// 
    /// Most likely to an interface
    fn send_telemetry(&mut self) {
        if let Some(sender) = &mut self.telemetry_sender &&
            let Ok(telemetry_state)= self.telemetry_state.lock() {
            let _ = sender.try_send(AnyMessage::ImuState(telemetry_state.imu_measurements.clone()));
            let _ = sender.try_send(AnyMessage::GnssState(telemetry_state.gnss_measurements.clone()));
            for idx in 0..telemetry_state.arms.len() {
                let _ = sender.try_send(AnyMessage::MotorState(telemetry_state.arms[idx].clone()));
            }
            for idx in 0..telemetry_state.propellers.len() {
                let _ = sender.try_send(AnyMessage::MotorState(telemetry_state.propellers[idx].clone()));
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
    fn exec(&mut self, inputs: Option<AnyMessage>, _dt: Duration) -> Option<AnyMessage> {
        self.apply_actuator_setpoints(inputs.into_iter().collect());
        self.send_telemetry();
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
}








