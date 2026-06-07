use std::{ops::Add, sync::{Arc, Mutex}, thread::sleep, time::{Duration, Instant}, u8};

use gz::{msgs::{actuators::Actuators, double::Double, imu::IMU, laserscan::LaserScan, model::Model, navsat::NavSat, quaternion::Quaternion}, transport::Node};

struct LidarState {
    min_vertical_angle: f64,
    vertical_resolution: f64,
    min_horizontal_angle: f64,
    horizontal_resolution: f64,
    measurements: Vec<f64>,
}

#[derive(Clone)]
struct ServoMotorState {
    id: u32,
    current_angle: f64,
    setpoint_angle: f64,
    min_angle: f64,
    max_angle: f64,
    p: f64,
    i: f64,
    d: f64,
    max_integral: f64,
}

impl ServoMotorState {
    fn new() -> Self {
        return ServoMotorState {
            id: 0,
            current_angle: 0.0,
            setpoint_angle: 0.0,
            min_angle: (-180.0_f64).to_radians(), // Example minimum angle
            max_angle: 180.0_f64.to_radians(),  // Example maximum angle
            p: 1.0, // Proportional gain
            i: 0.0, // Integral gain
            d: 0.0, // Derivative gain
            max_integral: 10.0, // Maximum integral term to prevent windup
        };
    }
    
}

#[derive(Clone)]
struct MotorState {
    id: u32,
    current_thrust: f64,
    setpoint_thrust: f64,
    max_thrust: f64,
    p: f64,
    i: f64,
    d: f64,
    max_integral: f64,
}

impl MotorState {
    fn new() -> Self {
        return MotorState {
            id: 0,
            current_thrust: 0.0,
            setpoint_thrust: 0.0,
            max_thrust: 100.0,  // Example maximum thrust
            p: 1.0, // Proportional gain
            i: 0.0, // Integral gain
            d: 0.0, // Derivative gain
            max_integral: 100.0, // Maximum integral term to prevent windup
        };
    }

}

struct OsprAiState {
    joints_angles: Vec<ServoMotorState>,
    thrusters: Vec<MotorState>,
    linear_acceleration: [f64; 3],
    angular_velocity: [f64; 3],
    attitude: [f64; 3],
    latitude: f64,
    longitude: f64,
    altitude: f64,
    lidar_state: Option<LidarState>,
}

impl OsprAiState {
    /*fn new(bool: contains_lidar, u8: n_thrusters, u) -> Self {
        return OsprAiState {
            joints_angles: vec![(0.0, 0.0); 2], // Assuming 2 controllable joints (left and right arms)
            linear_acceleration: [0.0; 3],
            angular_velocity: [0.0; 3],
            latitude: 0.0,
            longitude: 0.0,
            altitude: 0.0,
            lidar_state: None,
        };
    }*/

    fn new_with_lidar() -> Self {
        let mut state= OsprAiState {
            joints_angles: vec![],
            thrusters: vec![], // Assuming 2 thrusters
            linear_acceleration: [0.0; 3],
            angular_velocity: [0.0; 3],
            attitude: [0.0; 3],
            latitude: 0.0,
            longitude: 0.0,
            altitude: 0.0,
            lidar_state: Some(LidarState {
                min_vertical_angle: 0.0,
                vertical_resolution: 0.0,
                min_horizontal_angle: 0.0,
                horizontal_resolution: 0.0,
                measurements: vec![],
            }),
        };
        for thruster in &mut state.thrusters {
            thruster.max_thrust = 1200.0;
        }
        return state;
    }
    
}

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

fn main() {
    let osprai_state= Arc::new(Mutex::new(OsprAiState::new_with_lidar()));
    if let Some(mut node)=  Node::new() {
        let osprai_state_imu = Arc::clone(&osprai_state);
        let _= node.subscribe("osprai/imu", move |msg : IMU| {
            if let Ok(mut state) = osprai_state_imu.lock() {
                state.linear_acceleration = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z];
                state.angular_velocity = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z];
                state.attitude= quaternion_to_euler(&msg.orientation);
            }
        });
        let osprai_state_gps = Arc::clone(&osprai_state);
        let _= node.subscribe("osprai/gps", move |msg : NavSat| {
            if let Ok(mut state) = osprai_state_gps.lock() {
                state.latitude = msg.latitude_deg;
                state.longitude = msg.longitude_deg;
                state.altitude = msg.altitude;
            }
        });
        let osprai_state_lidar = Arc::clone(&osprai_state);
        let _= node.subscribe("osprai/lidar", move |msg: LaserScan| {
            if let Ok(mut state) = osprai_state_lidar.lock() && let Some(lidar_state) = &mut state.lidar_state {
                lidar_state.min_vertical_angle = msg.vertical_angle_min;
                lidar_state.vertical_resolution = msg.vertical_angle_step;
                lidar_state.min_horizontal_angle = msg.angle_min;
                lidar_state.horizontal_resolution = msg.angle_step;
                lidar_state.measurements= msg.ranges.clone();
                //lidar_state.measurements = msg.measurements.clone();
            
            }
        });
        /*let _= node.subscribe("osprai/lidar/points", move |msg: PointCloudPacked| {
            println!("Received LIDAR message: {:?}", msg);
        });*/
        let osprai_state_joint = Arc::clone(&osprai_state);
        let _= node.subscribe("/osprai/joint_state", move |msg: Model| {
            if let Ok(mut state) = osprai_state_joint.lock(){
                for i in 0..msg.joint.len() {
                    let j= &msg.joint[i];
                    if let Some(axis1) = j.axis1.clone().into_option() {
                        if j.name.contains("spin") {
                            let mut is_new= true;
                            for thruster in &mut state.thrusters {
                                if thruster.id == j.id {
                                    is_new= false;
                                    thruster.current_thrust = axis1.velocity;
                                    break;
                                }
                            }
                            if is_new {
                                state.thrusters.push(MotorState {
                                    id: j.id,
                                    current_thrust: axis1.velocity,
                                    setpoint_thrust: axis1.velocity,
                                    max_thrust: 1200.0, // Example maximum thrust
                                    p: 1.0, // Proportional gain
                                    i: 0.0, // Integral gain
                                    d: 0.0, // Derivative gain
                                    max_integral: 100.0, // Maximum integral term to prevent windup
                                });
                            }
                        } else {
                            let mut is_new= true;
                            for joint_state in &mut state.joints_angles {
                                if joint_state.id == j.id {
                                    is_new= false;
                                    joint_state.current_angle = axis1.position;
                                    //println!("Joint {} Current Angle: {:.2}°", j.name, joint_state.current_angle.to_degrees());
                                    break;
                                }
                            }
                            if is_new {
                                state.joints_angles.push(ServoMotorState {
                                    id: j.id,
                                    current_angle: axis1.position,
                                    setpoint_angle: f64::to_radians(-0.0),
                                    min_angle: axis1.limit_lower,
                                    max_angle: axis1.limit_upper,
                                    p: 1.0, // Proportional gain
                                    i: 0.0, // Integral gain
                                    d: 0.0, // Derivative gain
                                    max_integral: 10.0, // Maximum integral term to prevent windup
                                });
                            } 
                        }
                    } 
                    
                }
            }
        });
 
        if let Some(mut left_arm) = node.advertise::<Double>("/osprai/command/tilt_left") &&
                let Some(mut right_arm) = node.advertise::<Double>("/osprai/command/tilt_right") &&
                let Some(mut thrusters) = node.advertise::<Actuators>("/osprai/command/motor_speed") {
            let mut start = Instant::now();
            let dt = Duration::from_millis(20);
            let osprai_state_cmd = Arc::clone(&osprai_state);
            loop {
                if let Ok(mut state) = osprai_state_cmd.lock() {
                    if state.joints_angles.len() >= 2 {
                        let mut left_setpoint = Double::new();
                        left_setpoint.data = state.joints_angles[0].setpoint_angle; // Setpoint angle for left arm
                        let mut right_setpoint = Double::new();
                        right_setpoint.data = state.joints_angles[1].setpoint_angle; // Setpoint angle for right arm
                        let _= left_arm.publish(&left_setpoint);
                        let _= right_arm.publish(&right_setpoint);
                        println!("Current Left Arm Angle: {:.2}°, Current Right Arm Angle: {:.2}°", state.joints_angles[0].current_angle.to_degrees(), state.joints_angles[1].current_angle.to_degrees());
                        println!("Published Left Arm Setpoint: {:.2}°, Right Arm Setpoint: {:.2}°", left_setpoint.data.to_degrees(), right_setpoint.data.to_degrees());
                    }
                    println!("Attitude: | {:.4}° roll | {:.4}° pitch | {:.4}° yaw |", state.attitude[0].to_degrees(), state.attitude[1].to_degrees(), state.attitude[2].to_degrees());
                    /*let mut idx= 0;
                    let mut thrusters_command = Actuators::new();
                    for thruster in &mut state.thrusters {
                        if (idx % 2 == 0) {
                            thruster.setpoint_thrust= 1000.0
                        } else {
                            thruster.setpoint_thrust= -1000.0
                        }
                        thrusters_command.velocity.push(thruster.setpoint_thrust); 
                        println!("Thruster {} Current Thrust: {:.2}, Setpoint Thrust: {:.2}", idx, thruster.current_thrust, thruster.setpoint_thrust);
                        idx+=1;
                    }
                    let _= thrusters.publish(&thrusters_command);*/
                }
                let elapsed = start.elapsed();
                if elapsed < dt {
                    //println!("Sleep for {:?}", dt - elapsed);
                    sleep(dt - elapsed);
                    start= start.add(dt); // Reset start time for the next iteration
                }
            }
        }
    }
        //ret= node.subscribe("osprai/tilt_right", callback);
        
}

