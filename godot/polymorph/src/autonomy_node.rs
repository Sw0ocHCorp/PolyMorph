use std::{collections::HashMap, sync::{Arc, Mutex}, time::{self, Instant, SystemTime}};

use godot::{classes::{Engine, InputEvent, InputEventJoypadButton, InputEventJoypadMotion, Performance, PhysicsRayQueryParameters3D, RigidBody3D, editor_vcs_interface::ChangeType}, global::{JoyAxis, JoyButton, atan2, cos, sin}, prelude::*};
use ordered_float::OrderedFloat;
use robomorph::{communication::UDPChannel, core::{messages::{self, DataChunk, SOF}, utils, worker::Worker}, lidar_management::measurements::LidarMeasurements, positionning::pose::{GPSData, IMUData}};

const ORIGIN_GPS_DATA: GPSData= GPSData{longitude: 5.7932940640423904, latitude: 43.25157380084422};
const REF_WORLD_MAGNETIC_FIELD: [f64; 3] = [15.3017, 0.4328527, -41.06483];
const GRAVITY: Vector3= Vector3{x: 0.0, y: -9.81, z: 0.0};

#[derive(GodotClass)]
#[class(init, base=Node3D)]
pub struct AutonomyNode {
    #[base]
    base: Base<Node3D>,
    udp_worker: Option<Arc<Worker>>,
    debug_worker: Option<Arc<Worker>>,
    lidar_points: u32,
    lidar_fov: f64,
    lidar_angle_offset: f64,
    motion_command: [f32; 2],
    motion_factor: f32,
    last_linear_vel: Vector3,
    world_magnetic_field: Vector3,
    data_fps: u32,
    gps_frequency: u32,
    dt: f64,
    gps_dt: f64
}
#[godot_api]
/**
 * /!\ WARNING: Execute tasks sequentially. Not in a dedicated thread
 * /!\ Unable to maintain a dedicated thread for  GodotClass  
 */
impl INode3D for AutonomyNode{

    fn ready(&mut self) {
        //let test= vec![]
        self.world_magnetic_field= Vector3 { x: REF_WORLD_MAGNETIC_FIELD[0] as f32, y: REF_WORLD_MAGNETIC_FIELD[2] as f32, z: REF_WORLD_MAGNETIC_FIELD[1] as f32 };
        self.world_magnetic_field= self.world_magnetic_field.normalized();
        let udp= UDPChannel::new_async("127.0.0.1", 8080, "127.0.0.1", 8090);
        let udp_debug= UDPChannel::new_async("127.0.0.1", 9010, "127.0.0.1", 9000);
        //Get and store the metadata of the 
        if self.base().has_meta("lidarPoints") {
            match self.base().get_meta("lidarPoints").try_to::<i32>() {
                Ok(lidar_points) => {
                    if lidar_points > 0 {
                        self.lidar_points= lidar_points as u32;
                        godot_print!("Loading lidarPoints metadata succesful, It's value is= {}", lidar_points);
                    } else {
                        godot_print!("/!\\ ERROR: lidarPoints metadata must be > 0.\nYou should change its value");
                        self.lidar_points= 100;
                    }
                },
                Err(_) => {
                    godot_print!("/!\\ ERROR: lidarPoints conversion in i32 failed.\nYou should change the metadata type in int");
                    self.lidar_points= 100;
                },
            } 
        } else {
            godot_print!("/!\\ ERROR: No lidarPoints metadata exist.\nYou should add a metadata called \"lidarPoints\" with int as type");
            self.lidar_points= 100;
        }
        if self.base().has_meta("lidarFov") {
            match self.base().get_meta("lidarFov").try_to::<i32>() {
                Ok(lidar_fov) => {
                    self.lidar_fov= (lidar_fov as f64).to_radians();
                    godot_print!("Loading lidarFov metadata succesful, It's value is= {}°", lidar_fov);
                },
                Err(_) => {
                    godot_print!("/!\\ ERROR: lidarFov conversion in i32 failed.\nYou should change the metadata type in int");
                    self.lidar_fov= (180 as f64).to_radians();
                },
            } 
        } else {
            godot_script_error!("/!\\ ERROR: No lidarFov metadata exist.\nYou should add a metadata called \"lidarFov\" with int as type");
            self.lidar_fov= (180 as f64).to_radians();
        }
        if self.base().has_meta("lidarAngleOffset") {
            match self.base().get_meta("lidarAngleOffset").try_to::<i32>() {
                Ok(lidar_angle_offset) => {
                    self.lidar_angle_offset= (lidar_angle_offset as f64).to_radians();
                    godot_print!("Loading lidarAngleOffset metadata succesful, It's value is= {}°", lidar_angle_offset);
                },
                Err(_) => {
                    godot_print!("/!\\ ERROR: lidarAngleOffset conversion in i32 failed.\nYou should change the metadata type in int");
                    self.lidar_angle_offset= 0 as f64;
                },
            } 
        } else {
            godot_script_error!("/!\\ ERROR: No lidarAngleOffset metadata exist.\nYou should add a metadata called \"lidarFov\" with int as type");
            self.lidar_angle_offset= 0 as f64;
        }
        if self.base().has_meta("motionFactor") {
            match self.base().get_meta("motionFactor").try_to::<f32>() {
                Ok(motion_factor) => {
                    self.motion_factor= motion_factor;
                    godot_print!("Loading motionFactor metadata succesful, It's value is= {}", motion_factor);
                },
                Err(_) => {
                    godot_print!("/!\\ ERROR: motionFactor conversion in f32 failed.\nYou should change the metadata type in float");
                    self.motion_factor= 1.0;
                },
            } 
        } else {
            godot_script_error!("/!\\ ERROR: No motionFactor metadata exist.\nYou should add a metadata called \"motionFactor\" with float as type");
            self.motion_factor= 1.0;
        }
        if self.base().has_meta("dataFrequency") {
            match self.base().get_meta("dataFrequency").try_to::<u32>() {
                Ok(data_frequency) => {
                    self.data_fps= data_frequency;
                    godot_print!("Loading dataFrequency metadata succesful, It's value is= {}", data_frequency);
                },
                Err(_) => {
                    godot_print!("/!\\ ERROR: dataFrequency conversion in f32 failed.\nYou should change the metadata type in u32");
                    self.data_fps= 10;
                },
            } 
        } else {
            godot_script_error!("/!\\ ERROR: No dataFrequency metadata exist.\nYou should add a metadata called \"dataFrequency\" with u32 as type");
            self.data_fps= 10;
        }
        if self.base().has_meta("gpsFrequency") {
            match self.base().get_meta("gpsFrequency").try_to::<u32>() {
                Ok(data_frequency) => {
                    self.gps_frequency= data_frequency;
                    godot_print!("Loading gpsFrequency metadata succesful, It's value is= {}", data_frequency);
                },
                Err(_) => {
                    godot_print!("/!\\ ERROR: gpsFrequency conversion in f32 failed.\nYou should change the metadata type in u32");
                    self.gps_frequency= 1;
                },
            } 
        } else {
            godot_script_error!("/!\\ ERROR: No dataFrequency metadata exist.\nYou should add a metadata called \"dataFrequency\" with u32 as type");
            self.data_fps= 10;
        }
        self.udp_worker= Some(Worker::new(udp, "UDP_WORKER", self.data_fps as i64, false));
        self.debug_worker= Some(Worker::new(udp_debug, "DEBUG_UDP_WORKER", self.data_fps as i64, false));

        //let fps = Engine::get_max_fps(&self);
        //godot_print!("GAME FPS= {}", fps);
    }

    fn input(&mut self, event: Gd < InputEvent >,) {
        //godot_print!("Other input event detected: {:?}\n", event.clone());
        match event.clone().try_cast::<InputEventJoypadButton>() {
            Ok(button_event) => {
                if button_event.get_button_index() == JoyButton::A {
                    self.motion_command= [0.0, 0.0];
                }
                if button_event.get_button_index() == JoyButton::X {
                    self.motion_command= [0.0, 1.0];
                }
                if button_event.get_button_index() == JoyButton::B {
                    self.motion_command= [0.0, -1.0];
                }
                //godot_print!("Joypad button detected: {:?}\n", button_event.get_button_index());
            },
            Err(_) => {

            },
        }
        match event.clone().try_cast::<InputEventJoypadMotion>() {
            Ok(joypad_event) => {
                let value= joypad_event.get_axis_value();
                if value.abs() > 0.2 {
                    if joypad_event.get_axis() == JoyAxis::LEFT_X {
                        self.motion_command[1]= -1.0;
                    } 
                    if joypad_event.get_axis() == JoyAxis::LEFT_Y {
                        self.motion_command[0]= -1.0;
                    }
                    if joypad_event.get_axis() == JoyAxis::RIGHT_X {
                        self.motion_command[1]= 1.0;   
                    }
                    if joypad_event.get_axis() == JoyAxis::RIGHT_Y {
                        self.motion_command[0]= 1.0;
                    }

                } else {
                    self.motion_command= [0.0, 0.0];
                }
                //godot_print!("Joypad motion detected: {:?} {}\n", joypad_event.get_axis(), joypad_event.get_axis_value());
            },
            Err(_) => { 
                
            },
        }
    }

    fn physics_process(&mut self, delta: f64) {
        //godot_print!("Motion command: {:?}\n", self.motion_command);
        //godot_print!("{}\n", delta);
        //Detect the collision point between the raycast and the rigidBodies in the scene
        //let mut imu_measurements= IMUData
        self.dt += delta;
        self.gps_dt += delta;
        let mut measurements= LidarMeasurements::new(true);
        let mut imu_data= IMUData::new();
        let mut angle_offset= self.lidar_angle_offset;
        let mut true_orientation:[f32; 3]= [0.0, 0.0, 0.0];
        //IF this node had a parent
        if let Some(mut parent_obj) = self.base().get_parent() {
            //Generate lidar_points times raycast measurement for lidar_fov°
            match parent_obj.clone().try_cast::<RigidBody3D>() {
                Ok(mut robot) => {
                    let rob_orientation= robot.get_global_rotation();
                    
                    true_orientation= [rob_orientation.x, rob_orientation.z, rob_orientation.y];
                    // 1. Calculate Linear Acceleration in Godot World Space
                    // We add gravity because an IMU at rest measures the "Normal Force" pushing UP.
                    let mut godot_linear_accel = (robot.get_linear_velocity() - self.last_linear_vel) / (delta as f32) - GRAVITY;
                    godot_linear_accel /= GRAVITY.y.abs();
                    self.last_linear_vel = robot.get_linear_velocity();

                    // 2. Get Rotation and Angular Velocity
                    let robot_q = robot.get_quaternion();
                    let godot_angular_vel = robot.get_angular_velocity();

                    // 3. Bring World Vectors into the Robot's Local Body Frame
                    // Godot's '*' operator for Quaternions/Vectors handles the rotation.
                    let mut local_accel = robot_q.inverse() * godot_linear_accel;
                    let local_mag   = robot_q.inverse() * self.world_magnetic_field;
                    let local_gyro  = robot_q.inverse() * godot_angular_vel; // If angular_vel is in world coords
                    local_accel= local_accel.normalized();
                    // 4. Map Godot (Y-Up) to IMU (Z-Up) Convention
                    // Godot X -> IMU X (Forward)
                    // Godot Y -> IMU Z (Up)
                    // Godot -Z -> IMU Y (Left) - This maintains a Right-Handed System
                    //local_accel= local_accel.normalized();
                    imu_data = IMUData {
                        accel: [
                            local_accel.x as f64, 
                            -local_accel.z as f64, 
                            local_accel.y as f64
                        ], 
                        gyro: [
                            local_gyro.x as f64, 
                            -local_gyro.z as f64, 
                            local_gyro.y as f64
                        ],
                        magnetic_field: [
                            local_mag.x as f64, 
                            -local_mag.z as f64, 
                            local_mag.y as f64
                        ],
                        elapsed_time: self.dt
                    };
                    godot_print!("Linear Accel:\n{:?}", godot_linear_accel);
                    godot_print!("IMU Data:\n{:?}", imu_data);
                    //Compute the velocities to apply to the robot from the joystick input
                    //let translation= Vector3 { x: self.motion_command[0]*self.motion_factor, y: 0.0, z: 0.0 };
                    let relative_rotation= Vector3 { x: 0.0, y: self.motion_command[1]*self.motion_factor, z: 0.0 };
                    
                    //Applying the velocity computed from the joystick input
                    //robot.set_linear_velocity(translation);
                    robot.set_angular_velocity(relative_rotation);
                    angle_offset += robot.get_global_rotation().y as f64;

                    for i in 0..self.lidar_points{
                        //let mut base_angle= (-self.lidar_fov / 2.0) + ((i as f64 / (self.lidar_points-1.0)) * self.lidar_fov);
                        let angle = utils::modulo_pi_f64((-self.lidar_fov / 2.0) + ((i as f64 / (self.lidar_points-1) as f64) * self.lidar_fov));
                        //let angle= utils::modulo_2pi_f64(base_angle);
                        //godot_print!("ANGLE {}", angle.to_degrees());
                        //IF the scene exist
                        if let Some(mut world)= self.base().get_world_3d() {
                            //IF the space_state (to test raycast) exist
                            if let Some(mut space_state)= world.get_direct_space_state(){
                                //If the parent is a Node3D
                                if let Ok(parent)= parent_obj.clone().try_cast::<Node3D>() {
                                    //Get the position of the parent
                                    let origin= parent.get_global_position();
                                    // generate a raycast of 50m with the specific angle
                                    let mut raycast = PhysicsRayQueryParameters3D::create(origin, origin + Vector3{x: 50.0*cos(utils::modulo_pi_f64(-(angle + angle_offset))) as f32, y: 0.5, z: 50.0*sin(utils::modulo_pi_f64(-(angle+ angle_offset))) as f32});
                                    //Get the collision of the raycast and Rigidbodies of in the scene
                                    let collision= space_state.intersect_ray(raycast.as_ref());
                                    //MATCH: there is a collider position? (means there is a collision with a RigidBody?)
                                    match collision.get("position") {
                                        //There is a collision with a RigidBody
                                        Some(pos_variant) => {
                                            match pos_variant.try_to::<Vector3>() {
                                                Ok(pos) => {
                                                    //Add the distance with the rigidbody in the list, keyed by angle
                                                    let distance = origin.distance_to(pos);
                                                    //godot_print!("Distance at angle {}rad {}° => {}", angle, angle.to_degrees(), distance);
                                                    if distance < 0.0 {
                                                        godot_print!("Error: Negative distance detected");
                                                    }
                                                    measurements.insert(angle.to_degrees(), distance as f64);
                                                    //measurements.push(origin.distance_to(pos));
                                                },
                                                Err(_) => {
                                                    godot_print!("Error: Collider position is not a Vector3D");
                                                },
                                            }
                                        },
                                        //No RigidBody detected
                                        None => {

                                        }
                                    }
                                }
                            }
                        }    
                    }

                    //IF the UDP module exist
                    if let Some(udp_worker)= &self.udp_worker && let Some(debug_udp) = &self.debug_worker {
                        if measurements.len() > 0 {
                            if self.dt >= 1.0 / udp_worker.get_frequency() as f64 {
                                if self.dt > 1.0 / udp_worker.get_frequency() as f64 *1.15 {
                                    godot_print!("Send Late\n")
                                }
                                //godot_print!("TRIG => Elapsed Time= {} |TIME BETWEEN FRAMES= {}", self.dt, delta);
                                self.dt= 0.0;
                                match udp_worker.get_module().downcast_ref::<UDPChannel>() {
                                    Some(udp) => {
                                        let gps_data= utils::local_to_global_frame(ORIGIN_GPS_DATA, robot.get_global_position().x as f64, robot.get_global_position().y as f64);
                                        if self.gps_dt >= 1.0 / self.gps_frequency as f64 {
                                            let mut gps_frame= messages::convert_to_frame(vec![Box::new(gps_data)]);
                                            udp.publish_message(gps_frame);
                                            self.gps_dt= 0.0;
                                        }
                                        let mut imu_frame=messages::convert_to_frame(vec![Box::new(imu_data)]);
                                        //debug_udp= rob_orientation.to_string();
                                        match debug_udp.get_module().downcast_ref::<UDPChannel>() {
                                            Some(udp_debug)=>{udp_debug.publish_message(Vec::from(rob_orientation.to_string()));}
                                            None => todo!(),
                                        }
                                        //godot_print!("Send IMU");
                                        //godot_print!(" => {:?}\n", frame);
                                        //godot_print!("= {:?}\n", frame);
                                        udp.publish_message(imu_frame.clone());
                                        /*let frame= messages::convert_to_frame(vec![Box::new(measurements)]);
                                        udp.publish_message(frame.clone());*/
                                    },
                                    None => {

                                    },
                                }
                                udp_worker.force_run();
                                debug_udp.force_run();
                            }
                            /*if udp_worker.try_run() {
                                godot_print!("Elapsed Time= {}", self.dt);
                                self.dt= 0.0;
                                match udp_worker.get_module().downcast_ref::<UDPChannel>() {
                                    Some(udp) => {
                                        let mut imu_frame=messages::convert_to_frame(vec![Box::new(imu_data)]);
                                        //godot_print!("Send IMU");
                                        //godot_print!(" => {:?}\n", frame);
                                        //godot_print!("= {:?}\n", frame);
                                        udp.publish_message(imu_frame.clone());
                                        let frame= messages::convert_to_frame(vec![Box::new(measurements)]);
                                        udp.publish_message(frame.clone());
                                    },
                                    None => {

                                    },
                                }
                            }*/
                            
                        }
                        //udp.exec_main_task();
                    }
                },
                Err(_) => {

                },
            }
            
        }
        
    }

    fn exit_tree(&mut self) {

    }
}