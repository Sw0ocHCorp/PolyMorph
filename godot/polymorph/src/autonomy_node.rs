use std::sync::Arc;

use godot::{classes::{InputEvent, InputEventJoypadButton, InputEventJoypadMotion, PhysicsRayQueryParameters3D, RigidBody3D, StaticBody3D}, global::{JoyAxis, JoyButton, cos, sin}, prelude::*};
use num_quaternion::UQ64;
use robomorph::{actuators::mixer_model::MixerModel, communication::UDPChannel, core::{messages::{self}, utils, worker::Worker}, lidar_management::measurements::LidarMeasurements, positionning::pose::{GPSData, IMUData, Pose}};

use crate::control_command::osprey_bicopter_mixer::OspreyBicopterMixer;

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
    last_linear_vel: Vector3,
    world_magnetic_field: Vector3,
    data_fps: u32,
    gps_frequency: u32,
    dt: f64,
    gps_dt: f64,
    wing_speed: f64,
    max_thrust: f64,
    waypoint: [f64; 3],
    wingl_vel: [f32;3],
    wingr_vel: [f32;3],
    mixer: OspreyBicopterMixer
}
#[godot_api]
/**
 * /!\ WARNING: Execute tasks sequentially. Not in a dedicated thread
 * /!\ Unable to maintain a dedicated thread for  GodotClass  
 */
impl INode3D for AutonomyNode{

    fn ready(&mut self) {
        //Get and store the metadata of the 
        if self.base().has_meta("lidarPoints") && let Ok(lidar_points) = self.base().get_meta("lidarPoints").try_to::<i32>() && lidar_points > 0 {
            self.lidar_points= lidar_points as u32;
            godot_print!("Loading lidarPoints metadata succesfully, It's value is= {}", lidar_points);
        } else {
            godot_print!("/!\\ ERROR: unknown lidarPoints metadata");
            self.lidar_points= 100;
        }

        if self.base().has_meta("lidarFov") && let Ok(lidar_fov) = self.base().get_meta("lidarFov").try_to::<i32>() {
            self.lidar_fov= (lidar_fov as f64).to_radians();
            godot_print!("Loading lidarFov metadata succesfully, It's value is= {}°", lidar_fov);
        } 
        else {
            godot_print!("/!\\ ERROR: unknown lidarFov metadata");
            self.lidar_fov= (180 as f64).to_radians();
        }

        if self.base().has_meta("lidarAngleOffset") && let Ok(lidar_angle_offset) = self.base().get_meta("lidarAngleOffset").try_to::<i32>() {
            self.lidar_angle_offset= (lidar_angle_offset as f64).to_radians();
            godot_print!("Loading lidarAngleOffset metadata succesfully, It's value is= {}°", lidar_angle_offset);
        } else {
            godot_print!("/!\\ ERROR: unknown lidarAngleOffset metadata");
            self.lidar_angle_offset= 0 as f64;
        }

        if self.base().has_meta("dataFrequency") && let Ok(data_frequency) = self.base().get_meta("dataFrequency").try_to::<u32>() {
            self.data_fps= data_frequency;
            godot_print!("Loading dataFrequency metadata succesfully, It's value is= {}Hz", data_frequency);
        }
        else {
            godot_print!("/!\\ ERROR: unknown dataFrequency metadata");
            self.data_fps= 10;
        } 

        if self.base().has_meta("gpsFrequency") && let Ok(data_frequency) = self.base().get_meta("gpsFrequency").try_to::<u32>() {
            self.gps_frequency= data_frequency;
            godot_print!("Loading gpsFrequency metadata succesfully, It's value is= {}Hz", data_frequency);
        }
        else {
            godot_print!("/!\\ ERROR: unknown gpsFrequency metadata");
            self.gps_frequency= 1;
        }

        let mut wing_pid_factors= [1.0, 0.0, 0.0];
        let mut max_wing_angle_error= 90.0;
        let mut wing_angle_threshold= 1.0;
        let mut thruster_pid_factors= [1.0, 0.0, 0.0];
        let mut max_thrust= 1.15;
        let mut dist_threshold= 0.5;
        if self.base().has_meta("wingPIDFactors") && let Ok(factors) = self.base().get_meta("wingPIDFactors").try_to::<Vector3>() {
            wing_pid_factors= [factors.x as f64, factors.y as f64, factors.z as f64];
            godot_print!("Loading wingPIDFactors metadata succesfully, It's value is= P= {} | I= {} | D= {}", wing_pid_factors[0], wing_pid_factors[1], wing_pid_factors[2]);
        }
        else {
            godot_print!("/!\\ ERROR: unknown wingPIDFactors metadata");
            self.wing_speed= 90.0;
        }

        if self.base().has_meta("wingMaxRotationSpeed") && let Ok(wing_rotation_speed) = self.base().get_meta("wingMaxRotationSpeed").try_to::<f32>() {
            self.wing_speed= wing_rotation_speed as f64;
            godot_print!("Loading wingMaxRotationSpeed metadata succesfully, It's value is= {}°/s", wing_rotation_speed);
        }
        else {
            godot_print!("/!\\ ERROR: unknown wingMaxRotationSpeed metadata");
            self.wing_speed= 90.0;
        }

        if self.base().has_meta("maxWingAngleError") && let Ok(wing_angle_error) = self.base().get_meta("maxWingAngleError").try_to::<f32>() {
            max_wing_angle_error= wing_angle_error as f64;
            godot_print!("Loading maxWingAngleError metadata succesfully, It's value is= {}°", wing_angle_error);
        }
        else {
            godot_print!("/!\\ ERROR: unknown maxWingAngleError metadata");
        }

        if self.base().has_meta("wingAngleThreshold") && let Ok(angle_threshold) = self.base().get_meta("wingAngleThreshold").try_to::<f32>() {
            wing_angle_threshold= angle_threshold as f64;
            godot_print!("Loading wingAngleThreshold metadata succesfully, It's value is= {}°", angle_threshold);
        }
        else {
            godot_print!("/!\\ ERROR: unknown wingAngleThreshold metadata");
        }

        if self.base().has_meta("thrusterPIDFactors") && let Ok(factors) = self.base().get_meta("thrusterPIDFactors").try_to::<Vector3>() {
            thruster_pid_factors= [factors.x as f64, factors.y as f64, factors.z as f64];
            godot_print!("Loading thrusterPIDFactors metadata succesfully, It's value is= P= {} | I= {} | D= {}", thruster_pid_factors[0], thruster_pid_factors[1], thruster_pid_factors[2]);
        }
        else {
            godot_print!("/!\\ ERROR: unknown thrusterPIDFactors metadata");
            self.wing_speed= 90.0;
        }

        if self.base().has_meta("maxThrust") && let Ok(max_thrust) = self.base().get_meta("maxThrust").try_to::<f32>() {
            self.max_thrust= max_thrust as f64;
            godot_print!("Loading maxThrust metadata succesfully, It's value is= {}m/s", self.max_thrust);
        }
        else {
            godot_print!("/!\\ ERROR: unknown maxThrust metadata");
        }

        if self.base().has_meta("distThreshold") && let Ok(dist_thresh) = self.base().get_meta("distThreshold").try_to::<f32>() {
            dist_threshold= dist_thresh as f64;
            godot_print!("Loading distThreshold metadata succesfully, It's value is= {}m", dist_thresh);
        }
        else {
            godot_print!("/!\\ ERROR: unknown distThreshold metadata");
        }

        if self.base().has_meta("wayPoint") && let Ok(waypoint) = self.base().get_meta("wayPoint").try_to::<Vector3>() {
            self.waypoint= [waypoint.x as f64, waypoint.y as f64, waypoint.z as f64];
            godot_print!("Loading wayPoint metadata succesfully, It's value is= {:?}", self.waypoint);
        }
        else {
            godot_print!("/!\\ ERROR: unknown wayPoint metadata");
            self.waypoint= [0.0, 0.0, 0.0];
        }

        self.world_magnetic_field= Vector3 { x: REF_WORLD_MAGNETIC_FIELD[0] as f32, y: REF_WORLD_MAGNETIC_FIELD[2] as f32, z: REF_WORLD_MAGNETIC_FIELD[1] as f32 };
        self.world_magnetic_field= self.world_magnetic_field.normalized();
        let udp= UDPChannel::new_async("127.0.0.1", 8080, "127.0.0.1", 8090);
        let udp_debug= UDPChannel::new_async("127.0.0.1", 9010, "127.0.0.1", 9000);
        let init_pose= Pose::new(GPSData{latitude: 0.0, longitude: 0.0}, [0.0, 0.0, 0.0], 
                                                        UQ64::one(), 
                                                        [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        self.mixer = OspreyBicopterMixer::new(init_pose, wing_pid_factors, max_wing_angle_error, wing_angle_threshold, thruster_pid_factors, max_thrust, dist_threshold);
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
        //IF this node had a parent
        if let Some(parent_obj) = self.base().get_parent() {
            //Generate lidar_points times raycast measurement for lidar_fov°
            match parent_obj.clone().try_cast::<RigidBody3D>() {
                Ok(robot) => {
                    if let Some(wingl) = robot.find_child("WingL") && let Some(wingr) = robot.find_child("WingR"){
                        if let Ok(mut wingl_body)= wingl.try_cast::<RigidBody3D>() && let Ok(mut wingr_body)= wingr.try_cast::<RigidBody3D>() {
                            let wingl_orientation= wingl_body.get_rotation_degrees();
                            let wingr_orientation= wingr_body.get_rotation_degrees();
                            let wing_lin_speeds= vec![wingl_body.get_linear_velocity().length(),  wingr_body.get_linear_velocity().length()];
                            let robot_loc= robot.get_global_position();
                            let mut commands= self.mixer.apply_command_law(vec![wingl_orientation.z as f64, wingr_orientation.z as f64, robot_loc.x as f64, robot_loc.y as f64, robot_loc.z as f64] , self.waypoint.to_vec(), delta);
                            for i in 0..commands.len() {
                                //godot_print!("Wing Orientation= {}    ->    Raw Command= {}", wingr_orientation.z, commands[i]);
                                //godot_print!("Lower Bound= {} Higher Bound= {}", -self.wing_speed*delta, self.wing_speed*delta);
                                if i < 2 {
                                    commands[i]= f64::clamp(commands[i], -self.wing_speed*delta, self.wing_speed*delta);
                                } else {
                                    //Compute the acceleration required to reach the given speed (clamp it to ensure staying under / at max accel / deccel)
                                    let delta_v= f64::clamp((commands[i] - wing_lin_speeds[i-2] as f64)*delta, -self.max_thrust*delta, self.max_thrust*delta);
                                    //godot_print!("Delta V to reach waypoint= {}", delta_v);
                                    //Update the new speed to apply to the wing
                                    commands[i] = delta_v;
                                    godot_print!("Next delta V= {}", commands[i]);
                                }
                                
                                
                            }
                            wingl_body.set_rotation_degrees(Vector3::new(wingl_orientation.x, wingl_orientation.y, utils::modulo_180( wingl_orientation.z + commands[0] as f32)));
                            wingr_body.set_rotation_degrees(Vector3::new(wingr_orientation.x, wingr_orientation.y, utils::modulo_180( wingr_orientation.z + commands[1] as f32)));
                            let wingl_orientation= wingl_body.get_rotation();
                            let wingr_orientation= wingr_body.get_rotation();
                            self.wingl_vel[0]= f32::clamp(self.wingl_vel[0] + f32::sin(wingl_orientation.z)*commands[2] as f32, 0.0, self.max_thrust as f32);
                            self.wingl_vel[1]= f32::clamp(self.wingl_vel[1] + f32::cos(wingl_orientation.z)*commands[2] as f32, 0.0, self.max_thrust as f32);
                            self.wingl_vel[2]= 0.0;
                            self.wingr_vel[0]= f32::clamp(self.wingr_vel[0] + f32::sin(wingr_orientation.z)*commands[3] as f32, 0.0, self.max_thrust as f32);
                            self.wingr_vel[1]= f32::clamp(self.wingr_vel[1] + f32::cos(wingr_orientation.z)*commands[3] as f32, 0.0, self.max_thrust as f32);
                            self.wingr_vel[2]= 0.0;
                            godot_print!("Theoric Next Left Wing velocity= {:?}", self.wingl_vel);
                            godot_print!("Theoric Next Right Wing velocity= {:?}", self.wingr_vel);
                            wingl_body.set_linear_velocity(Vector3 { x: self.wingl_vel[0], y: self.wingl_vel[1], z: self.wingl_vel[2] });
                            wingr_body.set_linear_velocity(Vector3 { x: self.wingr_vel[0], y: self.wingr_vel[1], z: self.wingr_vel[2] });
                            let wing_lin_speeds= vec![wingl_body.get_linear_velocity().length(),  wingr_body.get_linear_velocity().length()];
                            godot_print!("Real New Wings velocities= {:?}", wing_lin_speeds);
                        }
                    }
                    //if let Ok(wing_l) = robot.get_chil
                    let rob_orientation= robot.get_global_rotation();
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
                    //godot_print!("Linear Accel:\n{:?}", godot_linear_accel);
                    //godot_print!("IMU Data:\n{:?}", imu_data);
                    //Compute the velocities to apply to the robot from the joystick input*
                    
                    //Applying the velocity computed from the joystick input
                    //robot.set_linear_velocity(translation);
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
                                    let raycast = PhysicsRayQueryParameters3D::create(origin, origin + Vector3{x: 50.0*cos(utils::modulo_pi_f64(-(angle + angle_offset))) as f32, y: 0.5, z: 50.0*sin(utils::modulo_pi_f64(-(angle+ angle_offset))) as f32});
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
                                    godot_print!("Send Late\n    Expected Frequency= {} | Elapsed Time= {} |TIME BETWEEN FRAMES= {}", udp_worker.get_frequency(), self.dt, delta);
                                }
                                self.dt= 0.0;
                                match udp_worker.get_module().downcast_ref::<UDPChannel>() {
                                    Some(udp) => {
                                        let gps_data= utils::local_to_global_frame(ORIGIN_GPS_DATA, robot.get_global_position().x as f64, robot.get_global_position().y as f64);
                                        if self.gps_dt >= 1.0 / self.gps_frequency as f64 {
                                            let gps_frame= messages::convert_to_frame(vec![Box::new(gps_data)]);
                                            udp.publish_message(gps_frame);
                                            self.gps_dt= 0.0;
                                        }
                                        let imu_frame=messages::convert_to_frame(vec![Box::new(imu_data)]);
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