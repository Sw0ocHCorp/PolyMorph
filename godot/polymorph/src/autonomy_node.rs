use std::{collections::HashMap, sync::{Arc, Mutex}};

use godot::{classes::{PhysicsRayQueryParameters3D, editor_vcs_interface::ChangeType}, global::{cos, sin}, prelude::*};
use ordered_float::OrderedFloat;
use robomorph::{communication::UDPChannel, lidar::LidarMeasurements, messages::{self, Translatable}, utils, worker::{Module, Worker}};


#[derive(GodotClass)]
#[class(init, base=Node3D)]
pub struct AutonomyNode {
    #[base]
    base: Base<Node3D>,
    udp_worker: Option<Arc<Worker>>,
    lidar_points: u32,
    lidar_fov: f64,
    lidar_angle_offset: f64,
}
#[godot_api]
/**
 * /!\ WARNING: Execute tasks sequentially. Not in a dedicated thread
 * /!\ Unable to maintain a dedicated thread for  GodotClass  
 */
impl INode3D for AutonomyNode{

    fn ready(&mut self) {
        //let test= vec![]
        let udp= UDPChannel::new_async("127.0.0.1", 8080, "127.0.0.1", 8090);
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
        if self.base().has_meta("lidarRelativeMidAngle") {
            match self.base().get_meta("lidarRelativeMidAngle").try_to::<i32>() {
                Ok(lidar_angle_offset) => {
                    self.lidar_angle_offset= (lidar_angle_offset as f64).to_radians();
                    godot_print!("Loading lidarRelativeMidAngle metadata succesful, It's value is= {}°", lidar_angle_offset);
                },
                Err(_) => {
                    godot_print!("/!\\ ERROR: lidarRelativeMidAngle conversion in i32 failed.\nYou should change the metadata type in int");
                    self.lidar_angle_offset= 0 as f64;
                },
            } 
        } else {
            godot_script_error!("/!\\ ERROR: No lidarRelativeMidAngle metadata exist.\nYou should add a metadata called \"lidarFov\" with int as type");
            self.lidar_angle_offset= 0 as f64;
        }
        self.udp_worker= Some(Worker::new(udp, "UDP_WORKER", 50, false))
    }
    

    fn process(&mut self, delta: f64) {
        //godot_print!("{}\n", delta);
        //Detect the collision point between the raycast and the rigidBodies in the scene
        let mut measurements= LidarMeasurements::new(true);
        //IF this node had a parent
        if let Some(mut parent_obj) = self.base().get_parent() {
            //Generate lidar_points times raycast measurement for lidar_fov°

            for i in 0..self.lidar_points{
                //let mut base_angle= (-self.lidar_fov / 2.0) + ((i as f64 / (self.lidar_points-1.0)) * self.lidar_fov);
                let angle = utils::modulo_pi_f64((-self.lidar_fov / 2.0) + ((i as f64 / (self.lidar_points-1) as f64) * self.lidar_fov) + self.lidar_angle_offset);
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
                            let mut raycast = PhysicsRayQueryParameters3D::create(origin, origin + Vector3{x: 50.0*cos(utils::modulo_pi_f64(-angle)) as f32, y: 0.5, z: 50.0*sin(utils::modulo_pi_f64(-angle)) as f32});
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
                                            let tamere= measurements.len();
                                            measurements.insert(angle.to_degrees() as f32, distance);
                                            let lapute= measurements.len();
                                            if tamere == lapute {
                                                godot_print!("Error: Measurement at angle {}rad {}° not inserted", angle, angle.to_degrees());
                                            }
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
                                    //Return non-sense value to indicate no collision with RigidBody
                                    //measurements.push(-1.0),
                            }
                        }
                    }
                }    
            }
            /*if measurements.len() != 1000 {
                godot_print!("Warning:");
                godot_print!("=======================\n");
            }*/
            //godot_print!("=======================\n");
        }
        //IF the UDP module exist
        if let Some(udp_worker)= &self.udp_worker {
            if measurements.len() > 0 {
                if udp_worker.try_run() {
                    match udp_worker.get_module().downcast_ref::<UDPChannel>() {
                        Some(udp) => {
                            /*godot_print!("=======================\n");
                            godot_print!("LIDAR MEASUREMENTS SENT: {:?}\n", measurements);
                            godot_print!("=======================\n");*/
                            
                            let frame= messages::convert_to_frame(vec![Box::new(measurements)]);
                            //godot_print!("Frame generated {:?}\n", frame);
                            //godot_print!("Frame bytes: {:?}\n", frame);
                            udp.publish_message(frame.clone());//measurements.to_bytes());
                            godot_print!("SEND DATA\n");
                            /*let translatables= messages::parse_frame(frame.clone());
                            let mut i= 0;
                            for translatable in translatables {
                                godot_print!("Translatable {}\n", i);
                                i+= 1;
                                match translatable.downcast_ref::<LidarMeasurements>() {
                                    Some(lidar_meas) => {
                                        godot_print!("LIDAR MEASUREMENTS: {:?}\n", lidar_meas.order_by_angle())
                                    },
                                    None => {

                                    },
                                }
                                //println!("MailBox Loopback received: {:?}", translatable);
                            }*/
                        },
                        None => {

                        },
                    }
                } 
            }
            //udp.exec_main_task();
        }
    }

    fn exit_tree(&mut self) {

    }
}