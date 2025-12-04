use std::{collections::HashMap, sync::{Arc, Mutex}};

use godot::{classes::{PhysicsRayQueryParameters3D, editor_vcs_interface::ChangeType}, global::{cos, sin}, prelude::*};
use ordered_float::OrderedFloat;
use robomorph::{com_channels::{Channel, ChannelConfig, ChannelType, UDPChannel}, events_management::Observer, messages::Message, process::ModuleLinker, utils::normalize_angle};


#[derive(GodotClass)]
#[class(init, base=Node3D)]
pub struct AutonomyNode {
    #[base]
    base: Base<Node3D>,
    udp: Option<Arc<UDPChannel>>,
    lidar_points: f64,
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
        self.udp=Some(Arc::new(UDPChannel::new(ChannelConfig::new("127.0.0.1".to_string(),
                                                                                ModuleLinker::new("UDP2_WORKER".to_string())), 
                                                                                8080, "127.0.0.1".to_string(), 8090, 1)));
                                                                                
        if let Some(udp_clone)= self.udp.clone() {
            loop {
                if let Some(socket) = udp_clone.clone().get_socket() {
                    break;
                } 
                else {
                    if let Some(socket) = udp_clone.clone()._connect() &&  let ChannelType::UDP(s)= socket{
                        udp_clone.clone().set_socket(s);
                    }
                }        
            }
        }
        //Get and store the metadata of the 
        if self.base().has_meta("lidarPoints") {
            match self.base().get_meta("lidarPoints").try_to::<i32>() {
                Ok(lidar_points) => {
                    self.lidar_points= lidar_points as f64;
                    godot_print!("Loading lidarPoints metadata succesful, It's value is= {}", lidar_points);
                },
                Err(_) => {
                    godot_print!("/!\\ ERROR: lidarPoints conversion in i32 failed.\nYou should change the metadata type in int");
                    self.lidar_points= 100_f64;
                },
            } 
        } else {
            godot_print!("/!\\ ERROR: No lidarPoints metadata exist.\nYou should add a metadata called \"lidarPoints\" with int as type");
            self.lidar_points= 100_f64;
        }
        if self.base().has_meta("lidarFov") {
            match self.base().get_meta("lidarFov").try_to::<i32>() {
                Ok(lidar_fov) => {
                    self.lidar_fov= lidar_fov as f64;
                    godot_print!("Loading lidarFov metadata succesful, It's value is= {}°", lidar_fov);
                },
                Err(_) => {
                    godot_print!("/!\\ ERROR: lidarFov conversion in i32 failed.\nYou should change the metadata type in int");
                    self.lidar_fov= 180 as f64;
                },
            } 
        } else {
            godot_script_error!("/!\\ ERROR: No lidarFov metadata exist.\nYou should add a metadata called \"lidarFov\" with int as type");
            self.lidar_fov= 180 as f64;
        }
        if self.base().has_meta("lidarRelativeMidAngle") {
            match self.base().get_meta("lidarRelativeMidAngle").try_to::<i32>() {
                Ok(lidar_angle_offset) => {
                    self.lidar_angle_offset= lidar_angle_offset as f64;
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
    }
    

    fn process(&mut self, delta: f64) {
        //Detect the collision point between the raycast and the rigidBodies in the scene
        let mut measurements:HashMap<OrderedFloat<f64>, f64>= HashMap::new();
        //IF this node had a parent
        if let Some(mut parent_obj) = self.base().get_parent() {
            //Generate lidar_points times raycast measurement for lidar_fov°

            for i in 0..self.lidar_points as i32{
                let mut angle = (-self.lidar_fov / 2.0).to_radians() + ((i as f64 / (self.lidar_points-1.0)) * self.lidar_fov).to_radians() + self.lidar_angle_offset.to_radians();
                //IF the scene exist
                if let Some(mut world)= self.base().get_world_3d() {
                    //IF the space_state (to test raycast) exist
                    if let Some(mut space_state)= world.get_direct_space_state(){
                        //If the parent is a Node3D
                        if let Ok(parent)= parent_obj.clone().try_cast::<Node3D>() {
                            //Get the position of the parent
                            let origin= parent.get_global_position();
                            // generate a raycast of 50m with the specific angle
                            let mut raycast = PhysicsRayQueryParameters3D::create(origin, origin + Vector3{x: origin.x + 50.0*cos(angle) as f32, y: 0.5, z: origin.z + 50.0*sin(angle) as f32});
                            //Get the collision of the raycast and Rigidbodies of in the scene
                            let collision= space_state.intersect_ray(raycast.as_ref());
                            //MATCH: there is a collider position? (means there is a collision with a RigidBody?)
                            match collision.get("position") {
                                //There is a collision with a RigidBody
                                Some(pos_variant) => {
                                    match pos_variant.try_to::<Vector3>() {
                                        Ok(pos) => {
                                            //Add the distance with the rigidbody in the list, keyed by angle
                                            let distance = origin.distance_to(pos) as f64;
                                            measurements.insert(OrderedFloat(angle), distance);
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
        }
        //IF the UDP module exist
        if let Some(udp) = &self.udp {
            //IF there is measurements
            if let Some(socket)= udp.clone().get_socket() {
                if measurements.len() > 0 {
                    //Send thoses measurements
                    if let Err(_)= udp.clone().send_message(robomorph::com_channels::ChannelType::UDP(socket), Message::LidarMeasurements(measurements)) {
                        println!("ERROR: Failed to send UDP frame");
                    }
                    //udp.send_message(ChannelType::UDP(socket), Message::LidarMeasurements(measurements))
                    //linker.send_message(Message::LidarMeasurements(measurements));
                }
            }
        } else {
            let a= 1;
        }
    }

    fn exit_tree(&mut self) {

    }
}