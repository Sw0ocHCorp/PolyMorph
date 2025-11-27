use std::sync::Arc;

use godot::{classes::{PhysicsRayQueryParameters3D, RayCast3D, RigidBody3D, class_macros::private::virtuals::PhysicsDirectBodyState2DExtension::get_transform}, global::{cos, sin}, prelude::*};
use robomorph::{com_channels::{ChannelConfig, UDPChannel}, events_management::Observer, messages::Message, process::ModuleLinker};


#[derive(GodotClass)]
#[class(init, base=Node3D)]
pub struct AutonomyNode {
    #[base]
    base: Base<Node3D>,
    udp: Option<Arc<UDPChannel>>,
}
#[godot_api]
/**
 * /!\ WARNING: Execute tasks sequentially. Not in a dedicated thread
 * /!\ Unable to maintain a dedicated thread for  GodotClass  
 */
impl INode3D for AutonomyNode{

    fn ready(&mut self) {
        //Create the UDP Channels / Modules
        self.udp=Some(Arc::new(UDPChannel::new(ChannelConfig::new("127.0.0.1".to_string(),
                                                                                ModuleLinker::new("UDP1_WORKER".to_string())), 
                                                                                8090, "127.0.0.1".to_string(), 8080)));
        if let Some(udp)= &self.udp{
            let udp_cl= udp.clone();
            let obs= Observer::new(Arc::new(Box::new(move |x| {
                if let Message::LidarMeasurements(msg) = x {
                    let size= msg.len();
                    godot_print!("{}= Incoming data {:?} from {}:{}", "UDP1_WORKER".to_string(), msg, udp_cl.clone().get_target_address(), udp_cl.clone().get_target_port());
                    godot_print!("");
                }

            })));
            if let Ok(mut linker)= udp.clone().chan_config.linker.try_lock() {
                linker.attach_data_observer(obs);
            }
        }
    }
    

    fn process(&mut self, delta: f64) {
        //Detect the collision point between the raycast and the rigidBodies in the scene
        let mut measurements:Vec<f32>= Vec::new();
        let mut ray_count= 0;
        //IF this node had a parent
        if let Some(mut parent_obj) = self.base().get_parent() {
            //Generate 200 raycast measurement for 90°
            for i in 0..200 {
                //Compute angles for 180° scan
                let mut angles= vec![(90.0 + (i as f64/200.0)*90.0 as f64).to_radians(), -(90.0 + (i as f64/200.0)*90.0 as f64).to_radians()];
                //IF the scene exist
                if let Some(mut world)= self.base().get_world_3d() {
                    //IF the space_state (to test raycast) exist
                    if let Some(mut space_state)= world.get_direct_space_state(){
                        for angle in angles {
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
                                                //Add the distance with the rigidbody in the list
                                                measurements.push(origin.distance_to(pos));
                                            },
                                            Err(_) => {
                                                godot_print!("Error: Collider position is not a Vector3D");
                                            },
                                        }
                                    },
                                    //No RigidBody detected
                                    None => 
                                        //Return non-sense value to indicate no collision with RigidBody
                                        measurements.push(-1.0),
                                }
                            }
                        }
                    }
                }    
            }
        }
        //IF the UDP module exist
        if let Some(udp) = &self.udp {
            //IF the ModuleLinker is available
            if let Ok(mut linker)= udp.clone().chan_config.linker.try_lock() {
                //IF there is measurments
                if measurements.len() > 0 {
                    //Send thoses measurements
                    linker.send_message(Message::LidarMeasurements(measurements));
                }
            }
        }
    }

    fn exit_tree(&mut self) {

    }
}