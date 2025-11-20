use std::{collections::VecDeque, sync::{Arc, Mutex}};

use godot::prelude::*;
use godot::prelude::INode3D;
use crate::{com_channels::{self, Message, UDPChannel}, events_management::{self, Observer}, process::{self, ProcessConfig, Worker}};

#[derive(GodotClass)]
#[class(init, base=Node3D)]
pub struct FlightController {
    #[base]
    base: Base<Node3D>,
    udp_worker: Option<Arc<UDPChannel>>
}
#[godot_api]
impl INode3D for FlightController{
    
    fn ready(&mut self) {
        
        let udp_data_observer: events_management::Observer<Message>= events_management::Observer::new(Arc::new(Box::new(|x| {
            print!("Receive Message to Send through UDP");
        })));

        let incoming_data_observer: Observer<Message>= events_management::Observer::new(Arc::new(Box::new(|x| {
            if let Message::Frame(msg) = x {
                if let Ok(data)= String::from_utf8(msg) {
                    println!("Incoming data {}", data);
                }
            }
        })));
        //UDP event:
        //Were the received data is sent
        let udp_data_event: events_management::Event<Message>= events_management::Event::new(vec![incoming_data_observer]);
        let mut udp_worker= Some(Arc::new(com_channels::UDPChannel::new( 
                com_channels::ChannelConfig {
                                address: "127.0.0.1".to_string(),
                                worker_config: process::WorkerConfig { 
                                    process_config: ProcessConfig {
                                        data_event: udp_data_event, 
                                        data_observer: udp_data_observer,
                                    }, 
                                    worker_thread: Mutex::new(None) ,
                                    is_running: Mutex::new(false),
                                }, 
                                message_buffer: Mutex::new(VecDeque::new())
                            }, 
                8080, "127.0.0.1".to_string(), 8090)));
        if let Some(udp)= udp_worker.clone() {
            udp.clone().start();
        } 
        godot_print!("Hello\n");
        //return FlightController {base};
    }
    

    fn physics_process(&mut self, delta: f64) {
        let a= 1;
        //godot_print!("Hello from process ");
        //self.base_mut().rotate(Vector3 { x: 0.0, y: 1.0, z: 0.0 }, 0.15);
    }
}