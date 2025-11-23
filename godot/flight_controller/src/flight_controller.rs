use std::{collections::VecDeque, net::UdpSocket, sync::{Arc, Mutex, mpsc::{Receiver, Sender, channel}}, thread::{self, JoinHandle, Thread}, time::{Duration, Instant}};

use godot::prelude::*;
use godot::prelude::INode3D;
use crate::{com_channels::{self, Message, UDPChannel}, events_management::{self, Event, Observer}, process::{self, ProcessConfig, Worker}};

#[derive(GodotClass)]
#[class(init, base=Node3D)]
/*pub struct FlightController {
    #[base]
    base: Base<Node3D>,
    tx: Arc<Mutex<Option<Sender<String>>>>,
    rx: Arc<Mutex<Option<Receiver<String>>>>,
}*/
pub struct FlightController {
    #[base]
    base: Base<Node3D>,
    udp: Option<Arc<UDPChannel>>,
}
#[godot_api]
// /!\ WARNING: Using UDP WORKER sequentially. Not in a dedicated thread
// /!\ Unable to maintain a dedicated thread for  GodotClass  
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
        let mut udp= UDPChannel::new(com_channels::ChannelConfig {
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
                8080, "127.0.0.1".to_string(), 8090);
        //udp.clone().start_routine();
        if self.udp.is_none() {
            self.udp= Some(udp);
        }
        godot_print!("Hello\n");
        //return FlightController {base};
    }
    

    fn process(&mut self, delta: f64) {
        if let Some(udp) = self.udp.clone() {
            udp.clone().start_task();
        }
        thread::sleep(Duration::from_millis(20));
    }

    fn exit_tree(&mut self) {
        if let Some(udp) = self.udp.clone() {
            udp.clone().end();
            //udp.
        }
    }
}