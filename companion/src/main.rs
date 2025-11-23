pub mod com_channels;
pub mod events_management;
pub mod process;

use std::{collections::VecDeque, sync::{Arc, Mutex}, thread, time::Duration};

use crate::{com_channels::Message, events_management::Observer, process::{ProcessConfig, Worker}};


fn main() {
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
                8090, "127.0.0.1".to_string(), 8080)));
        if let Some(udp)= udp_worker.clone() {
            udp.clone().start_routine();
        } 

        loop {
        }
}
