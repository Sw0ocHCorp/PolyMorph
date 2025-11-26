use std::{collections::VecDeque, net::UdpSocket, sync::{Arc, Mutex, mpsc::{Receiver, Sender, channel}}, thread::{self, JoinHandle, Thread}, time::{Duration, Instant}};

use godot::{classes::{Image, class_macros::private::virtuals::ImageTexture, editor_vcs_interface::ChangeType}, prelude::*};
use godot::prelude::INode3D;

use crate::{com_channels::{self, Channel, Message, UDPChannel}, events_management::{self, Event, Observer}, process::{self, ProcessConfig, Worker}};

#[derive(Clone)]
pub struct ImageConfig {
    width: u32,
    height: u32,
    data: Vec<u8>,
}

#[derive(GodotClass)]
#[class(init, base=Node3D)]
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
        let mut udp= UDPChannel::new(
                                            com_channels::ChannelConfig::new("127.0.0.1".to_string()), 8080, 
                                            "127.0.0.1".to_string(), 8090);
        udp.
        /*let mut udp= UDPChannel::new(com_channels::ChannelConfig {
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
        }*/
        godot_print!("Hello\n");
        //return FlightController {base};
    }
    

    fn process(&mut self, delta: f64) {
        if let Some(parent)= self.base().get_parent() {
            for child_index in 0..parent.get_child_count() {
                if let Some(node)= parent.get_child(child_index){ 
                    if node.clone().is_class("Viewport") && node.get_name().to_string() == "MonoCamVP" {
                        let vp= node.clone().cast::<godot::classes::Viewport>();
                        if let Some(texture)= vp.get_texture() && let Some(image)= texture.get_image() {
                            let height= image.get_height();
                            let width= image.get_width();
                            //frame.resize(height, width)
                            let mut pixels: Vec<u8> = Vec::with_capacity((width * height * 4) as usize);
                            for y in 0..height {
                                for x in 0..width {
                                    let c = image.get_pixel(x, y);
                                    pixels.push((c.r * 255.0) as u8);
                                    pixels.push((c.g * 255.0) as u8);
                                    pixels.push((c.b * 255.0) as u8);
                                    pixels.push((c.a * 255.0) as u8);
                                }
                            }
                            let img= ImageConfig {
                                width: width as u32,
                                height: height as u32,
                                data: pixels.clone(),
                            };
                            if let Some(udp) = self.udp.clone() {
                                udp.clone().add_message_in_queue(Message::Image(img));
                            }
                        }
                    }
                }
            }
            if let Some(udp) = self.udp.clone() {
                udp.clone().start_task();
            }
        }
    }

    fn exit_tree(&mut self) {
        if let Some(udp) = self.udp.clone() {
            udp.clone().end();
            //udp.
        }
    }
}