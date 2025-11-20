use godot::global::godot_print;

use crate::{process::{Worker, WorkerConfig}};
use std::{collections::VecDeque, io::{self, ErrorKind}, sync::{Arc, Mutex}, thread::{self}};
use std::net::UdpSocket;

#[derive(Clone)]
pub enum Message {
    Sentence(String),
    Frame(Vec<u8>),
}

pub enum ChannelType {
    UDP(UdpSocket)
}

pub struct ChannelConfig {
    pub address: String,
    pub worker_config: WorkerConfig,
    pub message_buffer: Mutex<VecDeque<Message>>,
}

pub trait Channel : Worker{
    fn _connect(self: Arc<Self>) -> Option<ChannelType>;
    fn send_message(self: Arc<Self>, port: ChannelType, msg: Message) -> Result<bool, io::Error>;
    fn _listen_port(self: Arc<Self>, port: ChannelType);
}

pub struct UDPChannel {
    base_config: ChannelConfig,
    socket: Mutex<Option<UdpSocket>>,
    port: u32,
    target_address: String,
    target_port: u32
} 


impl UDPChannel {
    pub fn new(base_config: ChannelConfig, port: u32, target_address: String, target_port: u32) -> UDPChannel {
        return UDPChannel {base_config, port, target_address, target_port, socket: Mutex::new(None)};
    } 
}

impl Worker for UDPChannel {
    fn start(self: Arc<Self>) {
        let mut is_connected = false;
        let this: Arc<UDPChannel>= self.clone();
        if let Ok(mut buffer) =self.clone().base_config.message_buffer.try_lock() {
            buffer.push_back(Message::Frame("CAVA".as_bytes().to_vec()));
        }
        //Set the thread in running mode
        if let Ok(mut is_running) = self.clone().base_config.worker_config.is_running.try_lock() {
            *is_running= true;
        }
        //Thread used to maintain the UDP connection
        let running_thread= thread::Builder::new()
            .name("UDP_WORKER".to_string())
            .spawn(move || {
                loop{
                    //Stop the thread if the thread is not running 
                    if let Ok(mut is_running) = this.clone().base_config.worker_config.is_running.try_lock() {
                        if !*is_running {
                            godot_print!("Soft Stop channel thread");
                            println!("Soft Stop channel thread");
                            break;
                        }
                    } else {
                        godot_print!("Is_Running is used");
                        println!("Is_Running is used");
                    }
                    //IF the socket variable is available
                    if let Ok(mut socket_guard) = this.clone().socket.try_lock() {
                        //IF no socket
                        // /!\ socket_guard.is_none() is prefered than socket_guard.take() 
                        //because socket_guard.take() force to None after the enclosure
                        // socket_guard.is_none() use socket_guard as ref
                        if socket_guard.is_none() {
                            if let Some(ChannelType::UDP(sock)) =  this.clone()._connect(){
                                *socket_guard= Some(sock);
                            }
                        }
                        //ELSE the socket is created -> Send the data in the buffer and listen the incoming frames
                        else {
                            //IF the message_buff variable is available
                            if let Ok(mut message_buff) = this.clone().base_config.message_buffer.try_lock() {
                                //IF there is messages to send through UDP
                                if message_buff.len() > 0 {
                                    if let Some(msg_to_send)= message_buff.pop_front(){
                                        if let Some(ref sock) = *socket_guard {
                                            this.clone().send_message(ChannelType::UDP(sock.try_clone().expect("Failed to clone UdpSocket")), msg_to_send);
                                        }
                                    }
                                    //Sen the oldest message
                                    /*if let Some(msg_to_send)= message_buff.pop_front(){
                                        if let Some(ref sock) = *socket_guard {
                                            this.clone().send_message(ChannelType::UDP(sock.try_clone().expect("Failed to clone UdpSocket")), msg_to_send);
                                        }
                                    }*/
                                }
                                //Listen the incoming frame
                                if let Some(ref socket) = *socket_guard {
                                    //Get the socket instance by cloning the ref of the data protected by the mutex
                                    //And call the listening with this UDP socket 
                                    //      Using try_clone() instead of clone() because clone() is not implemented 
                                    //      clone() = Deep Copy and Trait
                                    //      try_clone()= Specific for few types: 
                                    //                      File, TcpStream, TcpListener, UdpSocket
                                    //                      And it's not a Trait 
                                    match socket.try_clone() {
                                        Ok(sock) => this.clone()._listen_port(ChannelType::UDP(sock)),
                                        Err(_) => {
                                            godot_print!("Error cloning the socket\n");
                                            println!("Error cloning the socket")
                                        },
                                    }                     
                                }
                            }
                        }
                    }
                    
                }
        });
        
        //Store the channel thread in a class variable to be able to correctly end the thread
        match self.clone().base_config.worker_config.worker_thread.try_lock() {
            Ok(mut worker_thread) => {
                if worker_thread.is_none() {
                    match running_thread {
                        Ok(trd) => {*worker_thread= Some(trd)},
                        Err(_) => {
                            godot_print!("Error in the UDP thread\n")
                        },
                    }
                    
                    
                }
            },
            Err(_) => {
                godot_print!("Thread still lock\n");
                println!("Thread still lock");
            },
        }
    }

    fn stop(self: Arc<Self>) {
        //Force thread stop by loop as long as the running mode mutex is not available
        loop {
            if let Ok(mut is_running) = self.clone().base_config.worker_config.is_running.try_lock() {
                *is_running= false;
                break;
            }
        }
        
    }
    
    fn end(self: Arc<Self>) {
        self.clone().stop();
        match self.clone().base_config.worker_config.worker_thread.try_lock() {
            Ok(mut worker_thread) => {
                match worker_thread.take() {
                    Some(thread) => {
                        thread.join();
                        //*worker_thread= None;
                        godot_print!("Delete the channel thread\n");
                        println!("Delete the channel thread")
                    },
                    None => {
                        godot_print!("No Existing Thread\n");
                        println!("No Existing Thread")
                    },
                }
            },
            Err(_) => {
                godot_print!("Thread still lock\n");
                println!("Thread still lock");
            },
        }
    }
}

impl Channel for UDPChannel {
    
    fn _connect(self: Arc<Self>) -> Option<ChannelType>{
        match UdpSocket::bind(format!("{}:{}", self.clone().base_config.address.clone(), self.clone().port.clone())) {
            //IF the socket is created
            Ok(s) => {
                //Set socket non blocking mod
                s.set_nonblocking(true).expect("Failed to set socket to non-blocking mode");
                godot_print!("Connected at {}:{}\n", self.clone().base_config.address, self.clone().port);
                println!("Connected at {}:{}", self.clone().base_config.address, self.clone().port);
                return Some(ChannelType::UDP(s));
            },
            Err(e) => { 
                godot_print!("Not able to create Socket at {}:{}\n", self.clone().base_config.address, self.clone().port);
                println!("Not able to create Socket at {}:{}", self.clone().base_config.address, self.clone().port);
                return None;
            }
        };
    }

    fn send_message(self: Arc<Self>, port: ChannelType, msg: Message) -> Result<bool, io::Error> {
        let mut frame:Vec<u8>= Vec::new();
        match msg {
            Message::Sentence(s) => frame= s.as_bytes().to_vec(),
            Message::Frame(f) => frame= f,
        }
        if let ChannelType::UDP(socket) = port {
            return socket
                .send_to(&frame, format!("{}:{}", self.clone().target_address, self.clone().target_port))
                .map(|size| size > 0);
        } else {
            return Ok(false);
        }
    }

    fn _listen_port(self: Arc<Self>, port: ChannelType) {
        let mut buf = [0; 1024];
        if let ChannelType::UDP(socket) = port {
            match socket.recv_from(&mut buf) {
                Ok((size, src)) => {
                    self.clone().base_config.worker_config.process_config.data_event.trig(Message::Frame(buf.to_vec()));
                },
                Err(e) => {
                    //The error WouldBlock is raised when there is no data
                    if e.kind() != ErrorKind::WouldBlock {
                        godot_print!("Error listening the socket");
                        println!("Error listening the socket");
                    }
                },
            }
        }
        
    }
}