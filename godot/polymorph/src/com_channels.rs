use godot::global::godot_print;

use crate::{flight_controller::ImageConfig, process::{Work, Worker}};
use std::{collections::VecDeque, io::{self, ErrorKind}, sync::{Arc, Mutex}, thread::{self}};
use std::net::UdpSocket;

#[derive(Clone)]
pub enum Message {
    Sentence(String),
    Frame(Vec<u8>),
    Image(ImageConfig),
}

pub enum ChannelType {
    UDP(UdpSocket)
}

pub struct ChannelConfig {
    pub address: String,
    pub worker: Option<Worker>,
    pub message_buffer: Mutex<VecDeque<Message>>,
}

impl ChannelConfig {
    
    pub fn new(address: String) -> Self {
        ChannelConfig {
            address,
            worker: None,
            message_buffer: Mutex::new(VecDeque::new()),
        }
    }

    pub fn set_worker(&mut self, worker: Worker) {
        self.worker= Some(worker);
    }
}

pub trait Channel : Work{
    fn _connect(self: Arc<Self>) -> Option<ChannelType>;
    fn send_message(self: Arc<Self>, port: ChannelType, msg: Message) -> Result<bool, io::Error>;
    fn _listen_port(self: Arc<Self>, port: ChannelType) -> Result<Vec<u8>, io::Error>;
    fn add_message_in_queue(self: Arc<Self>, msg: Message);
}

pub struct UDPChannel {
    base_config: Mutex<ChannelConfig>,
    socket: Mutex<Option<UdpSocket>>,
    port: u32,
    target_address: String,
    target_port: u32
} 

impl UDPChannel {
    pub fn new(base_config: ChannelConfig, port: u32, target_address: String, target_port: u32) -> Arc<Self> {
        return Arc::new(UDPChannel {base_config: Mutex::new(base_config), port, target_address, target_port, socket: Mutex::new(None)});
    } 

    pub fn set_worker(self: Arc<Self>, worker: Worker) {
        if let Ok(mut config) = self.clone().base_config.try_lock() {
            config.set_worker(worker);
        }
    }
}

impl Work for UDPChannel {
    fn start_task(self: Arc<Self>) {
        //IF the socket variable is available (the mutex is available)
        if let Ok(mut socket_guard) = self.clone().socket.try_lock() {
            //IF no socket
            //  /!\ socket_guard.is_none() is prefered than socket_guard.take() 
            //  because socket_guard.take() force to None after the enclosure
            //  socket_guard.is_none() use socket_guard as ref
            //AND  socket created by the _connect() function
            if socket_guard.is_none() && let Some(ChannelType::UDP(sock)) =  self.clone()._connect(){
                //Store the socket created
                *socket_guard= Some(sock);
            }
            //IF the socket exist (not NONE) -> Send the data in the buffer and listen the incoming frames
            if let Some(ref sock) = *socket_guard {
                //IF the message buffer variable is available
                //  Sending Buffer Data
                if let Ok(mut config) = self.clone().base_config.try_lock() && let Ok(mut message_buffer) = config.message_buffer.try_lock() {
                    //message_buffer.push_back(Message::Frame("CAVA".as_bytes().to_vec()));
                    //Send and consum all the data in the buffer IF the socket can be cloned for each data in the buffer
                    while  message_buffer.len() > 0 && let Ok(s)= sock.try_clone() {
                        if let Some(msg_to_send)= message_buffer.pop_front() {
                            self.clone().send_message(ChannelType::UDP(s), msg_to_send);
                        }
                    }
                }
                //Data Listening
                //Get the socket instance by cloning the ref of the data protected by the mutex
                //And call the listening with this UDP socket 
                //      Using try_clone() instead of clone() because clone() is not implemented 
                //      clone() = Deep Copy and Trait
                //      try_clone()= Specific for few types: 
                //                      File, TcpStream, TcpListener, UdpSocket
                //                      And it's not a Trait 
                while let Ok(s)= sock.try_clone() && let Ok(mut config) = self.clone().base_config.try_lock()  && let Some(worker)= config.worker.as_ref(){
                    match self.clone()._listen_port(ChannelType::UDP(s)) {
                        Ok(frame_received) =>  {
                            if frame_received.len() > 0 {
                                worker.process_config.data_event.trig(Message::Frame(frame_received));   
                            }
                        },
                        Err(e) => {
                            //IF the error trigger because No data was received
                            if e.kind() == ErrorKind::WouldBlock {
                            }
                            //IF the error trigger because No devices connected
                            if e.kind() == ErrorKind::ConnectionReset {

                            }
                            // Stop loop because no data are received or there is error with the socket
                            break;
                        },
                    }
                    
                }
            }
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
            Message::Image(image_config) => todo!(),
        }
        if let ChannelType::UDP(socket) = port {
            match socket
                .send_to(&frame, format!("{}:{}", self.clone().target_address, self.clone().target_port)) {
                    Ok(_) => { 
                        return Ok(true);
                    },
                    Err(e) => {
                        let test= e.kind();
                        let test2= test.to_string();
                        let a= 1;
                        return Err(e);
                    },
                } 
                
        } else {
            return Ok(false);
        }
    }

    fn _listen_port(self: Arc<Self>, port: ChannelType) -> Result<Vec<u8>, io::Error> {
        let mut buf = [0; 1024];
        if let ChannelType::UDP(socket) = port {
            match socket.recv_from(&mut buf) {
                Ok((size, src)) => {
                    //self.clone().base_config.worker_config.process_config.data_event.trig(Message::Frame());
                    return Ok(buf[..size].to_vec());
                },
                Err(e) => {
                    //The error WouldBlock is raised when there is no data
                    if e.kind() != ErrorKind::WouldBlock {
                        return Ok(Vec::new())
                    } else {
                        return Err(e);
                    }
                },
            }
        }
        else {
            return Err(io::Error::new(ErrorKind::NotFound, "Port type not supported"));
        }
        
    }

    fn add_message_in_queue(self: Arc<Self>, msg: Message) {
        if let Ok(mut message_buffer) = self.clone().base_config.message_buffer.try_lock() {
            message_buffer.push_back(msg);
        }
    }
}