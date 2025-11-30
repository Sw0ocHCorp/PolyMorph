use crate::{messages::Message, process::{ModuleLinker, Process}};
use std::{collections::VecDeque, io::{self, ErrorKind}, sync::{Arc, Mutex}};
use std::net::UdpSocket;



pub enum ChannelType {
    UDP(UdpSocket)
}

pub struct ChannelConfig {
    pub address: String,
    pub linker: Arc<Mutex<ModuleLinker>>,
    pub message_buffer: Mutex<VecDeque<Message>>,
}

impl ChannelConfig {
    pub fn new(address: String, linker: ModuleLinker) -> Self {
        ChannelConfig {
            address,
            linker: Arc::new(Mutex::new(linker)),
            message_buffer: Mutex::new(VecDeque::new()),
        }
    }
}

/**
 * Channel Trait
 * Defines the basic fondamental functions of a communication channel
 * Channel : Process means that every struct implementing Channel must also implement Process
 * The process will be the way that the channel manages the communication
 */
pub trait Channel : Process{
    fn _connect(self: Arc<Self>) -> Option<ChannelType>;
    fn send_message(self: Arc<Self>, port: ChannelType, msg: Message) -> Result<bool, io::Error>;
    fn _listen_port(self: Arc<Self>, port: ChannelType) -> Result<Vec<u8>, io::Error>;
}

pub struct UDPChannel {
    pub chan_config: ChannelConfig,
    socket: Mutex<Option<UdpSocket>>,
    port: u32,
    target_address: String,
    target_port: u32,
    pub frequency: u32,
} 


impl UDPChannel {
    pub fn new(chan_config: ChannelConfig, port: u32, target_address: String, target_port: u32, frequency: u32) -> UDPChannel {
        return UDPChannel {chan_config, port, target_address, target_port, socket: Mutex::new(None), frequency: frequency};
    } 

    pub fn get_socket(&self) -> Option<UdpSocket> {
        if let Ok(socket_guard) = self.socket.try_lock() {
            if let Some(ref socket) = *socket_guard {
                return Some(socket.try_clone().expect("Failed to clone UDP socket"));
            }
        }
        return None;
    }

    pub fn get_port(self: Arc<Self>) -> u32 {
        return self.clone().port;
    }

    pub fn get_target_address(self: Arc<Self>) -> String {
        return self.clone().target_address.clone();
    }
    pub fn get_target_port(self: Arc<Self>) -> u32 {
        return self.clone().target_port;
    }
}

impl Process for UDPChannel {
    fn exec_task(self: Arc<Self>) {
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
                if let Ok(mut message_buffer) = self.clone().chan_config.message_buffer.try_lock() {
                    //LET IT FOR TESTING
                    message_buffer.push_back(Message::Frame("CAVA".as_bytes().to_vec()));
                    //Send and consum all the data in the buffer IF the socket can be cloned for each data in the buffer
                    while  message_buffer.len() > 0 && let Ok(s)= sock.try_clone() {
                        if let Some(msg_to_send)= message_buffer.pop_front() {
                            match self.clone().send_message(ChannelType::UDP(s), msg_to_send) {
                                Ok(status) => {
                                    if status == true {
                                        println!("Data sent to {}:{}", self.clone().target_address, self.clone().target_port);
                                    }
                                },
                                Err(_) => {
                                    
                                },
                            }
                        }
                    }
                }
                let this= self.clone();
                //Data Listening
                //Get the socket instance by cloning the ref of the data protected by the mutex
                //And call the listening with this UDP socket 
                //      Using try_clone() instead of clone() because clone() is not implemented 
                //      clone() = Deep Copy and Trait
                //      try_clone()= Specific for few types: 
                //                      File, TcpStream, TcpListener, UdpSocket
                //                      And it's not a Trait 
                while let Ok(s)= sock.try_clone() {
                    match this.clone()._listen_port(ChannelType::UDP(s)) {
                        Ok(frame) => {
                            if frame.len() > 0 && let Ok(mut linker)= this.clone().chan_config.linker.try_lock() {
                                linker.send_message(Message::Frame(frame));
                                //self.clone().base_config.worker_config.process_config.data_event.trig(Message::Frame(frame));   
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
        match UdpSocket::bind(format!("{}:{}", self.clone().chan_config.address.clone(), self.clone().port.clone())) {
            //IF the socket is created
            Ok(s) => {
                //Set socket non blocking mod
                s.set_nonblocking(true).expect("Failed to set socket to non-blocking mode");
                println!("Connected at {}:{}", self.clone().chan_config.address, self.clone().port);
                return Some(ChannelType::UDP(s));
            },
            Err(e) => { 
                println!("Not able to create Socket at {}:{}", self.clone().chan_config.address, self.clone().port);
                return None;
            }
        };
    }

    fn send_message(self: Arc<Self>, port: ChannelType, msg: Message) -> Result<bool, io::Error> {
        let mut frame:Vec<u8>= Vec::new();
        match msg {
            Message::Sentence(s) => frame= s.as_bytes().to_vec(),
            Message::Frame(f) => frame= f,
            Message::Image() => {},
            Message::LidarMeasurements(items) => todo!(),
        }
        if let ChannelType::UDP(socket) = port {
            return socket
                .send_to(&frame, format!("{}:{}", self.clone().target_address, self.clone().target_port))
                .map(|size| size > 0);
        } else {
            return Ok(false);
        }
    }

    fn _listen_port(self: Arc<Self>, port: ChannelType) -> Result<Vec<u8>, io::Error> {
        let mut buf = [0; 1024];
        if let ChannelType::UDP(socket) = port {
            match socket.recv_from(&mut buf) {
                Ok((size, src)) => {
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
}

