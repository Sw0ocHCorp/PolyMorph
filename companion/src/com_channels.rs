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
    fn _listen_port(self: Arc<Self>, port: ChannelType) -> Result<Vec<u8>, io::Error>;
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
                if let Ok(mut message_buffer) = self.clone().base_config.message_buffer.try_lock() {
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
                //Data Listening
                //Get the socket instance by cloning the ref of the data protected by the mutex
                //And call the listening with this UDP socket 
                //      Using try_clone() instead of clone() because clone() is not implemented 
                //      clone() = Deep Copy and Trait
                //      try_clone()= Specific for few types: 
                //                      File, TcpStream, TcpListener, UdpSocket
                //                      And it's not a Trait 
                while let Ok(s)= sock.try_clone() {
                    match self.clone()._listen_port(ChannelType::UDP(s)) {
                        Ok(frame) => {
                            if frame.len() > 0 {
                                self.clone().base_config.worker_config.process_config.data_event.trig(Message::Frame(frame));   
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

    fn start_routine(self: Arc<Self>) {
        let mut is_connected = false;
        let this: Arc<UDPChannel>= self.clone();
        /*if let Ok(mut buffer) =self.clone().base_config.message_buffer.try_lock() {
            buffer.push_back(Message::Frame("CAVA".as_bytes().to_vec()));
        }*/
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
                            println!("Soft Stop channel thread");
                            break;
                        }
                    } else {
                        println!("Is_Running already used");
                    }
                    this.clone().start_task();
                    
                }
        });
        
        //Store the channel thread in a class variable to be able to correctly end the thread
        match self.clone().base_config.worker_config.worker_thread.try_lock() {
            Ok(mut worker_thread) => {
                if worker_thread.is_none() {
                    match running_thread {
                        Ok(trd) => {*worker_thread= Some(trd)},
                        Err(_) => {
                            println!("Error in the UDP thread\n")
                        },
                    }
                    
                    
                }
            },
            Err(_) => {
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
                        println!("Delete the channel thread")
                    },
                    None => {
                        println!("No Existing Thread")
                    },
                }
            },
            Err(_) => {
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
                println!("Connected at {}:{}", self.clone().base_config.address, self.clone().port);
                return Some(ChannelType::UDP(s));
            },
            Err(e) => { 
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
}