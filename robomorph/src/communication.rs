use std::{io::{self, Error}, net::UdpSocket, sync::{Arc, Mutex}};

use socket2::SockRef;

use crate::core::{event_management::{Event, Observer}, worker::Module};

const MAX_RECEPTION_BUFFER: usize= 2*1024*5012;

pub trait Channel : Module {
    fn connect(&self);
    fn send_message(&self, msg: Vec<u8>) ;
    fn listen_for_frame(&self) -> Result<Vec<u8>, io::Error> ;
    fn close(&self);
}

pub struct UDPChannel {
    addr: String,
    target_addr: String,
    port: u32,
    target_port: u32,
    socket: Arc<Mutex<Option<UdpSocket>>>,
    cmd_observer: Mutex<Option<Observer<Vec<u8>>>>,
    frame_event: Mutex<Event<Vec<u8>>>
}

impl UDPChannel {
    pub fn new(addr: &str, port: u32, target_addr: &str, target_port: u32) -> Arc<Self> {
        let udp= Arc::new(Self{addr: addr.to_string(),port: port, socket: Arc::new(Mutex::new(None)),
                                                            frame_event: Mutex::new(Event::new_empty()), cmd_observer: Mutex::new(None),
                                                        target_addr: target_addr.to_string(), target_port: target_port});
        let udp_cl= udp.clone();
        let obs= Observer::new(Arc::new(Mutex::new(move |x: Vec<u8>| {
            udp_cl.send_message(x);
        })));
        if let Ok(mut frame_observer) = udp.cmd_observer.try_lock() {
            *frame_observer= Some(obs);
        }
        return udp;
    }

    pub fn new_async(addr: &str, port: u32, target_addr: &str, target_port: u32) -> Arc<Self> {
        return Arc::new(Self{addr: addr.to_string(),port: port, socket: Arc::new(Mutex::new(None)),
                                                        target_addr: target_addr.to_string(), target_port: target_port,
                                                    frame_event: Mutex::new(Event::new_empty()), cmd_observer: Mutex::new(Some(Observer::new_async()))});
    }

    pub fn get_cmd_observer(&self) -> Option<Observer<Vec<u8>>> {
        if let Ok(observer) = self.cmd_observer.try_lock() {
            if let Some(obs) = observer.as_ref() {
                return Some(obs.clone());
            } else {
                return None;
            }
                
        }
        else {
            return None;
        }
    }

    pub fn publish_message(&self, msg: Vec<u8>) {
        if let Ok(cmd_observer) = self.cmd_observer.try_lock() {
            if let Some(observer) = cmd_observer.as_ref() {
                observer.put_data_in_buffer(msg);
            }
        }
    }

    pub fn add_frame_observer(&self, data_observer: Observer<Vec<u8>>) {
        if let Ok(mut frame_event)= self.frame_event.try_lock() {
            frame_event.plug_observer(data_observer);
        }
    }
}

impl Channel for UDPChannel {
    fn connect(&self) {
        match UdpSocket::bind(format!("{}:{}", self.addr.clone(), self.port.clone())) {
            //IF the socket is created
            Ok(s) => {
                //Set socket non blocking mod
                s.set_nonblocking(true).expect("Failed to set socket to non-blocking mode");
                //Set the max buffer size for the UDP reception (OS buffer size, not buffer size of the listening operation)
                let socket_ref= SockRef::from(&s);
                if let Ok(())= socket_ref.set_recv_buffer_size(MAX_RECEPTION_BUFFER) {
                    println!("Connected at {}:{}", self.addr.clone(), self.port.clone());
                    if let Ok(mut socket) = self.socket.try_lock() {
                        *socket= Some(s);
                    }
                }
            },
            Err(_) => { 
                println!("Not able to create Socket at {}:{}", self.addr.clone(), self.port.clone());
            }
        };
    }

    fn send_message(&self, msg: Vec<u8>) {
        match self.socket.clone().try_lock() {
            //IF socket mutex is avalable to take, take it to be able to send frame
            Ok(socket) => {
                match socket.as_ref() {
                    //IF socket is Some, it means the interface can send cmd through UDP
                    Some(sock) => {
                        if let Err(_)= sock.send_to(&msg, format!("{}:{}", self.target_addr.clone(), self.target_port.clone())) {
                            println!("ERROR: Failed to send message");
                        } /*else {
                            println!("{}:{} -> Send: {:?} To {}:{}", self.addr.clone(), self.port.clone(), String::from_utf8(msg), self.target_addr.clone(), self.target_port.clone());
                        }*/
                    },
                    //IF socket is None, it means the interface is not connected
                    None => {

                    },
                }
            },
            Err(_) => {
                println!("WARNING: Can't send message because Socket mutex has already been taken");
                println!("INFO: Frame is stored in the cmd queue, it will be sent when mutex will be available");
                match self.cmd_observer.try_lock() {
                    Ok(observer) => {
                        if let Some(obs) = observer.as_ref() {
                            obs.put_data_in_buffer(msg);
                        }
                    },
                    Err(_) => {
                        println!("")
                    },
                }
            },
        }
    }

    fn listen_for_frame(&self)  -> Result<Vec<u8>, io::Error> {
        let mut buf = [0; 5000];
        match self.socket.clone().try_lock() {
            //IF socket mutex is availabe, take it to be able to listen for incoming frame
            Ok(socket) => {
                match socket.as_ref() {
                    //IF socket is some, it means interface can listen for incoming frames
                    Some(sock) => {
                        match sock.recv_from(&mut buf) {
                            //IF frame is received, return it
                            Ok((bytes_received, _)) => {
                               //println!("Received Data");
                                /*match String::from_utf8(buf[..bytes_received].to_vec()) {
                                    Ok(utf8_frame) => {
                                        println!("{}:{} -> Received {:?} from {}:{}", self.addr.clone(), self.port.clone(), utf8_frame, src_addr.ip().to_string(), src_addr.port());    
                                    },
                                    Err(_) => {
                                        println!("{}:{} -> Received {:?} from {}:{}", self.addr.clone(), self.port.clone(), buf[..bytes_received].to_vec(), src_addr.ip().to_string(), src_addr.port());    
                                    },
                                }*/
                                return Ok(buf[..bytes_received].to_vec())
                            },
                            //IF there is error, return the error /!\WARNING: No incoming frame trigger error WouldBlock (because socket is set in non blocking mode)
                            Err(e) => {
                                //println!("No Data Received");
                                return Err(e);
                            },
                        }
                    }
                    //IF socket is none, it means interface is not connected, return empty frame
                    None => {
                        return Ok(Vec::new());
                    },
                }
            }
            Err(_) => {
                return Err(Error::new(io::ErrorKind::Other, "WARNING: Can't listen for frame because Socket mutex has already been taken"));
            },
        }
    }

    fn close(&self) {
        //Loop until socket is not clode
        loop {
            match self.socket.clone().try_lock() {
                Ok(mut socket) => {
                    *socket= None;
                    break;
                }
                Err(_) => {},
            }
        }
    }
}

impl Module for UDPChannel {
    fn exec_main_task(&self) {
        let mut is_connected= -1; 
        match self.socket.clone().try_lock() {
            Ok(sock) => {
                match sock.as_ref() {
                    Some(_) => {is_connected= 1;},
                    None => {is_connected= 0;},
                }
            },
            Err(_) => {
                is_connected= -1
            },
        }
        if is_connected == 1  {
            //IF data has been received and frame_event is available
            //  Send the frame to the modules linked to the frame event
            if let Ok(frame) = self.listen_for_frame() && let Ok(frame_event)= self.frame_event.try_lock() {
                frame_event.trig(frame);
            }
            //IF the cmd observer is available
            if let Ok(mut cmd_observer)= self.cmd_observer.try_lock() && 
                        let Some(observer) = cmd_observer.as_mut() {
                //IF there is data in the buffer(observer is asynchrone)
                //  Send the cmd through UDP
                while observer.is_data_in_buffer() {
                    if let Some(cmd)= observer.get_incoming_data() {
                        self.send_message(cmd);
                    }
                }
            }
        } else if is_connected == 0 {
            self.connect();
        }
    }
}