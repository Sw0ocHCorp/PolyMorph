use std::net::{SocketAddr, UdpSocket};
use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::Duration;

use tokio::sync::{broadcast, mpsc};

use crate::communications::interface::{HardwareInterface, decode_frame, encode_frame};
use crate::core::scheduler::Process;
use crate::messages::registered_message::{AnyMessage, MessageType, Translatable};
use crate::messages::{
    lidar_messages::LidarMeasurements,
    motor_messages::MotorFeedBack,
    pose_messages::{GNSSMeasurement, IMUMeasurements, Pose},
};
/// Struct that represent an UDP hardware interface
pub struct UdpInterface {
    /// Address of the interface
    src_addr: String,
    /// Adress of the device connected to the interface
    dest_addr: String,
    /// Adress of the device connected to the interface
    dest_socket_addr: Option<SocketAddr>,
    /// frame sender to the connected processes
    inbound_tx:  broadcast::Sender<AnyMessage>,
    /// frame sender to the interface
    ///
    /// processes -> interface (clone() passed to the processes that can send frames to the interface)
    outbound_tx: mpsc::Sender<AnyMessage>,
    /// receiver that listen frame from the processes
    outbound_rx: Option<mpsc::Receiver<AnyMessage>>,
    /// Hardware socket to listen / send packet to the connected device / endpoint
    socket:    Option<Arc<UdpSocket>>,
    /// RX thread:
    /// 
    /// listen for incomming frames and send them to processes
    rx_handle: Option<JoinHandle<()>>,
    /// TX thread:
    /// 
    /// send the waiting frames from processes to the device / endpoint
    tx_handle: Option<JoinHandle<()>>,
}

impl UdpInterface {
    /// Constructor
    pub fn new(src_addr: &str, src_port: u16, dest_addr: &str, dest_port: u16, buffer_capacity: usize) -> Self {
        let (inbound_tx, _) = broadcast::channel(buffer_capacity);
        let (outbound_tx, outbound_rx) = mpsc::channel(buffer_capacity);
        Self {
            src_addr: format!("{}:{}", src_addr, src_port),
            dest_addr: format!("{}:{}", dest_addr, dest_port),
            dest_socket_addr: None,
            inbound_tx,
            outbound_tx,
            outbound_rx: Some(outbound_rx),
            socket: None,
            rx_handle: None,
            tx_handle: None,
        }
    }

    /// get the object that receive data from the interface
    pub fn get_inbound_receiver(&self) -> broadcast::Receiver<AnyMessage> {
        self.inbound_tx.subscribe()
    }
}

impl HardwareInterface for UdpInterface {
    /// Connect the interface to the device / endpoint
    fn connect(&mut self) -> Result<(), String> {
        let dest: SocketAddr = self.dest_addr
            .parse()
            .map_err(|e| format!("invalid dest addr '{}': {}", self.dest_addr, e))?;

        let outbound_rx = self.outbound_rx
            .take()
            .ok_or_else(|| "connect() called twice".to_string())?;

        let socket = UdpSocket::bind(&self.src_addr)
            .map_err(|e| format!("UDP bind failed on '{}': {}", self.src_addr, e))?;

        let socket = Arc::new(socket);
        self.socket = Some(Arc::clone(&socket));
        self.dest_socket_addr = Some(dest);

        // RX thread — bloquant pur, se débloque via poison pill dans disconnect()
        let rx_socket = Arc::clone(&socket);
        let rx_tx     = self.inbound_tx.clone();
        self.rx_handle = Some(thread::spawn(move || {
            let mut buf = [0u8; 4096];
            loop {
                match rx_socket.recv_from(&mut buf) {
                    Ok((0, _)) => break,
                    Ok((len, _)) => {
                        if let Some(msg) = decode_frame(&buf[..len]) {
                            let _ = rx_tx.send(msg);
                        }
                    }
                    Err(e) => { eprintln!("[UdpInterface] RX error: {}", e); break; }
                }
            }
        }));

        // TX thread — se termine quand tous les mpsc::Sender sont droppés
        let tx_socket  = Arc::clone(&socket);
        let mut tx_rx  = outbound_rx;
        self.tx_handle = Some(thread::spawn(move || {
            while let Some(msg) = tx_rx.blocking_recv() {
                if let Err(e) = tx_socket.send_to(&encode_frame(&msg), dest) {
                    eprintln!("[UdpInterface] TX send error: {}", e);
                }
            }
        }));

        Ok(())
    }

    /// Send data to the endpoint / device
    /// 
    /// Notes:
    /// 
    /// msg= the message to send
    fn send_message(&mut self, msg: AnyMessage) {
        match (&self.socket, self.dest_socket_addr) {
            (Some(socket), Some(dest)) => {
                if let Err(e) = socket.send_to(&encode_frame(&msg), dest) {
                    eprintln!("[UdpInterface] send_message error: {}", e);
                }
            }
            _ => eprintln!("[UdpInterface] send_message called before connect()"),
        }
    }

    fn listen(&mut self) -> Option<AnyMessage> {
        None
    }

    /// Interface disconnect routine
    /// 
    /// Notes:
    /// 
    /// The Interface must be mutable because RX / TX threads are udated to be stopped
    fn disconnect(&mut self) {
        // Send empty packet to stop blocking RX THread
        if let Ok(wake) = UdpSocket::bind("0.0.0.0:0") {
            let _ = wake.send_to(&[], &self.src_addr);
        }
        // Join the RX / TX Threads
        if let Some(h) = self.rx_handle.take() { let _ = h.join(); }
        if let Some(h) = self.tx_handle.take() { let _ = h.join(); }
        self.socket = None;
        self.dest_socket_addr = None;
    }

    /// set the receiver that listen frame from the process
    fn set_outbound_rx(&mut self, receiver: mpsc::Receiver<AnyMessage>) {
        self.outbound_rx = Some(receiver);
    }

    /// Connect the interface to a given process
    /// 
    /// Arguments: 
    /// 
    /// proc= Process to connect to
    fn connect_process(&mut self, proc: &mut dyn Process) {
        // put interface (device / endpoint) frame receiver in the process 
        proc.set_inbound_receiver(self.inbound_tx.subscribe());
        // put the process frame sender to the process the interface connected with it
        proc.set_outbound_sender(self.outbound_tx.clone());
    }
}

// Drop implementation allow to execute some code just before the object is destroyed 
impl Drop for UdpInterface {
    fn drop(&mut self) {
        if self.socket.is_some() {
            self.disconnect();
        }
    }
}
