//! UDP implementation of `HardwareInterface`.
//!
//! `connect()` binds the local socket and spawns two threads:
//! * RX: blocks in `recv_from`, decodes each datagram (`decode_frame`) and publishes the
//!   `AnyMessage` on `frame_sender`; it stops on an EMPTY datagram (the "poison pill" that
//!   `disconnect()` sends to the local address) or on a socket error;
//! * TX: drains `frame_receiver` (a broadcast subscription) and sends every message, encoded
//!   with `encode_frame`, to the destination address.
//!
//! The interface is registered with `Scheduler::register_interface`, never as a process: the
//! `Process` impl below only exists so that it can be wired with the same `set_sender` /
//! `set_receiver` calls as a process.

use std::net::{SocketAddr, UdpSocket};
use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::Duration;

use tokio::sync::{broadcast, mpsc};

use crate::communications::interface::{HardwareInterface, decode_frame, encode_frame};
use crate::core::scheduler::Process;
use crate::messages::registered_message::AnyMessage;

/// Struct that represent an UDP hardware interface
///
/// Life cycle: `new` -> `set_sender` / `set_receiver` -> `connect` -> (threads run) ->
/// `disconnect` (also called by `Drop` if still connected).
pub struct UdpInterface {
    /// Name reported by `Process::get_name` (empty by default; set with `set_name`).
    name: String,
    /// Address of the interface
    /// (`ip:port` the local socket is bound to; also the target of the poison pill in `disconnect`)
    src_addr: String,
    /// Adress of the device connected to the interface
    /// (`ip:port` as a string, parsed at `connect`)
    dest_addr: String,
    /// Adress of the device connected to the interface
    /// (parsed form of `dest_addr`; `Some` while connected)
    dest_socket_addr: Option<SocketAddr>,
    /// processes -> interface (clone() passed to the processes that can send frames to the interface)
    ///
    /// Set by `Process::set_sender`. The RX thread publishes every decoded frame on it (a clone is
    /// moved into the thread).
    frame_sender: Option<broadcast::Sender<AnyMessage>>,
    /// receiver that listen frame from the processes
    ///
    /// Set by `Process::set_receiver`, taken out of `self` and moved into the TX thread by
    /// `connect()`, which therefore fails if it is missing or if it is called twice.
    // NOTE: when the sender and the receiver are wired to the SAME channel (as the Gazebo binary does
    // with the telemetry channel), every datagram received by RX is re-published on that channel,
    // picked up by TX and echoed back to the destination.
    frame_receiver: Option<broadcast::Receiver<AnyMessage>>,
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
    /// Period reported through `Process`; unused since the interface is never scheduled.
    period: Duration,
}

impl UdpInterface {
    /// Constructor
    ///
    /// `src_addr:src_port` is the local bind address, `dest_addr:dest_port` the remote endpoint.
    /// Nothing is opened here: see `connect`.
    // NOTE: `buffer_capacity` is accepted but not used (the RX buffer is a fixed 4096-byte array and
    // the channel capacity is chosen by whoever creates the broadcast channel).
    pub fn new(src_addr: &str, src_port: u16, dest_addr: &str, dest_port: u16, buffer_capacity: usize) -> Self {
        Self {
            name: "".to_string(),
            src_addr: format!("{}:{}", src_addr, src_port),
            dest_addr: format!("{}:{}", dest_addr, dest_port),
            dest_socket_addr: None,
            frame_sender: None,
            frame_receiver: None,
            socket: None,
            rx_handle: None,
            tx_handle: None,
            period: Duration::from_millis(0)
        }
    }
}

/// `Process` is implemented only for the channel wiring (`set_sender` / `set_receiver`) and the
/// name; the interface is never registered in a `ProcessesChain`.
impl Process for UdpInterface {

    // NOTE: `todo!()`: registering the interface in a chain would panic at its first tick. The
    // interface is not scheduled; its work is done by the RX / TX threads started in `connect`.
    fn exec(&mut self, _input: &Option<AnyMessage>, _dt: std::time::Duration) -> Option<AnyMessage> {
        todo!()
    }

    /// Subscription drained by the TX thread (frames processes -> device).
    fn set_receiver(&mut self, receiver: broadcast::Receiver<AnyMessage>) {
        self.frame_receiver= Some(receiver);
    }

    /// Channel the RX thread publishes the received frames on (device -> processes).
    fn set_sender(&mut self, sender: tokio::sync::broadcast::Sender<AnyMessage>) {
        self.frame_sender= Some(sender);
    }

    fn set_period_from_freq(&mut self, frequency: u64) {
        self.period= Duration::from_nanos(1_000_000_000 / frequency);
    }

    fn get_period(&self) -> std::time::Duration {
        return self.period;
    }

    fn set_name(&mut self, name: String) {
        self.name= name;
    }
    fn get_name(&self) ->String {
        return self.name.clone();
    }
}

impl HardwareInterface for UdpInterface {
    /// Connect the interface to the device / endpoint
    ///
    /// Parses the destination, takes `frame_receiver` out of `self`, binds `src_addr`, then spawns
    /// the RX and TX threads described in the module doc. Fails on an invalid destination, a
    /// missing receiver (`set_receiver` not called, or a second `connect`) or a bind error.
    // NOTE: the receiver is taken out of `self` BEFORE the bind, so a bind failure leaves the
    // interface without receiver: a retry of `connect` then fails with "connect() called twice".
    fn connect(&mut self) -> Result<(), String> {
        let dest: SocketAddr = self.dest_addr
            .parse()
            .map_err(|e| format!("invalid dest addr '{}': {}", self.dest_addr, e))?;

        // taken out of `self`, not borrowed: the receiver is moved into the TX thread below,
        // which outlives this call. A second connect() finds None and reports it.
        let mut frame_receiver = self.frame_receiver
            .take()
            .ok_or_else(|| "connect() called twice".to_string())?;

        let socket = UdpSocket::bind(&self.src_addr)
            .map_err(|e| format!("UDP bind failed on '{}': {}", self.src_addr, e))?;

        let socket = Arc::new(socket);
        self.socket = Some(Arc::clone(&socket));
        self.dest_socket_addr = Some(dest);

        // RX thread — bloquant pur, se débloque via poison pill dans disconnect()
        // RX thread: purely blocking `recv_from`; unblocked by the empty datagram sent by `disconnect()`
        // (`Ok((0, _))` below) or by a socket error. Undecodable frames are silently ignored.
        let rx_socket = Arc::clone(&socket);
        let frame_sender     = self.frame_sender.clone();
        self.rx_handle = Some(thread::spawn(move || {
            let mut buf = [0u8; 4096];
            loop {
                match rx_socket.recv_from(&mut buf) {
                    Ok((0, _)) => break,
                    Ok((len, _)) => {
                        // `&frame_sender`: the Option is owned by the closure and re-read at every
                        // packet, so it must be borrowed here, not moved out on the first one.
                        if let Some(msg) = decode_frame(&buf[..len]) && let Some(sender)= &frame_sender {
                            let _ = sender.send(msg);
                        }
                    }
                    Err(e) => { eprintln!("[UdpInterface] RX error: {}", e); break; }
                }
            }
        }));

        // TX thread — se termine quand tous les mpsc::Sender sont droppés
        // TX thread: forwards every frame received from the broadcast subscription to `dest`.
        // The channel is a tokio `broadcast` (not mpsc): `blocking_recv` returns `Err(Closed)` once every
        // `Sender` is dropped and `Err(Lagged(n))` when this receiver fell behind by n messages.
        // NOTE: `while let Ok(..)` exits on BOTH errors, so a single `Lagged` (a slow consumer on a
        // small-capacity channel) ends the TX thread permanently and silently. `Lagged` means "messages
        // were skipped, keep receiving", not "stop".
        let tx_socket  = Arc::clone(&socket);
        self.tx_handle = Some(thread::spawn(move || {
            while let Ok(msg) = frame_receiver.blocking_recv() {
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
    ///
    /// Synchronous send on the caller's thread, bypassing the TX thread (and its channel).
    /// Logs and drops the message if the interface is not connected.
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

    /// Always `None`: incoming frames are pushed on `frame_sender` by the RX thread instead.
    fn listen(&mut self) -> Option<AnyMessage> {
        None
    }

    /// Interface disconnect routine
    /// 
    /// Notes:
    /// 
    /// The Interface must be mutable because RX / TX threads are udated to be stopped
    ///
    /// Sends an empty datagram to the local address so that the RX thread returns from `recv_from`,
    /// then joins both threads and drops the socket. Safe to call when not connected.
    // NOTE: joining the TX thread only returns once its loop exits, i.e. once every `Sender` of the
    // channel `frame_receiver` subscribes to is dropped (or a `Lagged` error occurred). In the Gazebo
    // binary that channel is also held by `self.frame_sender` and by the vehicle controller, so this
    // join can block indefinitely.
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

}
// Drop implementation allow to execute some code just before the object is destroyed 
/// Disconnects (poison pill + joins) if the interface is still connected when it is dropped.
impl Drop for UdpInterface {
    fn drop(&mut self) {
        if self.socket.is_some() {
            self.disconnect();
        }
    }
}
