use std::sync::{Arc, atomic::{AtomicBool, Ordering}, mpsc};
use std::time::{Duration, Instant};

use tokio::sync::broadcast::{Receiver, Sender, channel};

use crate::{communications::interface::HardwareInterface, messages::registered_message::AnyMessage};

/// Trait that give the fundamental methods of all the processes
pub trait Process {
    /// Execute the process task. The main task of the process
    fn exec(&mut self, inputs: Option<AnyMessage>, dt: Duration) -> Option<AnyMessage>;
    /// Helper function that force the Process to have receiver object to receive cmd from other process or interface
    fn set_inbound_receiver(&mut self, receiver: Receiver<AnyMessage>);
    /// Helper function that force the Process to have sender object to send data to other process or interface
    fn set_outbound_sender(&mut self, sender: tokio::sync::mpsc::Sender<AnyMessage>);
}

/// Struct that represent a Processes Execution Chain
/// 
/// This execution chain runs the main tasks of the processes one after another in the order in which processes are registered in the chain.

//? before Sized because generic params is ALWAYS Sized 
//  but we want to use dyn Trait that is not Sized
//  ? before Sized allow to put Sized or not Sized type / trait as we want
pub struct ProcessesChain<P: ?Sized = dyn Process> {
    /// Id of the chain
    pub chain_id: u8,
    /// List of all the processes and the execution frequency ot their main task
    /// 
    /// The first process registered in the list is executed first, the second after and so on
    processes: Vec<(Box<P>, Duration)>,
}

impl<P: Process + ?Sized> ProcessesChain<P> {
    /// Constructor
    pub fn new(chain_id: u8) -> Self {
        return Self { chain_id, processes: vec![]};
    }

    /// Add process in the list of all the processes
    /// 
    /// Arguments:
    /// 
    /// process: 
    /// 
    /// The process to add and execute in the future
    /// 
    /// frequency_hz:
    /// 
    /// The execution frequency of the process main task 
    pub fn add_process(&mut self, process: Box<P>, frequency_hz: u64) {
        self.processes.push((process, Duration::from_millis(1000 / frequency_hz)));
    }

    /// Run the process execution chain one time 
    /// 
    /// Run processes one time / several times according to thir frequencies
    fn run_once(&mut self) -> Option<AnyMessage> {
        let mut state = None;
        let len = self.processes.len();
        for i in 0..len {
            let dt1 = self.processes[i].1;
            let repeat = if i + 1 < len {
                let dt1_ms = dt1.as_millis();
                let dt2_ms = self.processes[i + 1].1.as_millis();
                if dt1_ms > 0 && dt2_ms > dt1_ms {
                    (dt2_ms / dt1_ms).max(1) as usize
                } else {
                    1
                }
            } else {
                1
            };

            for _ in 0..repeat {
                let exec_start = Instant::now();
                let (process, _) = &mut self.processes[i];
                state = process.exec(state, dt1);
                if dt1 > Duration::ZERO {
                    let elapsed = exec_start.elapsed();
                    if elapsed < dt1 {
                        std::thread::sleep(dt1 - elapsed);
                    }
                }
            }
        }
        return state;
    }
}

/// Struct that represent the scheduler
/// 
/// The goal of the scheduler is to manage the processes chains 
pub struct Scheduler {
    /// The chain executed in the main thread (same as the scheduler)
    main_chain: ProcessesChain,
    /// The list of all the other chains that will run on other threads
    side_chains: Vec<ProcessesChain<dyn Process + Send>>,
    /// The required objects to run / stop side chains that lived in dedicated threads 
    side_chain_handles: Vec<(std::thread::JoinHandle<ProcessesChain<dyn Process + Send>>, Arc<AtomicBool>)>,
    /// The list of the interfaces used to communicated with other external softwares / devices
    interfaces: Vec<Box<dyn HardwareInterface>>,
}

impl Scheduler {
    /// Constructor
    pub fn new() -> Self {
        return Self {
            main_chain: ProcessesChain::new(0),
            side_chains: vec![],
            side_chain_handles: vec![],
            interfaces: vec![],
        };
    }

    /// Register process in the main chain
    /// 
    /// Arguments:
    /// 
    /// process:
    /// 
    /// The process to register
    /// 
    /// frequency_hz: 
    /// 
    /// The process main task execution frequency
    pub fn register_process(&mut self, process: Box<dyn Process>, frequency_hz: u64) {
        self.main_chain.add_process(process, frequency_hz);
    }

    /// Register process in side chain
    /// 
    /// Arguments:
    /// 
    /// process:
    /// 
    /// The process to register
    /// 
    /// frequency_hz: 
    /// 
    /// The process main task execution frequency
    /// 
    /// chain_id:
    /// 
    /// ID of the chain in which the process is to be recorded
    pub fn register_side_process(&mut self, process: Box<dyn Process + Send>, frequency_hz: u64, chain_id: u8) {
        let idx = (chain_id - 1) as usize;
        while self.side_chains.len() <= idx {
            self.side_chains.push(ProcessesChain::new(self.side_chains.len() as u8 + 1));
        }
        self.side_chains[idx].add_process(process, frequency_hz);
    }

    /// Register interface
    /// 
    /// Arguments:
    /// 
    /// interface:
    /// 
    /// The hardware interface to register
    pub fn register_interface(&mut self, interf: Box<dyn HardwareInterface>) {
        self.interfaces.push(interf);
    }

    /// Run main chain one time
    /// 
    /// Notes:
    /// 
    /// The scheduler must be to be mutable because the main chain also must be to be mutable in order to run.  
    pub fn run_main_chain(&mut self) {
        self.main_chain.run_once();
    }

    /// Run main chains in dedicated threads
    /// 
    /// Notes: 
    /// 
    /// The scheduler must be to be mutable because it create and stored dedicated threads for all the side chains
    pub fn start_all_side_chains(&mut self) {
        for chain in self.side_chains.drain(..) {
            let running = Arc::new(AtomicBool::new(true));
            let running_clone = Arc::clone(&running);
            let handle = std::thread::spawn(move || {
                let mut chain = chain;
                while running_clone.load(Ordering::Relaxed) {
                    chain.run_once();
                }
                return chain;
            });

            self.side_chain_handles.push((handle, running));
        }
    }

    /// Start all the interfaces by trying to connect them to the external device / endpoint
    /// 
    /// Notes: 
    /// 
    /// The scheduler must be mutable because running the connect() routine of an interface modifies that interface.
    pub fn start_all_interfaces(&mut self) {
        for interf in &mut self.interfaces {
            if let Err(e) = interf.connect() {
                eprintln!("Interface connection failed: {e}");
            }
        }
    }

    /// Stop all the side chains
    /// 
    /// Notes:
    /// 
    /// The scheduler must be mutable because each chain are returned by their dedicated thread 
    /// 
    /// And we needs to store them back to the list of side chain before restart dedicated threads again  
    pub fn stop_all_side_chains(&mut self) {
        for (_, running) in &self.side_chain_handles {
            running.store(false, Ordering::Relaxed);
        }
        for (handle, _) in self.side_chain_handles.drain(..) {
            if let Ok(chain) = handle.join() {
                self.side_chains.push(chain);
            }
        }
    }

    /// Stop all the interfaces
    /// 
    /// Notes: 
    /// 
    /// Scheduler must be mutable because unning the disconnect() routine of an interface modifies that interface.
    pub fn stop_all_interfaces(&mut self) {
        for interf in &mut self.interfaces {
            interf.disconnect();
        }
    }
}
