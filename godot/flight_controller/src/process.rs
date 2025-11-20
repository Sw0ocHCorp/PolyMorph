use std::{sync::{Arc, Mutex, atomic::AtomicBool}, thread::{self, JoinHandle, Thread}};

use crate::{com_channels, events_management};


pub struct ProcessConfig {
    pub data_event: events_management::Event<com_channels::Message>,
    pub data_observer:events_management::Observer<com_channels::Message>,
    
}

pub struct ModuleConfig {
    pub process_config: ProcessConfig,
    pub next_modules_event: events_management::Event<()>,
}

pub struct WorkerConfig {
    pub process_config: ProcessConfig,
    pub worker_thread: Mutex<Option<JoinHandle<()>>>,
    pub is_running: Mutex<bool>,
}

pub trait Module {
    fn trig_next_module(&mut self);
    
}

pub trait Worker {
    //self -> Arc<Self> to allow using sel.function() in a separated thread (the worker routine)
    fn start(self: Arc<Self>);
    fn stop(self: Arc<Self>);
    fn end(self: Arc<Self>);
}

