use std::{sync::{Arc, Mutex, atomic::AtomicBool}, thread::{self, JoinHandle, Thread}};

use crate::{com_channels, events_management, flight_controller};

pub trait Module {
    fn trig_next_module(&mut self);
    
}

pub trait Work {
    fn start_task(self: Arc<Self>);
}

pub struct ProcessConfig {
    pub data_event: events_management::Event<com_channels::Message>,
    pub data_observer:events_management::Observer<com_channels::Message>,
    
}

pub struct ModuleConfig {
    pub process_config: ProcessConfig,
    pub next_modules_event: events_management::Event<()>,
}

pub struct Worker {
    pub process_config: ProcessConfig,
    pub worker_thread: Mutex<Option<JoinHandle<()>>>,
    //The object implementing the Work trait to be able to execute
    //start_task() in start_routine()
    //Dev needs to only implement the start_task() function 
    //  The Arc used to have acces to the worker object in different threads
    //  The dyn Work is used to store an object implementing the Work trait in this variable
    //  + Send + Sync ensure that the object can be safely shared and moved across threads
    pub worker_obj: Arc<dyn Work + Send + Sync>,
    pub is_running: Mutex<bool>,
    pub worker_name: String,
    pub task_frequency: u32,
}

impl Worker {
    //self -> Arc<Self> to allow using self.function() in a separated thread (the worker routine)
    fn start_routine(self: Arc<Self>) {
        let this= self.clone();
        let worker_obj = self.clone().worker_obj.clone();
        /*if let Ok(mut buffer) =self.clone().base_config.message_buffer.try_lock() {
            buffer.push_back(Message::Frame("CAVA".as_bytes().to_vec()));
        }*/
        //Set the thread in running mode
        if let Ok(mut is_running) = self.clone().is_running.try_lock() {
            *is_running= true;
        }
        //Thread used to maintain the Worker behavior
        let running_thread= thread::Builder::new()
            .name(self.worker_name.to_string())
            .spawn(move || {
                loop{
                    //Stop the thread if the thread is not running 
                    if let Ok(mut is_running) = this.clone().is_running.try_lock() {
                        if !*is_running {
                            println!("Soft Stop channel thread");
                            break;
                        }
                    } else {
                        println!("Is_Running already used");
                    }
                    worker_obj.clone().start_task();
                    
                }
        });
        
        //Store the channel thread in a class variable to be able to correctly end the thread
        match self.clone().worker_thread.try_lock() {
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
            if let Ok(mut is_running) = self.clone().is_running.try_lock() {
                *is_running= false;
                break;
            }
        }
        
    }
    
    fn end(self: Arc<Self>) {
        self.clone().stop();
        match self.clone().worker_thread.try_lock() {
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

