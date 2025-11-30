use std::{sync::{Arc, Mutex}, thread::{self, JoinHandle}};

use crate::{com_channels, events_management, messages::Message};

/**
 * Trait used to execute a specific process task
 * Each module process must implement this trait to be used by a Worker
 */
pub trait Process {
    fn exec_task(self: Arc<Self>);
}
/** 
 * Class to link modules together
 * This object need to be a variable of the Module struct
 * Transmit data between them
 * Execute modules tasks in a specified sequence
 */
pub struct ModuleLinker {
    module_name: String,
    /**
     * Event to send data to other module
     */
    data_event: events_management::Event<Message>,
    /**
     * Observer to receive data from other module
     */
    data_observer: Option<events_management::Observer<Message>>,
    /**
     * Event to trigger next module in the chain
     */
    next_modules_event: events_management::Event<()>,
    /**
     * Observer to be notified when previous module in the chain has finished its task
     */
    next_module_observer: Option<events_management::Observer<()>>,
}
/** 
 * Module Linker
 * Used to link modules togethers -> Transmit & received data between them, Execute modules tasks in a specified sequence
 */
impl ModuleLinker {
    /** 
     * Constructor:
     * data_event and next_modules_event are initialized empty: The observers must be attached later
     * The data_observer and next_module_observer are initialized to None. This allow executing specific Module function in the observer callbacks
     */
    pub fn new(module_name: String) -> Self {
        return ModuleLinker {
            module_name,
            data_event: events_management::Event::new(vec![]),
            data_observer: None,
            next_modules_event: events_management::Event::new(vec![]),
            next_module_observer: None,
        };
    }
    /** 
     * Send data to the modules attached as observers to the data_event
     */
    pub fn send_message(&self, msg: Message) {
        self.data_event.trig(msg);
    }
    /**
     * Tell the next module in the chain to start its task
     */ 
    pub fn trig_next_module(&mut self) {
        self.next_modules_event.trig(());
    }
    /** 
     * Attach observer notifyed when the Module has finished its task
     */
    pub fn attach_next_module(&mut self, next_module_observer: events_management::Observer<()>) {
        self.next_modules_event.plug_observer(next_module_observer);
    }
    /**
     * Attach observer to send data to the given observer when data is available
     */
    pub fn attach_data_observer(&mut self, data_observer: events_management::Observer<Message>) {
        self.data_event.plug_observer(data_observer);
    }

    pub fn get_module_name(&self) -> String {
        return self.module_name.clone();
    }
    /**
     * Set the data observer called when data is received from an event(possibly an event in another module)
     */
    pub fn set_data_observer(&mut self, data_observer: events_management::Observer<Message>) {
        self.data_observer= Some(data_observer);
    }
    pub fn get_data_observer(&self) -> Option<events_management::Observer<Message>> {
        return self.data_observer.clone();
    }

    /**
     * Set the next module observer called when the previous module in the chain has finished its task
     */
    pub fn set_next_module_observer(&mut self, next_module_observer: events_management::Observer<()>) {
        self.next_module_observer= Some(next_module_observer);
    }
    pub fn get_next_module_observer(&self) -> Option<events_management::Observer<()>> {
        return self.next_module_observer.clone();
    }
}

/**
 * Worker to execute periodically the task of a Process in a dedicated thread
 */
pub struct Worker {
    pub name: String,
    pub task_frequency: u32,
    pub is_running: Mutex<bool>,
    pub worker_thread: Mutex<Option<JoinHandle<()>>>,
    /**
     * The object implementing the process trait to execute periodically its task
     * dyn Process + Send + Sync to allow using different struct implementing Process trait
     */
    pub target_process: Arc<dyn Process + Send + Sync>,
}

impl Worker {
    pub fn new(name: String, process: Arc<dyn Process + Send + Sync>, task_frequency: u32) -> Self {
        
        return Worker { name, task_frequency, 
                        is_running: Mutex::new(true), worker_thread: Mutex::new(None), 
                        target_process: process
                    };
    }

    /**
     * Create and start the worker thread to execute periodically the target process task
     */
    pub fn start_routine(self:Arc<Self>) {
        let this= self.clone();
        let running_thread= thread::Builder::new().name(self.clone().name.clone())
        .spawn(move || {
            let mut prev= std::time::Instant::now();
            //Loop as long as the is_running mutex is available and its value is true --> The thread is running
            while let Ok(is_running) = this.clone().is_running.try_lock() && *is_running == true {
                //millis loop to respect the task frequency between each task execution (
                // /!\ BE CAREFUL: If the task execution time is longer than the period, the time between each execution will be longer than expected. 
                //     The millis loop will be disturbed
                let now= std::time::Instant::now();
                let elapsed= now.duration_since(prev);
                if elapsed.as_millis() >= (1000 / this.clone().task_frequency) as u128 {
                    prev = now;
                    this.clone().target_process.clone().exec_task();
                }
            }
            
        });
        //Store the running thread in the worker_thread mutex to be able to stop and join it properly, later
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
            }
            Err(_) => {
                println!("Thread still lock");
            },
        }
    }

    /**
     * Stop the worker thread execution loop -> Soft stop
     */
    pub fn stop(self: Arc<Self>) {
        //Force thread stop by loop as long as the running mode mutex is not available
        loop {
            if let Ok(mut is_running) = self.clone().is_running.try_lock() && *is_running == true {
                *is_running= false;
                break;
            }
        }
        
    }

    /**
     * End the worker thread -> Hard stop: The thread is joined and deleted after join
     */
    pub fn end(self: Arc<Self>) {
        self.clone().stop();
        match self.clone().worker_thread.try_lock() {
            Ok(mut worker_thread) => {
                match worker_thread.take() {
                    Some(thread) => {
                        thread.join();
                        *worker_thread= None;
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

/**
 * Factory to manage execution of multiples Process via Workers
 * Each worker execute periodically the task of its target process in a dedicated thread
 */ 
pub struct WorkerFactory {
    workers: Vec<Arc<Worker>>,
}

impl Default for WorkerFactory {
    fn default() -> Self {
        return WorkerFactory {
            workers: vec![],
        };
    }
}

impl WorkerFactory {
    pub fn new(workers: Vec<Arc<Worker>>) -> Self {
        return Self { workers };
    }
    /**
     * Create and store a new worker from a given process
     */
    pub fn register_process(&mut self, process_name: String, process: Arc<dyn Process + Send + Sync>, task_frequency: u32) {
        self.workers.push(Arc::new(Worker::new(process_name, process, task_frequency)));
    }
    pub fn detach_all_process_workers(&mut self) {
        self.workers.clear();
    }
    pub fn detach_process_workers(&mut self, process_names: Vec<String>) {
        self.workers.retain(|worker| !process_names.contains(&worker.name));
    }

    pub fn start_all_process_workers(&mut self) {
        for worker in &self.workers {
            worker.clone().start_routine();
        }
    }
    pub fn start_process_workers(&mut self, process_names: Vec<String>) {
        for worker in &self.workers {
            if process_names.contains(&worker.name) {
                worker.clone().start_routine();
            }
        }
    }

    pub fn stop_all_process_workers(&mut self) {
        for worker in &self.workers {
            worker.clone().stop();
        }
    }
    pub fn stop_process_workers(&mut self, process_names: Vec<String>) {
        for worker in &self.workers {
            if process_names.contains(&worker.name) {
                worker.clone().stop();
            }
        }
        
    }

    pub fn end_all_process_workers(&mut self) {
        for worker in &self.workers {
            worker.clone().end();
        }
        self.detach_all_process_workers();
    }
    pub fn end_process_workers(&mut self, process_names: Vec<String>) {
        for worker in &self.workers {
            if process_names.contains(&worker.name) {
                worker.clone().end();
            }
        }
        self.detach_process_workers(process_names);
    }

    pub fn get_factory_size(&mut self) -> usize {
        return self.workers.len();
    }
}

