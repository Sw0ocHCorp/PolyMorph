use core::time;
use std::{ sync::{Arc, Mutex}, thread::{self, JoinHandle}, time::SystemTime};

use crate::{event_management::{Event, Observer}, worker};

pub trait Module : Send + Sync {
    fn exec_main_task(&self);
}

/**  A Worker is the spaceship of a Module
 * It implement and perform the common behavior of all Worker Module (timed task execution in main / dedicated thread)
 * The specialization of the Worker comes with the Module using the worker
 * 
 * The Worker needs to be an Arc<> to be able to call its functions in the observers like the previous_worker_observer
 * The mutable fields are wrapped in a Mutex<> or Arc<Mutex<>> to be mutable even with an Arc<> struct
 *  
 * With that, The only things to do, to create a Worker for a specific module is to create the given module struct (struct implementing the trait Module)
 * And pass it as argument in the Worker constructor
 * */
pub struct Worker {
    module: Arc<dyn Module>,
    next_worker_event: Arc<Mutex<Event<()>>>,
    previous_worker_observer: Mutex<Option<Observer<()>>>,
    start_time: Mutex<SystemTime>,
    frequency: u32,
    task_thread: Option<JoinHandle<()>>,
    name: String,
    is_thread_running: Arc<Mutex<bool>>
}

impl Worker {

    pub fn new(module: Arc<dyn Module>, name: &str, freq: u32,) -> Arc<Self> {
        let mut worker= Arc::new(Self {
            module,
            next_worker_event: Arc::new(Mutex::new(Event::new_empty())),
            previous_worker_observer: Mutex::new(None),
            start_time: Mutex::new(SystemTime::now()),
            frequency: freq,
            task_thread: None,
            name: name.to_string().clone(),
            is_thread_running: Arc::new(Mutex::new(false))
        });
        let mut worker_weak= Arc::downgrade(&worker);
        let obs = Observer::new(Arc::new(Mutex::new(Box::new(move |x: ()| {
            // Step 3: Upgrade Weak to Strong only when the callback runs
            if let Some(mut w) = worker_weak.upgrade() {
                w.try_run();
            }
        }))));

        if let Ok(mut obs_guard) = worker.previous_worker_observer.try_lock() {
            *obs_guard = Some(obs);
        }
        return worker;
    }

    pub fn try_run(&self) {
        let mut task_executed= 0;
        if let Ok(mut start)= self.start_time.try_lock() {
            match start.elapsed() {
                Ok(elapsed_time) => {
                    if elapsed_time.as_millis() >= (1000 / self.frequency) as u128 {
                        print!("Elapsed Time= {} =>", elapsed_time.as_millis());
                        *start= SystemTime::now();
                        self.module.clone().exec_main_task();
                        if let Ok(next_worker_event)= self.next_worker_event.try_lock() {
                            next_worker_event.trig(());
                        }
                    }
                },
                Err(_) => {
                },
            }
        }
    }

    pub fn run_in_dedicated_thread(&self) {
        let name= self.name.clone();
        if let Ok(mut is_running) = self.is_thread_running.try_lock() {
            *is_running= true;
        }
        let mut starting_time= SystemTime::now();
        let module= self.module.clone();
        let next_worker_event= self.next_worker_event.clone();
        let freq= self.frequency;
        let is_thread_running= self.is_thread_running.clone();
        let task_thread= thread::Builder::new()
                                            .name(name.clone())
                                            .spawn(move || {
            while true {
                if let Ok(is_running) = is_thread_running.clone().try_lock() {
                    if *is_running == true {
                        match starting_time.elapsed() {
                            Ok(elapsed_time) => {
                                if elapsed_time.as_millis() >= (1000 / freq) as u128 {
                                    starting_time= SystemTime::now();
                                    module.clone().exec_main_task();
                                    if let Ok(worker_publisher)= next_worker_event.try_lock() {
                                        worker_publisher.trig(());
                                    }
                                }
                            },
                            Err(_) => {

                            },
                        }
                    } else {
                        println!("INFO: {} Dedicated Task Thread ENDED", name.clone());
                        break;
                    }
                }
            }
        });
    }

    pub fn stop_task_thread(&self) {
        while true {
            if let Ok(mut is_running) = self.is_thread_running.clone().try_lock() {
                *is_running= false;
                println!("INFO: END Task Thread CMD sent for {}", self.name.clone());
                break;
            }
        }
    }

    pub fn set_next_worker(&self, worker_observer: Observer<()>) {
        if let Ok(mut worker_event) = self.next_worker_event.clone().try_lock() {
                worker_event.plug_observer(worker_observer);
            }
    }

    pub fn get_worker_observer(&self) -> Option<Observer<()>> {
        match self.previous_worker_observer.try_lock() {
            Ok(worker_observer) => {
                match worker_observer.as_ref() {
                    Some(observer) => { return Some(observer.clone()); },
                    None => { return None; },
                }
            },
            Err(_) => {
                println!("ERROR: Observer still used");
                return None;
            },
        }
    }
}