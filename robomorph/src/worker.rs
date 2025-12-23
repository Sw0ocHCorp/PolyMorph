use core::time;
use std::{ any::Any, sync::{Arc, Mutex}, thread::{self, JoinHandle}, time::{Duration, Instant, SystemTime}};

use downcast_rs::{Downcast, impl_downcast};

use crate::{event_management::{Event, Observer}, worker};

pub trait Module : Downcast + Send + Sync {
    fn exec_main_task(&self);

}
impl_downcast!(Module);

/**  A Worker is the spaceship of a Module
 * It implement and perform the common behavior of all Worker Module (timed task execution in main / dedicated thread)
 * The specialization of the Worker comes with the Module using the worker
 * 
 * is_async is used to implement a worker  executing module task in dedicated thread
 * IF frequency is set to -1, there is no verification of the time elapsed before the module task is executed.
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
    trig_time: Mutex<Instant>,
    frequency: i64,
    task_thread: Arc<Mutex<Option<JoinHandle<()>>>>,
    name: String,
    is_thread_running: Arc<Mutex<bool>>,
    is_async: bool,
    prev_worker_name: Mutex<String>
}

impl Worker {

    pub fn new(module: Arc<dyn Module>, name: &str, freq: i64, is_async: bool) -> Arc<Self> {
        let mut worker= Arc::new(Self {
            module,
            next_worker_event: Arc::new(Mutex::new(Event::new_empty())),
            previous_worker_observer: Mutex::new(None),
            trig_time: Mutex::new(Instant::now()),
            frequency: freq,
            task_thread: Arc::new(Mutex::new(None)),
            name: name.to_string().clone(),
            is_thread_running: Arc::new(Mutex::new(false)),
            is_async: is_async,
            prev_worker_name: Mutex::new("".to_string())
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
    //Function to try running a task execution
    pub fn try_run(&self) -> bool{
        let mut is_executed= false;
        //IF a frequency is specified 
        if self.frequency > 0 {
            if let Ok(mut trig_time)= self.trig_time.try_lock() {
                let now= Instant::now();
                //IF the time to start executing the task is reached or exceeded
                if now >= *trig_time {
                    //Compute the next trig time
                    if let Some(new_trig_time) = trig_time.checked_add(Duration::from_millis((1000 / self.frequency) as u64)) {
                        is_executed= true;
                        //print!("NEW TRIG TIME => {} ", new_trig_time.duration_since(now).as_millis());
                        self.module.clone().exec_main_task();
                        if let Ok(next_worker_event)= self.next_worker_event.try_lock() {
                            next_worker_event.trig(());
                        }
                        *trig_time= new_trig_time;
                    }
                }
            }
        } 
        //ELSE, no frequency is set, try_run execute force the execution of the module main task
        else {
            is_executed= true;
            self.force_run();
        }
        return is_executed;
    }

    pub fn force_run(&self) {
        self.module.clone().exec_main_task();
    }

    pub fn run_in_dedicated_thread(&self) {
        if self.is_async {
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
                            if freq > 0 {
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
                                module.clone().exec_main_task();
                            }
                        } else {
                            println!("INFO: {} Dedicated Task Thread ENDED", name.clone());
                            break;
                        }
                    }
                }
            });
            if let Ok(mut worker_thread_guard)= self.task_thread.clone().try_lock() {
                if let Some(mut worker_thread)= worker_thread_guard.as_ref() &&
                        let Ok(tsk_thread)= task_thread {
                    worker_thread= &tsk_thread;
                }
            }
        } else {
            println!("ERROR: Unable to run {} in dedicated thread because worker is synchrone", self.name)
        }
    }

    pub fn stop_task_thread(&self) {
        if self.is_async {
            while true {
                if let Ok(mut is_running) = self.is_thread_running.clone().try_lock() {
                    *is_running= false;
                    println!("INFO: END Task Thread CMD sent for {}", self.name.clone());
                    break;
                }
            }
        } else {
            println!("ERROR: Unable to stop the {} thread because this worker is synchrone", self.name)
        }
    }

    pub fn set_next_worker(&self, worker_observer: Observer<()>) {
        if let Ok(mut worker_event) = self.next_worker_event.clone().try_lock() {
            //worker_observer.set_observing_status(true);
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

    pub fn get_module(&self) -> Arc<dyn Module> {
        return self.module.clone();
    }

    /*pub fn get_module(&self) -> &dyn Module {
        return self.get_module().clone().as_concrete();
    }*/
}

pub struct WorkerFactory {
    workers: Vec<Arc<Worker>>
}

impl WorkerFactory {
    pub fn new() -> Self {
        return Self {workers: Vec::new()};
    }

    pub fn new_with_workers(workers: Vec<Arc<Worker>>) -> Self {
        return Self{workers: workers};
    }

    pub fn register_worker(&mut self, module: Arc<dyn Module>, worker_name: &str, freq: i64, is_async: bool) {
        self.workers.push(Worker::new(module, worker_name, freq, is_async));
    }

    pub fn register_workers(&mut self, modules_properties: Vec<(Arc<dyn Module>, &str, i64, bool)>) {
        for (module, worker_name, freq, is_async) in modules_properties {
            self.workers.push(Worker::new(module, worker_name, freq, is_async));
        }
    }

    pub fn set_workers_link(&mut self, given_worker_name: &str, next_worker_name: &str) {
        let mut given_worker= None;
        for w in self.workers.clone() {
            if given_worker_name == w.name {
                given_worker= Some(w);
            }
            else if next_worker_name == w.name && let Some(mut prev_worker)= given_worker.clone() &&
                                                let Some(prev_worker_observer)= w.get_worker_observer() {
                prev_worker.set_next_worker(prev_worker_observer);
            }
        }
    }

    pub fn set_workers_links(&mut self, given_worker_name: &str, next_worker_names: Vec<String>) {
        let mut given_worker= None;
        for w in self.workers.clone() {
            if given_worker_name == w.name {
                given_worker= Some(w);
            }
            else if next_worker_names.contains(&w.name) &&  let Some(mut prev_worker)= given_worker.clone() &&
                                                        let Some(prev_worker_observer)= w.get_worker_observer() {
                prev_worker.set_next_worker(prev_worker_observer);
            }
        }
    }

    pub fn remove_worker(&mut self, name: &str) {
        self.workers.retain(|worker| !worker.name.contains(name));
    }

    pub fn remove_workers(&mut self, names: Vec<String>) {
        self.workers.retain(|worker| !names.contains(&worker.name));
    }

    pub fn start_worker(&self, worker_name: &str) {
        for w in self.workers.clone() {
            if w.name == worker_name  {
                if w.is_async {
                    w.run_in_dedicated_thread();
                } else {
                    w.try_run();
                }
            }
        }
    }

    pub fn start_workers(&self, worker_names: Vec<String>) {
        for w in self.workers.clone() {
            if worker_names.contains(&w.name) { 
                if w.is_async {
                    w.run_in_dedicated_thread();
                } else {
                    w.try_run();
                }
            }
        }
    }

    pub fn start_all_async_workers(&self) {
        for w in self.workers.clone() {
            if w.is_async {
                w.run_in_dedicated_thread();
            }
        }
    }

    pub fn stop_async_worker(&self, worker_name: &str) {
        for w in self.workers.clone() {
            if w.name == worker_name  {
                w.stop_task_thread();
            }
        }
    }

    pub fn stop_async_workers(&self, worker_names: Vec<String>) {
        for w in self.workers.clone() {
            if worker_names.contains(&w.name) && w.is_async {
                w.stop_task_thread();
            }
        }
    }

    pub fn stop_all_async_workers(&self) {
        for worker in self.workers.clone() {
            if worker.is_async {
                worker.stop_task_thread();
            }
        }
    }

}