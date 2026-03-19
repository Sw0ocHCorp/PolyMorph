use core::cell::RefCell;

use embassy_time::{Duration, Instant};

use crate::embed_core::{event_data::EventData, polyvec::PolyVec};

pub trait EmbeddedModule<const NOBS: usize> {
    fn exec_module_task(&mut self) -> Option<(EventData, PolyVec<u64, NOBS>)>;

    fn exec_callback(&mut self, data: &EventData) -> Option<(EventData, PolyVec<u64, NOBS>)>;

    fn set_observer_ids(&mut self, observers: PolyVec<u64, NOBS>);

    fn set_alive_status(&mut self, status: bool);
    
    fn get_observer_ids(&self) -> PolyVec<u64, NOBS>;

    fn is_alive(&self) -> bool;

}

pub struct EmbeddedWorker<'a, const NOBS: usize> {
    next_module_task_exec: Instant,
    module: &'a mut dyn EmbeddedModule<NOBS>,
    frequency: u64,
    is_alive: bool
}

impl<'a, const NOBS: usize> EmbeddedWorker<'a, NOBS> {
    pub fn new(modl: &'a mut dyn EmbeddedModule<NOBS>, freq: u64) -> Self {
        return Self {is_alive: true, next_module_task_exec: Instant::now(), 
            module: modl, frequency: freq }
    }

    pub fn try_run(&mut self) -> (Instant, Option<(EventData, PolyVec<u64, NOBS>)>) {
        let elapsed= Instant::now().duration_since(self.next_module_task_exec).as_micros();
        let mut result_data= None;
        if self.frequency == 0 {
            result_data= self.module.exec_module_task();
            return (Instant::now(), result_data);
        } else { 
            let period= Duration::from_millis(1000 / self.frequency).as_micros();
            if elapsed >= period {
                result_data= self.module.exec_module_task();
                self.next_module_task_exec += Duration::from_micros(Duration::from_millis(1000 / self.frequency).as_micros());
            }
            return (self.next_module_task_exec, result_data);
        }
        
    }

    pub fn launch_module_callback(&mut self, data: &EventData) -> Option<(EventData, PolyVec<u64, NOBS>)> {
        return self.module.exec_callback(data);
    }
}