use core::any::Any;
use embassy_stm32::timer::low_level::OutOfRangeError;
use embassy_time::{Duration, Instant};

use crate::embed_core::{event_management::{EmbeddedEventPublisher, EmbeddedObserver}, utils::PolyError};

pub trait EmbeddedModule {
    fn exec_module_task(&mut self);
}

pub struct EmbeddedWorker<'a, const N: usize> {
    next_module_task_exec: Instant,
    module: &'a mut dyn EmbeddedModule,
    next_worker_publisher: EmbeddedEventPublisher<'a, (), N>,
    frequency: u64,
    is_alive: bool
}

impl<'a, const N: usize> EmbeddedObserver<()> for EmbeddedWorker<'a, N> {
    fn exec_answer(&mut self, data: &()) {
        self.try_run();
    }

    fn is_alive(&self) -> bool {
        return self.is_alive;
    }

    fn set_alive(&mut self, status: bool) {
        self.is_alive= status;
    }
}

impl<'a, const N: usize> EmbeddedWorker<'a, N> {
    pub fn new(modl: &'a mut dyn EmbeddedModule, freq: u64) -> Self {
        return Self {is_alive: true, next_module_task_exec: Instant::now(), 
            module: modl, next_worker_publisher: EmbeddedEventPublisher::<(), N>::new_empty() , frequency: freq }
    }

    pub fn try_run(&mut self) -> Instant {
        let elapsed= Instant::now().duration_since(self.next_module_task_exec).as_micros();
        let period= Duration::from_millis(1000 / self.frequency).as_micros();
        if self.frequency == 0 {
            self.module.exec_module_task();
            self.next_worker_publisher.publish(&());
        }else if elapsed >= period {
            self.module.exec_module_task();
            self.next_worker_publisher.publish(&());
            self.next_module_task_exec += Duration::from_micros(Duration::from_millis(1000 / self.frequency).as_micros());
            return self.next_module_task_exec;
        }
        return Instant::now();
    }

    pub fn attach_next_worker(&mut self, worker_observer: &'a mut dyn EmbeddedObserver<()>) -> Result<usize, PolyError> {
        return self.next_worker_publisher.attach_observer(worker_observer);
    }
}