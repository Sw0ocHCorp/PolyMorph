#![no_std]
#![no_main]

use core::cell::RefCell;

use defmt::{info, println};
use defmt_rtt as _; use embassy_stm32::{Peri, bind_interrupts, gpio::{Level, Output, Speed}, mode::{Async, Blocking, Mode}, pac::Interrupt::{UART4, UART5}, usart::{Config, InterruptHandler, Uart}};
// Global logger
use panic_probe as _; // Panic handlers
use embassy_time::{Instant, Timer};
use embassy_stm32::dma;
use polymorph_embedded::embed_core::{event_data::EventData, execution::{EmbeddedModule, EmbeddedWorker}, orchestrator::Orchestrator, polyvec::PolyVec, utils::PolyError};

pub struct DummyModule<'a, const NOBS: usize> {
    pin: Output<'a>,
    alive: bool,
    observer_ids: PolyVec<u64, NOBS>
}

impl<'a, const NOBS: usize> EmbeddedModule<NOBS> for DummyModule<'a, NOBS> {
    fn exec_module_task(&mut self) -> Option<(EventData, PolyVec<u64, NOBS>)> {
        self.pin.toggle();
        return None;
        //return Some((EventData::TrigPin, self.observer_ids));
    }

    fn exec_callback(&mut self, data: &EventData) -> Option<(EventData, PolyVec<u64, NOBS>)> {
        self.pin.toggle();
        //return Some((EventData::TrigPin, self.observer_ids));
        return None;
    }

    fn set_observer_ids(&mut self, observers: PolyVec<u64, NOBS>) {
        self.observer_ids= observers;
    }

    fn get_observer_ids(&self) -> PolyVec<u64, NOBS> {
        return self.observer_ids;
    }

    fn set_alive_status(&mut self, status: bool) {
        self.alive= status;
    }
    
    fn is_alive(&self) -> bool {
        return self.alive;
    }
}

impl<'a, const NOBS: usize> DummyModule<'a, NOBS> {
    pub fn new(p: Output<'a>) -> Self {
        return Self { pin: p, alive: true, observer_ids: PolyVec::<u64, NOBS>::new_empty() };
    }
}

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    //Defines the hardware peripherals used for the system
    let peripherals = embassy_stm32::init(Default::default());
    let mut pin1= Output::new(peripherals.PA5, Level::Low, Speed::Low);
    let mut pin2= Output::new(peripherals.PA6, Level::High, Speed::Low);
    //Define the modules contained in the system
    let mut mod1 = DummyModule::<2>::new(pin1);
    let mut mod2 = DummyModule::<2>::new(pin2);
    //Link the modules between them
    mod1.set_observer_ids(PolyVec::<u64, 2>::from_array([2]));
    mod2.set_observer_ids(PolyVec::<u64, 2>::from_array([1]));
    //Defines the orchestrator that execute the modules tasks
    let mut orchestr= Orchestrator::<2, 2>::new();
    //Register all the modules used by the orchestrator
    orchestr.register(1, &mut mod1, 0);
    orchestr.register(2, &mut mod2, 0);
    //Set the execution order of the tasks modules
    orchestr.set_execution_graph(PolyVec::<u64, 2>::from_array([1, 2]));
    let mut exec_time= Instant::now();
    loop {
        if Instant::now().as_micros() > exec_time.as_micros() {
            if let Some(next_time)= orchestr.launch_execution_graph() {
                exec_time= next_time;
            }
            /*let remain_time= Instant::now() - exec_time;
            if remain_time.as_micros() > 0 {
                Timer::after(remain_time);
            }*/
        }
        //let next_exec_time= worker1.try_run();
    }
}