#![no_std]
#![no_main]

use defmt::println;
use defmt_rtt as _; use embassy_stm32::{Peri, gpio::{Level, Output, Speed}, peripherals::PA3};
// Global logger
use panic_probe as _;
use polymorph_embedded::embed_core::{event_management::EmbeddedObserver, execution::{EmbeddedModule, EmbeddedWorker}, polyvec::PolyVec}; // Panic handlers
use embassy_time::{Instant, Timer};

struct DummyModule<'a> {
    pin: Output<'a>,
}

impl<'a> EmbeddedModule for DummyModule<'a> {
    fn exec_module_task(&mut self) {
        let mut level= self.pin.get_output_level();
        self.pin.toggle();
        level= self.pin.get_output_level();
        let a= 1;
    }
}

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    //Initialize all the peripherals
    let peripherals = embassy_stm32::init(Default::default());
    //Create objects before passing them to the workers to satisfy the lifetime constraint
    let mut pin1= Output::new(peripherals.PA5, Level::Low, Speed::Low);
    let mut pin2= Output::new(peripherals.PA3, Level::High, Speed::Low);
    let mut mod1= DummyModule { pin: pin1 };
    let mut mod2= DummyModule { pin: pin2 };
    let mut worker1= EmbeddedWorker::<1>::new(&mut mod2, 1000);
    let mut worker2= EmbeddedWorker::<1>::new(&mut mod1, 10);
    worker1.attach_next_worker(&mut worker2);
    
    defmt::info!("Hello World!");

    // Your application code goes here
    loop {
        let next_exec_time= worker1.try_run();
        //let time_to_sleep= next_exec_time.duration_since(Instant::now()).as_millis();
        /*if time_to_sleep > 0 {
            Timer::after_micros(time_to_sleep);
        }*/
    }
}