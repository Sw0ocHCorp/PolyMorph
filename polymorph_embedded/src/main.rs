#![no_std]
#![no_main]

use defmt_rtt as _; // Global logger
use panic_probe as _; // Panic handler

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    // Initialize the STM32 peripherals
    let _p = embassy_stm32::init(Default::default());
    let a= 1;
    let b= 5;
    let c= a+b;

    defmt::info!("Hello World!");

    // Your application code goes here
    loop {
        // ...
    }
}