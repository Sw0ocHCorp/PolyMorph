pub mod gui;

use std::sync::{Arc, Mutex};

use robomorph::{com_channels::{ChannelConfig, UDPChannel}, events_management::Observer, process::{ModuleLinker, WorkerFactory}, translator::Translator};

use crate::gui::Controller;

slint::include_modules!();

fn main() {
    /*let mut translat= Translator::new(vec![0xab, 0xcd], vec![ 
                Box::new(TranslatorLidarMessage {lidar_range_id:vec![0x00, 0x01], lidar_measurements_id: vec![0x00, 0x0a]}),
        ]);*/
    let mut factory= WorkerFactory::default();
    let mut linker= ModuleLinker::new("Controller Module".to_string());
    let udp=Arc::new(UDPChannel::new(ChannelConfig::new("127.0.0.1".to_string(),
                                                                                                        ModuleLinker::new("UDP1_WORKER".to_string())), 
                                                                                8090, "127.0.0.1".to_string(), 8080, 1));
    linker.set_data_observer(Observer::new_async());
    if let Ok(mut linkr)= udp.clone().chan_config.linker.try_lock() {
        if let Some(data_observer) = linker.get_data_observer() {
            linkr.attach_data_observer(data_observer);
        }
        factory.register_process(linkr.get_module_name(), udp.clone(), 1);
    }
    let mut soft= Controller::new(factory, linker);
    soft.run();
    
}
