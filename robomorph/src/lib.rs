pub mod com_channels;
pub mod events_management;
pub mod process;
pub mod messages;

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use crate::{com_channels::{ChannelConfig, UDPChannel}, events_management::Observer, messages::Message, process::{ModuleLinker, WorkerFactory}};


    #[test]
    fn main() {
    let mut worker_factory= WorkerFactory::new();
    //Create the UDP Channels / Modules
    let udp=Arc::new(UDPChannel::new(ChannelConfig::new("127.0.0.1".to_string(),
                                                                            ModuleLinker::new("UDP1_WORKER".to_string())), 
                                                                            8090, "127.0.0.1".to_string(), 8080));
    let udp2=Arc::new(UDPChannel::new(ChannelConfig::new("127.0.0.1".to_string(),
                                                                            ModuleLinker::new("UDP2_WORKER".to_string())), 
                                                                            8080, "127.0.0.1".to_string(), 8090));
    let udp_cl= udp.clone();
    let udp2_cl= udp2.clone();
    //Configure the modules linkers and register the modules(because implement a process) in the worker factory
    if let Ok(mut linker)= udp.clone().chan_config.linker.try_lock() && let Ok(mut linker2) = udp2.clone().chan_config.linker.try_lock() {
        //Set the data observers to received the incoming data from the other UDP channel module
        let udp1_name= linker.get_module_name().clone();
        linker.set_data_observer(Observer::new(Arc::new(Box::new(move |x| {
            if let Message::Frame(msg) = x {
                if let Ok(data)= String::from_utf8(msg) {
                    println!("{}= Incoming data {} from {}:{}", udp1_name, data, udp_cl.clone().get_target_address(), udp_cl.clone().get_target_port());
                }
            }
        }))));
        let udp2_name= linker2.get_module_name().clone();
        linker2.set_data_observer(Observer::new(Arc::new(Box::new(move |x| {
            if let Message::Frame(msg) = x {
                if let Ok(data)= String::from_utf8(msg) {
                    println!("{}= Incoming data {} from {}:{}", udp2_name, data, udp2_cl.clone().get_target_address(), udp2_cl.clone().get_target_port());
                }
            }
        }))));
        //Attach the data observers to the other module linker to enable cross-module communication
        if let Some(udp_obs) = linker.get_data_observer() {
            linker2.attach_data_observer(udp_obs);
            
        }
        if let Some(udp2_obs) = linker2.get_data_observer() {
            linker.attach_data_observer(udp2_obs);
            
        }
        //Register the modules in the worker factory with their respective worker frequencies
        //The factorty can now start and manage the workers for these modules
        worker_factory.register_process(linker.get_module_name(), udp.clone(), 1);
        worker_factory.register_process(linker2.get_module_name(), udp2.clone(), 5);
    }

    if worker_factory.get_factory_size() > 0 {
        worker_factory.start_all_process_workers();
    }
    let mut prev= std::time::Instant::now();
    loop {
        let now= std::time::Instant::now();
        let elapsed= now.duration_since(prev);
        if elapsed.as_secs() >= 5 && worker_factory.get_factory_size() > 0 {
            worker_factory.end_all_process_workers();
            break;
        }
    }
}
}
