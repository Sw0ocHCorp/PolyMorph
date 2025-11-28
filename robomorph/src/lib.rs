pub mod com_channels;
pub mod events_management;
pub mod process;
pub mod messages;
pub mod utils;

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use crate::{com_channels::{ChannelConfig, UDPChannel}, events_management::Observer, messages::Message, process::{ModuleLinker, WorkerFactory}, utils::normalize_angle};


    #[test]
    fn sim_lidar() {
        let lidar_points= 200_f64;
        let lidar_fov= 180_f64;
        let offset= 180_f64;
        for i in 0..lidar_points as i32 {
            let angle = (-lidar_fov / 2.0).to_radians() + ((i as f64 / (lidar_points-1.0)) * lidar_fov).to_radians() + offset.to_radians();
            //let angle= (-(lidar_fov/2)as f64).to_radians() + (((i / lidar_points) as f64) * lidar_fov as f64).to_radians();
            print!( "{}\n", normalize_angle(angle as f32).to_degrees());
        }
    }

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
