use crate::worker::Module;

pub mod event_management;
pub mod worker;
pub mod communication;

#[cfg(test)]
mod TestLinkModules {
    use std::{sync::{Arc, Mutex}, thread, time};

    use crate::worker::Worker;

    use super::*;

    struct DummyModule {
        value: u32
    }

    impl Module for DummyModule {
        fn exec_main_task(&self) {
            println!("EXEC THE TASK OF {}", self.value)
        }
    }

    #[test]
    fn it_works() {
        let worker1= Worker::new(Arc::new(DummyModule{value: 1}), "Dummy1", 200);
        let worker2= Worker::new(Arc::new(DummyModule{value: 2}), "Dummy2", 100);
        if let Some(worker_observer) = worker2.clone().get_worker_observer() {
            worker1.clone().set_next_worker(worker_observer);
        }
        while true {
            thread::sleep(time::Duration::from_micros(100));
            worker1.try_run();
        }
    }
}

#[cfg(test)]
mod TestComInterface {
    use std::{thread, time};

    use crate::{communication::UDPChannel, worker::{self, Worker}};
 
    #[test]
    fn it_works() {
        let mut udp1= UDPChannel::new_async("127.0.0.1", 8080, "127.0.0.1", 9000);
        let mut udp2= UDPChannel::new_async("127.0.0.1", 9000, "127.0.0.1", 8080);
        if let Some(cmd_observer)= udp2.get_cmd_observer() {
            udp1.set_data_observer(cmd_observer);
        }
        let udp_cl= udp2.clone();
        if let Some(observer)= udp1.get_cmd_observer() {
            udp2.set_data_observer(observer);
        }
        let worker1= Worker::new(udp1, "UDP1", 100);
        let worker2= Worker::new(udp2, "UDP2", 50);
        worker2.run_in_dedicated_thread();
        
        worker1.run_in_dedicated_thread();
        thread::sleep(time::Duration::from_secs(1));
        udp_cl.clone().publish_cmd("COUCOU".into());
        while true {
            
            
        }
    }
}