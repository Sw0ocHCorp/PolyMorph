use crate::worker::Module;

pub mod event_management;
pub mod worker;
pub mod communication;
pub mod messages;
pub mod lidar;
pub mod utils;

#[cfg(test)]
mod TestUtilsFunctions {
    use crate::utils;

 
    #[test]
    fn it_works() {
        let tests = [
            0.0,
            std::f32::consts::PI / 2.0,
            std::f32::consts::PI,
            3.5 * std::f32::consts::PI,
            -1.0 * std::f32::consts::PI,
            -2.7 * std::f32::consts::PI,
        ];

        for &v in &tests {
            println!("{} -> {}", v, utils::modulo_pi(v));
        }
    }
}

#[cfg(test)]
mod TestWorkers {
    use std::{sync::{Arc, Mutex}, thread, time};

    use crate::worker::{Worker, WorkerFactory};

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
    fn test_simple_workers() {
        let worker1= Worker::new(Arc::new(DummyModule{value: 1}), "Dummy1", 200, false);
        let worker2= Worker::new(Arc::new(DummyModule{value: 2}), "Dummy2", 100, false);
        if let Some(worker_observer) = worker2.clone().get_worker_observer() {
            worker1.clone().set_next_worker(worker_observer);
        }
        while true {
            thread::sleep(time::Duration::from_micros(100));
            worker1.try_run();
        }
    }

    #[test]
    fn test_worker_factory() {
        let mut factory= WorkerFactory::new();
        let mut factory2= WorkerFactory::new();
        factory.register_workers(vec![
            (Arc::new(DummyModule{value: 100}), "Dummy1", 200, true),
            (Arc::new(DummyModule{value: 200}), "Dummy2", 100, true)
        ]);

        factory2.register_workers(vec![
            (Arc::new(DummyModule{value: 6667}), "AsyncDummy1", 2, false),
            (Arc::new(DummyModule{value: 776}), "AsyncDummy2", 1, false)
        ]);

        factory.set_workers_link("Dummy1", "Dummy2");

        factory2.set_workers_link("AsyncDummy1", "AsyncDummy2");

        //factory.start_first_workers();
        
        while true {
            thread::sleep(time::Duration::from_micros(100));thread::sleep(time::Duration::from_micros(100));
            factory2.start_worker("AsyncDummy1");
        }
        //factory
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
            udp1.add_data_observer(cmd_observer);
        }
        let udp_cl= udp2.clone();
        if let Some(observer)= udp1.get_cmd_observer() {
            udp2.add_data_observer(observer);
        }
        let worker1= Worker::new(udp1, "UDP1", 100, true);
        let worker2= Worker::new(udp2, "UDP2", 50, true);
        worker2.run_in_dedicated_thread();
        
        worker1.run_in_dedicated_thread();
        thread::sleep(time::Duration::from_secs(1));
        udp_cl.clone().publish_message("COUCOU".into());
        while true {
            
            
        }
    }
}

mod TestSerializationDeserialization {
    use std::collections::HashMap;

    use ordered_float::OrderedFloat;

    use crate::{lidar::LidarMeasurements, messages::{self, Translatable}};

    #[test]
    fn it_works() {
        let test_msgs:Vec<Box<dyn Translatable>>= vec![
                                                            Box::new(LidarMeasurements::new_from_measurements(HashMap::from([
                                                                                                                                (OrderedFloat::from(-90.0 as f32), 10.0 as f32), 
                                                                                                                                (OrderedFloat::from(0.0 as f32), 2.5 as f32), 
                                                                                                                                (OrderedFloat::from(90.0 as f32), 5.0 as f32),
                                                                                                                            ]))),
                                                            Box::new(LidarMeasurements::new_from_measurements(HashMap::from([
                                                                                                                                            (OrderedFloat::from(45.0 as f32), 10.0 as f32),
                                                                                                                                            (OrderedFloat::from(30.0 as f32), 50.0 as f32),
                                                                                                                                            (OrderedFloat::from(15.0 as f32), 40.0 as f32), 
                                                                                                                                            (OrderedFloat::from(0.0 as f32), 5.0 as f32), 
                                                                                                                                            (OrderedFloat::from(-15.0 as f32), 10.0 as f32),
                                                                                                                                            (OrderedFloat::from(-30.0 as f32), 50.0 as f32),
                                                                                                                                            (OrderedFloat::from(-45.0 as f32), 40.0 as f32),
                                                                                                                                ])))                 
                                                        ];
        let frame= messages::convert_to_frame(test_msgs);
        println!("Frame= {:?}",frame);
        let translatables= messages::parse_frame(frame);
        for translatable in translatables {
            println!("TRANSLATABLE: "); 
            match translatable.downcast_ref::<LidarMeasurements>() {
                Some(lidar_meas) => {
                    println!("LIDAR MEASUREMENTS: {:?}", lidar_meas.order_by_angle())
                },
                None => {

                },
            }
        }
        //println!("{}", translatables.len())

    }

    #[test]
    fn parse_real_frame() {
        let test_frame= vec![171, 205, 0, 212, 0, 5, 0, 10, 64, 26, 20, 131, 64, 114, 253, 239, 0, 11, 0, 48, 63, 
                                        150, 125, 144, 63, 108, 118, 208, 63, 47, 42, 0, 62, 233, 198, 96, 62, 117, 214, 128, 
                                        61, 11, 123, 0, 190, 37, 243, 64, 190, 178, 149, 96, 191, 6, 210, 96, 191, 31, 146, 224, 
                                        190, 179, 248, 128, 189, 128, 66, 0, 62, 122, 164, 192, 63, 18, 131, 128, 63, 107, 255, 
                                        112, 63, 165, 210, 168, 63, 217, 7, 136, 64, 65, 216, 100, 64, 48, 8, 132, 64, 37, 189, 
                                        244, 64, 32, 65, 248, 64, 30, 148, 236, 64, 32, 108, 252, 64, 38, 22, 128, 64, 48, 136, 
                                        192, 64, 67, 14, 240, 64, 1, 117, 200, 63, 213, 153, 72, 63, 188, 15, 120, 63, 172, 182, 
                                        80, 63, 164, 171, 80, 63, 162, 212, 112, 63, 166, 247, 224, 63, 177, 150, 112, 63, 196, 67, 
                                        64, 63, 227, 60, 48, 64, 17, 2, 36, 192, 43, 88, 202, 64, 69, 255, 32, 64, 39, 144, 76, 64, 
                                        11, 63, 140, 63, 225, 179, 16, 63, 176, 98, 136, 63, 130, 62, 56, 63, 113, 213, 96, 63, 220, 
                                        242, 184, 64, 38, 217, 192, 192, 43, 67, 178];
        let translatables= messages::parse_frame(test_frame);
        for translatable in translatables {
            println!("TRANSLATABLE: "); 
            match translatable.downcast_ref::<LidarMeasurements>() {
                Some(lidar_meas) => {
                    println!("LIDAR MEASUREMENTS: {:?}", lidar_meas.order_by_angle())
                },
                None => {

                },
            }
        }
    }
}