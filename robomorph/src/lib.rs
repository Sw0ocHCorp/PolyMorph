pub mod communication;
pub mod lidar_management;
pub mod filtering;
pub mod positionning;
pub mod control;
pub mod core;
pub mod actuators;

#[cfg(test)]
mod test_utils_functions {
    use crate::core::utils;


 
    #[test]
    fn it_works() {
        let tests = [
            0.0,
            std::f64::consts::PI / 2.0,
            std::f64::consts::PI,
            3.5 * std::f64::consts::PI,
            -1.0 * std::f64::consts::PI,
            -2.7 * std::f64::consts::PI,
        ];

        for &v in &tests {
            println!("{} -> {}", v, utils::modulo_pi(v as f32));
        }
    }
}

#[cfg(test)]
mod test_workers {
    use std::{sync::Arc, thread, time};

    use crate::core::worker::{Module, Worker, WorkerFactory};

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
        loop {
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

        factory.start_all_async_workers();
        
        loop {
            //thread::sleep(time::Duration::from_micros(100));thread::sleep(time::Duration::from_micros(100));
            //factory2.start_worker("AsyncDummy1");
        }
        //factory
    }
    
}

#[cfg(test)]
mod test_com_interface {
    use std::{thread, time};

    use crate::{communication::UDPChannel, core::worker::Worker};
 
    #[test]
    fn it_works() {
        let udp1= UDPChannel::new_async("127.0.0.1", 8080, "127.0.0.1", 9000);
        let udp2= UDPChannel::new_async("127.0.0.1", 9000, "127.0.0.1", 8080);
        if let Some(cmd_observer)= udp2.get_cmd_observer() {
            udp1.add_frame_observer(cmd_observer);
        }
        let udp_cl= udp2.clone();
        if let Some(observer)= udp1.get_cmd_observer() {
            udp2.add_frame_observer(observer);
        }
        let worker1= Worker::new(udp1, "UDP1", 100, true);
        let worker2= Worker::new(udp2, "UDP2", 50, true);
        worker2.run_in_dedicated_thread();
        
        worker1.run_in_dedicated_thread();
        thread::sleep(time::Duration::from_secs(1));
        udp_cl.clone().publish_message("COUCOU".into());
        loop {
            
            
        }
    }
}

mod test_serialization_deserialization {
    use crate::{core::messages::{self, Translatable}, lidar_management::measurements::LidarMeasurements, positionning::pose::IMUData};

    



    

    #[test]
    fn it_works() {
        let test_msgs:Vec<Box<dyn Translatable>>= vec![
                                                            Box::new(LidarMeasurements::new_from_measurements(vec![-90.0, 0.0, 90.0], vec![10.0, 2.5, 5.0], true)),
                                                            Box::new(LidarMeasurements::new_from_measurements(vec![-45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0], vec![10.0, 50.0, 40.0, 5.0, 10.0, 50.0, 40.0], true))             
                                                        ];
        let frame= messages::convert_to_frame(test_msgs);
        println!("Frame= {:?}",frame);
        let translatables= messages::parse_frame(frame);
        for translatable in translatables {
            println!("TRANSLATABLE: "); 
            match translatable.downcast_ref::<LidarMeasurements>() {
                Some(lidar_meas) => {
                    println!("LIDAR MEASUREMENTS: {:?}", lidar_meas)
                },
                None => {

                },
            }
        }
        //println!("{}", translatables.len())

    }

    #[test]
    fn parse_real_frame() {
        let test= f32::to_be_bytes(-1.5707963267948966);
        println!("Bytes of -PI/2: {:?}", test);
        let test2= f32::to_be_bytes(-90.0);
        println!("Bytes of -90.0: {:?}", test2);
        let test_frame= vec![171, 205, 3, 52, 0, 5, 0, 10, 192, 72, 142, 135, 60, 129, 83, 128, 0, 11, 0, 200, 64, 142, 201, 183, 64, 142, 210, 212,
         64, 142, 229, 16, 64, 143, 0, 115, 64, 143, 37, 2, 64, 143, 82, 205, 64, 143, 137, 223, 65, 88, 58, 35, 65, 88, 40, 83, 65, 88, 36, 35, 65, 88, 45,
          143, 65, 88, 68, 153, 65, 88, 100, 83, 65, 88, 65, 209, 65, 88, 45, 37, 65, 88, 38, 25, 65, 88, 44, 168, 65, 88, 64, 215, 65, 88, 98, 169, 65, 88,
           74, 113, 65, 88, 50, 202, 65, 88, 40, 197, 65, 88, 44, 92, 65, 88, 61, 144, 65, 88, 92, 102, 65, 88, 83, 154, 65, 88, 55, 214, 65, 88, 41, 178, 65,
            88, 41, 45, 65, 88, 54, 68, 65, 88, 80, 251, 65, 88, 88, 24, 65, 88, 58, 132, 65, 88, 42, 148, 65, 88, 40, 65, 65, 88, 51, 137, 65, 88, 76, 117, 65,
             88, 98, 193, 65, 88, 65, 74, 65, 88, 45, 132, 65, 88, 39, 92, 65, 88, 46, 209, 65, 88, 67, 229, 65, 88, 102, 159, 65, 88, 68, 26, 65, 88, 44, 174,
              65, 88, 34, 224, 65, 88, 38, 176, 65, 88, 56, 29, 65, 88, 87, 46, 65, 88, 69, 190, 65, 88, 45, 127, 65, 88, 34, 225, 65, 88, 37, 225, 65, 88, 54,
               123, 65, 88, 84, 185, 65, 88, 83, 177, 65, 88, 55, 204, 65, 88, 41, 135, 65, 88, 40, 224, 65, 88, 53, 215, 65, 88, 80, 108, 65, 88, 93, 191, 65,
                88, 62, 3, 65, 48, 70, 204, 65, 45, 240, 20, 65, 45, 39, 29, 65, 45, 101, 26, 65, 46, 202, 236, 65, 51, 91, 68, 65, 88, 45, 198, 65, 88, 39, 126,
                 65, 88, 46, 210, 65, 88, 67, 196, 65, 88, 102, 92, 65, 88, 75, 169, 65, 88, 51, 166, 65, 88, 41, 65, 65, 88, 44, 119, 65, 88, 61, 76, 65, 88, 91,
                  197, 65, 88, 80, 70, 65, 88, 53, 70, 65, 88, 39, 233, 65, 88, 40, 39, 65, 88, 54, 5, 65, 88, 81, 129, 65, 88, 84, 236, 65, 88, 55, 17, 65, 88,
                   38, 215, 65, 88, 36, 56, 65, 88, 47, 54, 65, 88, 71, 211, 65, 88, 91, 241, 65, 88, 59, 61, 65, 88, 40, 44, 65, 88, 34, 181, 65, 88, 42, 222, 65,
                    88, 64, 162, 65, 88, 100, 15, 65, 88, 100, 13, 65, 88, 64, 152, 65, 88, 42, 210, 65, 88, 34, 170, 65, 88, 40, 33, 65, 88, 59, 50, 65, 88, 91,
                     233, 65, 88, 72, 28, 65, 88, 47, 128, 65, 88, 36, 128, 64, 30, 56, 193, 64, 22, 84, 9, 64, 18, 35, 59, 64, 15, 46, 192, 64, 12, 235, 45, 64,
                      11, 29, 221, 64, 9, 167, 161, 64, 8, 117, 173, 64, 7, 123, 182, 64, 6, 177, 61, 64, 6, 16, 32, 64, 5, 147, 227, 64, 5, 57, 22, 64, 4, 253, 39,
                       64, 4, 222, 22, 64, 4, 218, 104, 64, 4, 240, 250, 64, 5, 32, 251, 64, 5, 105, 231, 64, 5, 203, 106, 64, 6, 69, 103, 64, 6, 215, 254, 64, 7,
                        131, 117, 64, 8, 72, 66, 64, 9, 39, 15, 64, 10, 32, 177, 64, 11, 54, 60, 64, 12, 104, 245, 64, 13, 186, 98, 64, 15, 44, 79, 64, 16, 192, 223,
                         64, 18, 122, 144, 64, 20, 92, 83, 64, 22, 105, 161, 64, 24, 166, 150, 64, 27, 24, 23, 64, 29, 196, 6, 64, 32, 177, 146, 64, 35, 233, 133, 64,
                          39, 118, 229, 64, 43, 103, 185, 64, 47, 206, 80, 64, 52, 195, 65, 64, 58, 104, 232, 64, 64, 241, 221, 64, 72, 174, 6, 64, 82, 42, 110, 64, 94,
                           115, 72, 64, 109, 18, 98, 64, 125, 210, 7, 64, 136, 152, 126, 64, 147, 236, 109, 64, 161, 88, 219, 64, 177, 128, 237, 64, 197, 81, 139, 64,
                            222, 48, 146, 65, 88, 42, 77, 65, 88, 58, 63, 65, 88, 87, 211, 65, 88, 80, 191, 65, 88, 54, 7, 65, 88, 40, 241, 65, 88, 41, 120, 65, 88, 55,
                             155, 65, 88, 83, 94, 65, 88, 92, 205, 65, 88, 61, 245, 65, 88, 44, 194, 65, 88, 41, 45, 65, 88, 51, 48, 65, 88, 74, 218, 65, 88, 98, 204, 65,
                              88, 65, 20, 65, 88, 44, 231, 65, 88, 38, 88, 65, 88, 45, 100, 65, 88, 66, 15, 65, 88, 100, 96, 65, 88, 68, 76, 65, 88, 45, 65, 65, 88, 35,
                               215, 65, 88, 40, 9, 65, 88, 57, 213, 64, 143, 137, 223, 64, 143, 82, 205, 64, 143, 37, 2, 64, 143, 0, 115, 64, 142, 229, 16, 64, 142, 210, 212, 64, 142, 201, 183];
        let translatables= messages::parse_frame(test_frame);
        let mut imu_measurements= IMUData { accel: [0.0, -0.0, 0.5385452508926392], gyro: [0.0, -0.0, 0.0], magnetic_field: [0.3491528332233429, -0.0098767951130867, -0.9370137453079224], elapsed_time: 0.01 };
        println!("{:?}", imu_measurements.to_bytes());
        let mut imu= IMUData::new();
        let _= imu.fill_from_bytes(imu_measurements.to_bytes());
        println!("{:?}", imu);
        for translatable in translatables {
            println!("TRANSLATABLE: "); 
            match translatable.downcast_ref::<LidarMeasurements>() {
                Some(lidar_meas) => {
                    println!("LIDAR MEASUREMENTS: {:?}", lidar_meas)
                },
                None => {

                },
            }
        }
    }
}

mod test_logs {
    use crate::core::file_logger::FileLogger;

    

 
    #[test]
    fn it_works() {
        let logger= FileLogger::new("Logs".to_string());
        logger.add_logs("COUCOU".to_string());
    }
}