use std::sync::{Arc, Mutex};
use ndarray::Array2;
use robomorph::{communication::UDPChannel, core::{event_management::{Event, Observer}, messages, worker::{Module, WorkerFactory}}, filtering::mahony_filter::MahonyFilter, lidar_management::{lidar_perception_manager::LidarPerceptionManager, measurements::LidarMeasurements, segmentation_algorithms::ClassicSolver}, positionning::pose::{IMUData, Pose}};



pub struct BenchmarkFiltering {
    imu_data_observer: Mutex<Option<Observer<IMUData>>>,
    data_filtered_event: Mutex<Event<Pose>>,
    filter: Mutex<MahonyFilter>,
}

impl Module for BenchmarkFiltering {
    fn exec_main_task(&self) {
        
    }
}

impl BenchmarkFiltering {
    pub fn new(p: f32, i: f32, d: f32, max_integral_error: f32, dt: f32 ) -> Arc<Self> {
        /*let filter= GenericEKF::new(
                                                Array2::zeros((3, 1)), Array2::eye(3), Array2::eye(3), Array2::eye(3), Array2::eye(3), 
                                                system_dynamics_fun, system_jacob_fun, dt);   
        let this = Arc::new(Self { imu_data_observer: Mutex::new(None), data_filtered_event: Mutex::new(Event::new_empty()), filter: Mutex::new(filter) });
        let mut filter_manager= this.clone();
        let obs= Observer::new(Arc::new(Mutex::new(move |imu_data: IMUData| {
            if let Ok(mut filter)= filter_manager.clone().filter.try_lock() {
                if let Ok(input_array)= Array2::from_shape_vec((3,1), imu_data.gyro.to_vec()) && 
                        let Ok(accel_measurement)= Array2::from_shape_vec((3,1), imu_data.accel.to_vec()) &&
                        let Ok(magnetic_measurement)= Array2::from_shape_vec((3,1), imu_data.magnetic_field.to_vec()) {
                    let pred_state= filter.predict(input_array);
                    let roll_pitch_state= filter.update(pred_state, accel_measurement);
                    let yaw_state= filter.update(roll_pitch_state, magnetic_measurement);
                    //let test= filter.compute_system_state(input_array, measurement_array);
                    if let Ok(data_event)= filter_manager.clone().data_filtered_event.try_lock() {
                        let pose= Pose::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
                        data_event.trig(pose);
                    }
                }
            }
        })));
        filter_manager= this.clone();
        if let Ok(mut obs_guard) = filter_manager.clone().imu_data_observer.try_lock() {
            *obs_guard= Some(obs);
        }
        return this;*/
        let filter= MahonyFilter::new(p, i, d, max_integral_error, dt, 0.5f32.to_radians(), 1.0);
        let this = Arc::new(Self { imu_data_observer: Mutex::new(None), data_filtered_event: Mutex::new(Event::new_empty()), filter: Mutex::new(filter) });
        let mut filter_manager= this.clone();
        let obs= Observer::new(Arc::new(Mutex::new(move |imu_data: IMUData| {
            if let Ok(mut filter)= filter_manager.clone().filter.try_lock() {
                let estimated_orientation= filter.estimate_orientation(imu_data);
                //let test= filter.compute_system_state(input_array, measurement_array);
                if let Ok(data_event)= filter_manager.clone().data_filtered_event.try_lock() {
                    let pose= Pose::new([0.0, 0.0, 0.0], 
                                                        [estimated_orientation[0], estimated_orientation[1], estimated_orientation[2]], 
                                                        [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
                    data_event.trig(pose);
                }
            }
        })));
        filter_manager= this.clone();
        if let Ok(mut obs_guard) = filter_manager.clone().imu_data_observer.try_lock() {
            *obs_guard= Some(obs);
        }
        return this;
        //let this= Self {}
    }

    pub fn get_imu_measurements_observer(&self) -> Option<Observer<IMUData>> {
        if let Ok(observer) = self.imu_data_observer.try_lock() {
            if let Some(obs) = observer.as_ref() {
                return Some(obs.clone());
            } else {
                return None;
            } 
        }
        else {
            return None;
        }
    }
}

pub struct MailBox {
    //Receive the GODOT frame from the UDPChannel
    frame_observer: Mutex<Option<Observer<Vec<u8>>>>,
    //Sensors Measurement frame: Mailbox => UDPChannel (to be sent to the Python VisuSoft)
    mailbox_event: Mutex<Event<Vec<u8>>>,
    //LiDAR measurements GODOT frame: Mailbox => LidarPerceptionManager
    lidar_data_event: Mutex<Event<LidarMeasurements>>,
    pose_filtering_event: Mutex<Event<IMUData>>,
    //Receive the LiDAR measurements from LidarPerceptionManager
    processed_lidar_data_obs: Mutex<Option<Observer<LidarMeasurements>>>,
    pose_filtered_observer: Mutex<Option<Observer<Pose>>>
}

impl Module for MailBox {
    fn exec_main_task(&self) {
        
    }
}

impl MailBox {
    pub fn new() -> Arc<Self> {
        let mut mailbox= Arc::new(Self{frame_observer: Mutex::new(None), mailbox_event: Mutex::new(Event::new_empty()), 
                                                            lidar_data_event: Mutex::new(Event::new_empty()), processed_lidar_data_obs: Mutex::new(None), 
                                                            pose_filtering_event: Mutex::new(Event::new_empty()), pose_filtered_observer: Mutex::new(None)});
        let mut mailbox_cl= mailbox.clone();
        //Callback for GODOT UDP frame reception
        let obs= Observer::new(Arc::new(Mutex::new(move |frame: Vec<u8>| {
            //Sending the frame to LidarPerceptionManager 
            mailbox_cl.send_godot_frame(frame);
        })));
        if let Ok(mut obs_guard) = mailbox.frame_observer.try_lock() {
            *obs_guard= Some(obs);
        }
        mailbox_cl= mailbox.clone();
        //Callback for processed LiDAR measurements reception
        let lidar_obs= Observer::new(Arc::new(Mutex::new(move |lidar_meas: LidarMeasurements| {
            //Sending the processed LiDAR measurement frame to the UDPChannel 
            let frame= messages::convert_to_frame(vec![Box::new(lidar_meas)]);
            if let Ok(frame_ev_guard) = mailbox_cl.mailbox_event.try_lock() {
                frame_ev_guard.trig(frame);
            }
        })));
        if let Ok(mut lidar_observer)= mailbox.processed_lidar_data_obs.try_lock() {
            *lidar_observer= Some(lidar_obs);
        }
        mailbox_cl= mailbox.clone();
        //Callback for filtered Pose reception
        let pose_obs= Observer::new(Arc::new(Mutex::new(move |pose: Pose| {
            //Sending the processed LiDAR measurement frame to the UDPChannel 
            let frame= messages::convert_to_frame(vec![Box::new(pose)]);
            if let Ok(frame_ev_guard) = mailbox_cl.mailbox_event.try_lock() {
                frame_ev_guard.trig(frame);
            }
        })));
        if let Ok(mut pose_observer)= mailbox.pose_filtered_observer.try_lock() {
            *pose_observer= Some(pose_obs);
        }
        return mailbox;

    }

    pub fn send_godot_frame(&self, frame: Vec<u8>) {
        let translatables= messages::parse_frame(frame.clone());
        let mut i= 0;
        
        for translatable in translatables {
            let mut is_sent= false;
            //println!("Translatable {}", i);
            i+= 1;
            match translatable.downcast_ref::<LidarMeasurements>() {
                Some(lidar_meas) => {
                    //println!("LiDAR:\n   measurements len= {}\n   lidar objects len= {}", lidar_meas.lidar_pts.len(), lidar_meas.lidar_objects.len());
                    if let Ok(lidar_event) = self.lidar_data_event.try_lock() {
                       lidar_event.trig(lidar_meas.clone());
                       is_sent= true;
                    }
                },
                None => { 

                },
            }
            if is_sent == false {
                match translatable.downcast_ref::<IMUData>() {
                    Some(imu_data) => {
                        //println!("IMU:\n   {:?}", imu_data);
                        if let Ok(pose_filtering_event)= self.pose_filtering_event.try_lock() {
                            pose_filtering_event.trig(imu_data.clone());
                            //mailbox_event.trig(frame.clone());
                            is_sent= true;
                        }
                    },
                    None => { 

                    },
                }
            }

        }
    }
    pub fn get_lidar_measurements_observer(&self) -> Option<Observer<LidarMeasurements>> {
        if let Ok(observer) = self.processed_lidar_data_obs.try_lock() {
            if let Some(obs) = observer.as_ref() {
                return Some(obs.clone());
            } else {
                return None;
            } 
        }
        else {
            return None;
        }
    }
    pub fn get_pose_filtered_observer(&self) -> Option<Observer<Pose>> {
        if let Ok(observer) = self.pose_filtered_observer.try_lock() {
            if let Some(obs) = observer.as_ref() {
                return Some(obs.clone());
            } else {
                return None;
            } 
        }
        else {
            return None;
        }
    }
}

fn main() {
    //System modules creation
    let mut factory= WorkerFactory::new();
    let mailbox= MailBox::new();
    let udp= UDPChannel::new_async("127.0.0.1", 8090, "127.0.0.1", 9000);
    let pose_filter= BenchmarkFiltering::new(20.0, 0.01, 3.25, 10.0, 1.0/50.0);
    let classic_solver= Arc::new(ClassicSolver::new( 0.10, 2,
        Box::new(|p1, p2|{
            let loc1= p1.get_location();
            let loc2= p2.get_location();
            let dist= ((loc2.0 - loc1.0).powf(2.0) + (loc2.1 - loc1.1).powf(2.0)).sqrt();
            return dist;
        })
    ));
    let lidar_manager= LidarPerceptionManager::new(classic_solver);

    //Defines the links between the modules
    //  Send the UDP GODOT SIM frames received by the UDPChannel to the Mailbox
    if let Ok(obs)= mailbox.clone().frame_observer.try_lock() {
        if let Some(observer)= obs.as_ref() {
            udp.add_frame_observer(observer.clone().into());
            //udp.add_data_observer(*observer);
        }
    }
    //  Send the IMU measurements(from godot frame), from the Mailbox to BenchmarkFiltering
    if let Ok(mut lidar_event) = mailbox.clone().pose_filtering_event.try_lock() {
        if let Some(obs)= pose_filter.clone().get_imu_measurements_observer() {
            lidar_event.plug_observer(obs);
        }
    }

    //  Send the LiDAR measurements(from godot frame), from the Mailbox to the LidarPerceptionManager
    if let Ok(mut lidar_event) = mailbox.clone().lidar_data_event.try_lock() {
        if let Some(obs)= lidar_manager.clone().get_measurements_observer() {
            lidar_event.plug_observer(obs);
        }
    }

    //  Send the processed LiDAR measurements from LidarPerceptionManager to the Mailbox
    if let Ok(mut proc_lidar_meas_event)= lidar_manager.processed_measurements_event.try_lock() {
        if let Some(obs) = mailbox.get_lidar_measurements_observer() {
            proc_lidar_meas_event.plug_observer(obs);
        }
    }
    //  Send the filtered Pose from BenchmarkFiltering to the Mailbox
    if let Ok(mut proc_lidar_meas_event)= pose_filter.data_filtered_event.try_lock() {
        if let Some(obs) = mailbox.get_pose_filtered_observer() {
            proc_lidar_meas_event.plug_observer(obs);
        }
    }

    //  Send the Sensors measurements data frame to UDPChannel
    if let Ok(mut mailbox_event) = mailbox.clone().mailbox_event.try_lock() {
        if let Some(obs)= udp.clone().get_cmd_observer() {
            mailbox_event.plug_observer(obs);
        }
    }
    
    //Creation of the workers through the WorkerFactory
    factory.register_workers(vec![
        (udp, "UDP_INTERFACE", 100, true),
        (pose_filter, "POSE_ESTIMATOR", -1, false),
        (lidar_manager, "LIDAR_PERCEPTION", -1, false),
        (mailbox, "MAILBOX", -1, false)
    ]);
    //Defines the links between the workers

    factory.start_worker("UDP_INTERFACE");

    while true {}
}
