mod filtering;

use std::{sync::{Arc, Mutex}, thread};
use faer::{Col, Mat, col, mat};
use num_quaternion::{Q64, Quaternion, UQ64};
use robomorph::{communication::UDPChannel, core::{event_management::{Event, Observer}, file_logger::FileLogger, messages, worker::{Module, WorkerFactory}}, filtering::{kalman_filter::{KalmanMeasurements, UnscentedKalmanFilter}, mahony_filter::MahonyFilter}, lidar_management::{lidar_perception_manager::LidarPerceptionManager, measurements::{LidarMap, LidarMeasurements}, segmentation_algorithms::ClassicSolver}, positionning::pose::{GPSData, IMUData, Pose}};

use crate::filtering::imu_ukf::OrientationUKF;

pub struct FiltersManager {
    imu_data_observer: Mutex<Option<Observer<IMUData>>>,
    data_filtered_event: Mutex<Event<Pose>>,
    filter: Mutex<MahonyFilter>,
    ukf_imu: Mutex<OrientationUKF>,
}

impl Module for FiltersManager {
    fn exec_main_task(&self) {
        
    }
}

impl FiltersManager {
    pub fn new(p: f64, i: f64, d: f64, max_integral_error: f64, dt: f64, ukf: OrientationUKF) -> Arc<Self> {
        let filter= MahonyFilter::new(p, i, d, max_integral_error, dt, 0.5f64.to_radians(), 1.0);
        let this = Arc::new(Self { imu_data_observer: Mutex::new(None), data_filtered_event: Mutex::new(Event::new_empty()),
                                                                    filter: Mutex::new(filter), ukf_imu: Mutex::new(ukf) });
        let mut filter_manager= this.clone();
        let obs= Observer::new(Arc::new(Mutex::new(move |imu_data: IMUData| {
            if let Ok(mut ukf)= filter_manager.ukf_imu.try_lock() {
                let state= ukf.estimate_true_state(KalmanMeasurements{input_sensor_measurements: col![imu_data.gyro[0], imu_data.gyro[1], imu_data.gyro[2]], 
                                                                                            ref_sensor_measurements: col![imu_data.accel[0], imu_data.accel[1], imu_data.accel[2], 
                                                                                                                            imu_data.magnetic_field[0], imu_data.magnetic_field[1], imu_data.magnetic_field[2]],delta_time: dt});
                let quat_state= Q64::new(state[0], state[1], state[2], state[3]);
                if let Some(unit_quat)= quat_state.normalize() && let Ok(data_event)= filter_manager.clone().data_filtered_event.try_lock() {
                    let pose= Pose::new(GPSData{latitude: 0.0, longitude: 0.0}, [0.0, 0.0, 0.0], 
                                                        unit_quat, 
                                                        [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
                    data_event.trig(pose);
                }
            }
            /*if let Ok(mut filter)= filter_manager.clone().filter.try_lock() {
                let estimated_orientation= filter.estimate_orientation(imu_data);
                //let test= filter.compute_system_state(input_array, measurement_array);
                if let Ok(data_event)= filter_manager.clone().data_filtered_event.try_lock() {
                    let pose= Pose::new(GPSData{latitude: 0.0, longitude: 0.0}, [0.0, 0.0, 0.0], 
                                                        UQ64::from_euler_angles(estimated_orientation[0], estimated_orientation[1], estimated_orientation[2]), 
                                                        [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
                    data_event.trig(pose);
                }
            }*/
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
    processed_lidar_map_obs: Mutex<Option<Observer<LidarMap>>>,
    pose_filtered_observer: Mutex<Option<Observer<Pose>>>,
    is_async: bool
}

impl Module for MailBox {
    fn exec_main_task(&self) {
        if let Ok(frame_observer) = self.frame_observer.try_lock() && let Some(obs) = frame_observer.as_ref() {
            if let Some(frame)= obs.clone().get_incoming_data(){
                self.send_godot_frame(frame);
            }
        } 
    }
}

impl MailBox {
    pub fn new(is_async: bool) -> Arc<Self> {
        let mut mailbox= Arc::new(Self{frame_observer: Mutex::new(None), mailbox_event: Mutex::new(Event::new_empty()), 
                                                            lidar_data_event: Mutex::new(Event::new_empty()), processed_lidar_map_obs: Mutex::new(None), 
                                                            pose_filtering_event: Mutex::new(Event::new_empty()), pose_filtered_observer: Mutex::new(None), is_async});
        let mut mailbox_cl= mailbox.clone();
        let mut obs= Observer::new_async();
        if is_async == false {
            //Callback for GODOT UDP frame reception
            obs= Observer::new(Arc::new(Mutex::new(move |frame: Vec<u8>| {
                //Sending the frame to LidarPerceptionManager 
                mailbox_cl.send_godot_frame(frame);
            })));
        }
        if let Ok(mut obs_guard) = mailbox.frame_observer.try_lock() {
            *obs_guard= Some(obs);
        }
        mailbox_cl= mailbox.clone();
        //Callback for processed LiDAR measurements reception
        let lidar_obs= Observer::new(Arc::new(Mutex::new(move |lidar_meas: LidarMap| {
            //Sending the processed LiDAR measurement frame to the UDPChannel 
            let frame= messages::convert_to_frame(vec![Box::new(lidar_meas)]);
            if let Ok(frame_ev_guard) = mailbox_cl.mailbox_event.try_lock() {
                frame_ev_guard.trig(frame);
            }
        })));
        if let Ok(mut lidar_observer)= mailbox.processed_lidar_map_obs.try_lock() {
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
    pub fn get_lidar_map_observer(&self) -> Option<Observer<LidarMap>> {
        if let Ok(observer) = self.processed_lidar_map_obs.try_lock() {
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
    let mailbox= MailBox::new(true);
    let udp= UDPChannel::new_async("127.0.0.1", 8090, "127.0.0.1", 9000);
    //Function used in UKF to estimate the theoric state of the system for a given sensor input measurements vector, and elapsed time
    //  In this case, state is the robot orientation in Quaternion space, the sensor_inputs is the gyro measurements
    /*let transition_function= Box::new(|state: &Col<f64>, sensor_inputs: &Col<f64>, delta_time: f64| {
        
    });*/
    //Function used in the UKF to compute the expected ref sensor measurements for a given state
    //  In this case, the state is a robot orientation in Quaternion space
    //  The ref sensors measurements we will compute are the measurements of an accelerometer and magnetometer 
    /*let ref_sensor_function= Box::new(|state: &Col<f64>, vec_size| {
        

    });*/

    let state_covariance= Mat::<f64>::identity(3, 3)*0.01;
    //Q matrix => Noise of the state that comes with the sensor input measurements (gyrometer)
    let sate_process_noise= Mat::<f64>::identity(3, 3)*0.001;

    //R matrix => Noise of the ref sensors (Accelerometer and Magnetometer)
    let mut bind = Mat::<f64>::identity(6, 6);
    let mut measurement_noise= bind.as_mut();
    let diag= [
                            0.01, 0.01, 0.01,   //Accelerometer measurements noise 
                            0.1, 0.1, 0.1       //Magnetometer measurements noise
                        ];
    for i in 0..diag.len() {
        measurement_noise[(i, i)]= diag[i];
    }
    
    // Box<dyn FnMut(&Col<f64>, &Col<f64>, f64) -> Col<f64> + Send + Sync + 'static>,
    //The sensor measurement estimation
    //  The goal of this function is to model the theoric ref sensor measurement for a given state
    //  (state, time) -> expected ref sensor measurements
    //ref_sensor_function: Box<dyn FnMut(&Col<f64>, f64) -> Col<f64> + Send + Sync + 'static>,
    //UKF for orientation estimation
    let binding = UQ64::from_euler_angles(0.0, 0.0, 0.0);
    let init_quat= binding.as_quaternion();
    let ukf_imu= OrientationUKF::new(col![init_quat.w, init_quat.x, init_quat.y, init_quat.z], state_covariance,
                                                                    measurement_noise.cloned(), sate_process_noise, 0.1, 2.0);
    let pose_filter= FiltersManager::new(20.0, 0.01, 3.25, 10.0, 1.0/60.0, ukf_imu);
    let classic_solver= Arc::new(ClassicSolver::new( 0.10, 2,
        Box::new(|p1, p2|{
            let loc1= p1.get_location();
            let loc2= p2.get_location();
            let dist= ((loc2[0] - loc1[0]).powf(2.0) + (loc2[1] - loc1[1]).powf(2.0)).sqrt();
            return dist;
        })
    ));
    //LidarPerceptionManager is Asynchronous to start processing LidarMeasurements after receiving the orientation from the MahonyFilter
    let lidar_manager= LidarPerceptionManager::new(classic_solver, false);

    //Defines the links between the modules
    //  Send the UDP GODOT SIM frames received by the UDPChannel to the Mailbox
    if let Ok(obs)= mailbox.clone().frame_observer.try_lock() {
        if let Some(observer)= obs.as_ref() {
            udp.add_frame_observer(observer.clone().into());
            //udp.add_data_observer(*observer);
        }
    }
    //  Send the IMU measurements(from godot frame), from the Mailbox to FiltersManager
    if let Ok(mut lidar_event) = mailbox.clone().pose_filtering_event.try_lock() {
        if let Some(obs)= pose_filter.clone().get_imu_measurements_observer() {
            lidar_event.plug_observer(obs);
        }
    }

    //  Send the LiDAR measurements(from godot frame), from the Mailbox to the LidarPerceptionManager
    if let Ok(mut lidar_event) = mailbox.clone().lidar_data_event.try_lock() {
        if let Some(obs)= lidar_manager.clone().get_lidar_measurements_observer() {
            lidar_event.plug_observer(obs);
        }
    }

    //  Send the processed LiDAR measurements from LidarPerceptionManager to the Mailbox
    if let Ok(mut proc_lidar_meas_event)= lidar_manager.lidar_map_event.try_lock() {
        if let Some(obs) = mailbox.get_lidar_map_observer() {
            proc_lidar_meas_event.plug_observer(obs);
        }
    }
    //  Send the filtered Pose from FiltersManager to the Mailbox
    if let Ok(mut pose_estimation_event)= pose_filter.data_filtered_event.try_lock() {
        if let Some(obs) = mailbox.get_pose_filtered_observer() {
            pose_estimation_event.plug_observer(obs);
        }
        if let Some(obs)= lidar_manager.get_pose_observer() {
            pose_estimation_event.plug_observer(obs);
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
        (mailbox.clone(), "MAILBOX", 60, mailbox.clone().is_async),
        (udp, "UDP_INTERFACE", -1, true),
        (lidar_manager, "LIDAR_PERCEPTION", -1, false),
        (pose_filter, "POSE_ESTIMATOR", -1, false),
    ]);
    //Defines the links between the workers
    factory.start_all_async_workers();
    /*factory.start_worker("UDP_INTERFACE");
    factory.start_worker("UDP_INTERFACE");*/

    thread::park();
}
