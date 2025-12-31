use std::sync::{Arc, Mutex};

use robomorph::{communication::UDPChannel, event_management::{Event, Observer}, lidar_management::{lidar_perception_manager::LidarPerceptionManager, measurements::LidarMeasurements, segmentation_algorithms::ClassicSolver}, messages, utils, worker::{Module, WorkerFactory}};

pub struct MailBox {
    //Receive the GODOT frame from the UDPChannel
    frame_observer: Mutex<Option<Observer<Vec<u8>>>>,
    //Sensors Measurement frame: Mailbox => UDPChannel (to be sent to the Python VisuSoft)
    mailbox_event: Mutex<Event<Vec<u8>>>,
    //LiDAR measurements GODOT frame: Mailbox => LidarPerceptionManager
    lidar_data_event: Mutex<Event<LidarMeasurements>>,
    //Receive the LiDAR measurements from LidarPerceptionManager
    processed_lidar_data_obs: Mutex<Option<Observer<LidarMeasurements>>>
}

impl Module for MailBox {
    fn exec_main_task(&self) {
        
    }
}

impl MailBox {
    pub fn new() -> Arc<Self> {
        let mut mailbox= Arc::new(Self{frame_observer: Mutex::new(None), mailbox_event: Mutex::new(Event::new_empty()), lidar_data_event: Mutex::new(Event::new_empty()), processed_lidar_data_obs: Mutex::new(None)});
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
        //Callback for processed LiDAR measurements
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
        return mailbox;

    }

    pub fn send_godot_frame(&self, frame: Vec<u8>) {
        let translatables= messages::parse_frame(frame.clone());
        let mut i= 0;
        for translatable in translatables {
            //println!("Translatable {}", i);
            i+= 1;
            match translatable.downcast_ref::<LidarMeasurements>() {
                Some(lidar_meas) => {
                    if let Ok(lidar_event) = self.lidar_data_event.try_lock() {
                       lidar_event.trig(lidar_meas.clone());
                    }
                },
                None => { },
            }
        }
    }
    pub fn get_lidar_measurements(&self) -> Option<Observer<LidarMeasurements>> {
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
}

fn main() {
    //System modules creation
    let mut factory= WorkerFactory::new();
    let mailbox= MailBox::new();
    let udp= UDPChannel::new_async("127.0.0.1", 8090, "127.0.0.1", 9000);

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
    //  Send the LiDAR measurements(from godot frame), from the Mailbox to the LidarPerceptionManager
    if let Ok(mut lidar_event) = mailbox.clone().lidar_data_event.try_lock() {
        if let Some(obs)= lidar_manager.clone().get_measurements_observer() {
            lidar_event.plug_observer(obs);
        }
    }
    //  Send the processed LiDAR measurements from LidarPerceptionManager to the Mailbox
    if let Ok(mut proc_lidar_meas_event)= lidar_manager.processed_measurements_event.try_lock() {
        if let Some(obs) = mailbox.get_lidar_measurements() {
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
        (lidar_manager, "LIDAR_PERCEPTION", -1, false),
        (mailbox, "MAILBOX", -1, false)
    ]);
    //Defines the links between the workers

    factory.start_worker("UDP_INTERFACE");

    while true {}
}
