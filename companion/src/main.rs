use std::sync::{Arc, Mutex};

use robomorph::{communication::UDPChannel, event_management::{Event, Observer}, lidar::LidarPerceptionManager, worker::{Module, WorkerFactory}};

pub struct MailBox {
    frame_observer: Mutex<Option<Observer<Vec<u8>>>>,
    loopback_event: Mutex<Event<Vec<u8>>>
}

impl Module for MailBox {
    fn exec_main_task(&self) {
        
    }
}

impl MailBox {
    pub fn new() -> Arc<Self> {
        let mut mailbox= Arc::new(Self{frame_observer: Mutex::new(None), loopback_event: Mutex::new(Event::new_empty())});
        let mailbox_cl= mailbox.clone();
        let obs= Observer::new(Arc::new(Mutex::new(move |frame: Vec<u8>| {
            mailbox_cl.send_loopback(frame);
        })));
        if let Ok(mut obs_guard) = mailbox.frame_observer.try_lock() {
            *obs_guard= Some(obs);
        }
        return mailbox;

    }

    pub fn send_loopback(&self, frame: Vec<u8>) {
        if let Ok(loopback_event) = self.loopback_event.try_lock() {
            loopback_event.trig(frame);
        }
    }
}

fn main() {
    //System modules creation
    let mut factory= WorkerFactory::new();
    let mailbox= MailBox::new();
    let udp= UDPChannel::new_async("127.0.0.1", 8090, "127.0.0.1", 9000);
    let lidar_perception_management= Arc::new(LidarPerceptionManager{});
    //Defines the links between the modules
    if let Some(obs)= udp.clone().get_cmd_observer() {
        if let Ok(mut loopback_event) = mailbox.clone().loopback_event.try_lock() {
            loopback_event.plug_observer(obs);
        }
    }
    if let Ok(obs)= mailbox.clone().frame_observer.try_lock() {
        if let Some(observer)= obs.as_ref() {
            udp.add_data_observer(observer.clone().into());
            //udp.add_data_observer(*observer);
        }
    }

    //Creation of the workers through the WorkerFactory
    factory.register_workers(vec![
        (udp, "UDP_INTERFACE", 100, true),
        (lidar_perception_management, "LIDAR_PERCEPTION", -1, false),
        (mailbox, "MAILBOX", -1, false)
    ]);
    //Defines the links between the workers

    factory.start_worker("UDP_INTERFACE");

    while true {}
}
