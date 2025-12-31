use std::sync::{Arc, Mutex};

use crate::{event_management::{Event, Observer}, lidar_management::{measurements::LidarMeasurements, segmentation_algorithms::{self, SegmentationAlgorithm}}, messages, utils, worker::Module};

pub struct LidarPerceptionManager {
    object_detection_solver: Arc<dyn SegmentationAlgorithm>,
    measurements_observer: Mutex<Option<Observer<LidarMeasurements>>>,
    pub processed_measurements_event: Mutex<Event<LidarMeasurements>>,
}

impl LidarPerceptionManager {
    pub fn new(object_detection_solver: Arc<dyn SegmentationAlgorithm>) -> Arc<Self> {
        let mut this= Arc::new(LidarPerceptionManager {
            object_detection_solver: object_detection_solver.clone(), measurements_observer: Mutex::new(None), processed_measurements_event: Mutex::new(Event::new_empty())
        });
        let lidar_manager= this.clone();
        //Callback for Lidar measurements reception
        let obs= Observer::new(Arc::new(Mutex::new(move |mut measurements: LidarMeasurements| {
            //Compute lidar points segmentation to detect obstacles
            let lidar_objects= lidar_manager.object_detection_solver.detect_objects(&mut measurements);
            //Add the resulting lidar obstacles to the measurements structure
            measurements.add_lidar_objects(lidar_objects);
            //Send the processed measurements to other modules
            if let Ok(processed_measurements_event)= lidar_manager.processed_measurements_event.try_lock() {
                processed_measurements_event.trig(measurements);
            }
        })));
        if let Ok(mut obs_guard) = this.measurements_observer.try_lock() {
            *obs_guard= Some(obs);
        }
        return this;
    }

    pub fn get_measurements_observer(&self) -> Option<Observer<LidarMeasurements>> {
        if let Ok(observer) = self.measurements_observer.try_lock() {
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

impl Module for LidarPerceptionManager {
    fn exec_main_task(&self) {
        
    }
}

