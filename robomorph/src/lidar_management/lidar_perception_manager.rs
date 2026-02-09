use std::{collections::LinkedList, sync::{Arc, Mutex}, time::{self, Instant}};

use crate::{core::{event_management::{Event, Observer}, utils, worker::Module}, lidar_management::{measurements::{self, LidarMap, LidarMeasurements, LidarObject, LidarPoint}, segmentation_algorithms::SegmentationAlgorithm}, positionning::pose::Pose};

pub struct LidarPerceptionManager {
    object_detection_solver: Arc<dyn SegmentationAlgorithm>,
    lidar_measurements_observer: Mutex<Option<Observer<LidarMeasurements>>>,
    pub lidar_map_event: Mutex<Event<LidarMap>>,
    pose_observer: Mutex<Option<Observer<Pose>>>,
    map: Arc<Mutex<LidarMap>>
}

impl LidarPerceptionManager {
    pub fn new(object_detection_solver: Arc<dyn SegmentationAlgorithm>, is_async: bool) -> Arc<Self> {
        let this= Arc::new(LidarPerceptionManager {
            object_detection_solver: object_detection_solver.clone(), lidar_measurements_observer: Mutex::new(None), 
            lidar_map_event: Mutex::new(Event::new_empty()), pose_observer: Mutex::new(None), map: Arc::new(Mutex::new(LidarMap::new()))
        });
        let orientation_obs= Observer::new_async();
        if let Ok(mut obs_guard) = this.pose_observer.try_lock() {
            *obs_guard= Some(orientation_obs)
        }
        let this_cl= this.clone();
        //Callback for Lidar measurements reception
        let mut obs= Observer::new(Arc::new(Mutex::new(move |lidar_measurements: LidarMeasurements| {
            let mut current_pose= Pose::default();
            if let Ok(mut pose_observer)= this_cl.clone().pose_observer.try_lock() && let Some(mut pose_obs) = pose_observer.as_mut() {
                if let Some(p) = pose_obs.get_incoming_data() {
                    current_pose= p;
                }
            }
            this_cl.clone().update_perception(lidar_measurements, current_pose);
            let mut map= None;
            if let Ok(mp)= this_cl.map.clone().try_lock() {
                map= Some((*mp).clone())
            }
            if let Ok(lidar_map_event)= this_cl.clone().lidar_map_event.try_lock() && let Some(mp)= map{
                lidar_map_event.trig(mp.clone());
            }
        })));
        if is_async {
            obs= Observer::new_async()
        }
        if let Ok(mut obs_guard) = this.lidar_measurements_observer.try_lock() {
            *obs_guard= Some(obs);
        }
        return this;
    }

    pub fn update_perception(&self, mut lidar_measurements: LidarMeasurements, pose: Pose) {
        //let start_time= Instant::now();
        //Compute lidar points segmentation to detect obstacles
        let lidar_objects= self.object_detection_solver.detect_objects(&mut lidar_measurements, pose.clone());
        if lidar_objects.len() > 0 {
            //Compute the closest point to get this data for potential obstacle avoidance
            let mut closest_point= [f64::INFINITY, f64::INFINITY];
            for pt in lidar_measurements.get_all_measurements() {
                let pt_loc= pt.get_location();
                let origin= pose.get_location();
                if utils::euclidean_distance(vec![origin[0], origin[1]], vec![pt_loc[0], pt_loc[1]]) < 
                            utils::euclidean_distance(vec![origin[0], origin[1]], vec![closest_point[0], closest_point[1]]){
                    closest_point= pt_loc;
                }
            }
            //Using the lidar obstacles to update the lidar map
            if let Ok(mut lidar_map)= self.map.try_lock() {
                lidar_map.update(lidar_objects, pose.clone(), LidarPoint::new_from_location(closest_point, pose));
            }
        }
        //println!("Elapsed Time for Building Perception: {:?}",Instant::now().duration_since(start_time));
    }

    pub fn get_pose_observer(&self) -> Option<Observer<Pose>> {
        if let Ok(observer) = self.pose_observer.try_lock() {
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

    pub fn get_lidar_measurements_observer(&self) -> Option<Observer<LidarMeasurements>> {
        if let Ok(observer) = self.lidar_measurements_observer.try_lock() {
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
        /*let mut current_pose= Pose::default();
        let mut lidar_measurements= None;
        if let Ok(mut pose_observer)= self.pose_observer.try_lock() && let Some(mut pose_obs) = pose_observer.as_mut() {
            if let Some(p) = pose_obs.get_incoming_data() {
                current_pose= p;
            }
        }
        if let Ok(mut lidar_measurements_obs)= self.lidar_measurements_observer.try_lock() && let Some(mut lidar_meas_obs) = lidar_measurements_obs.as_mut(){
            lidar_measurements = lidar_meas_obs.get_incoming_data();
        }

        if let Some(mut meas)= lidar_measurements {
            //Compute lidar points segmentation to detect obstacles
            let lidar_objects= self.object_detection_solver.detect_objects(&mut meas, current_pose.clone());
            if let Ok(mut map) = self.map.clone().try_lock() {
                map.update(lidar_objects, current_pose);
            }
        } 
        //No pose is available so we will get the closest object detected by the lidar
        else {
            
        }*/
    }
}

