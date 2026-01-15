
use crate::{lidar_management::measurements::{LidarMeasurements, LidarObject, LidarPoint}, positionning::pose::Pose};

pub trait SegmentationAlgorithm: Send + Sync {
    fn detect_objects(&self, measurements: &mut LidarMeasurements, robot_pose: Pose) -> Vec<LidarObject>;
}

//Classic Solver based on DBSCAN algorithm
pub struct ClassicSolver {
    distance_threshold: f32,
    min_pts_object: usize,
    distance_function: Box<dyn Fn(LidarPoint, LidarPoint) -> f32 + Send + Sync>,
}

impl ClassicSolver {
    pub fn new(distance_threshold: f32, min_pts_object: usize, distance_function: Box<dyn Fn(LidarPoint, LidarPoint) -> f32 + Send + Sync>) -> Self {
        return Self { distance_threshold, min_pts_object, distance_function };
    }
    pub fn get_neighbours_idxs(&self, target_pt_idx: usize, lidar_pts: Vec<LidarPoint>) -> Vec<usize> {
        let mut idxs= Vec::new();
        let mut prev_idx= None;     //index to iterate through the previous lidar points(in the lidar scan) 
        let mut next_idx= None;     //index to iterate through the nexts lidar points(in the lidar scan)
        //Stop searching if indexes will be out of bound
        if target_pt_idx > 0 {
            prev_idx= Some(target_pt_idx - 1);
        }
        if target_pt_idx < lidar_pts.len()-1 {
            next_idx= Some(target_pt_idx +1);
        }
        let mut stop_prev= false;
        let mut stop_next= false;

        /* To get ALL the neighbours
            We iterate through the previous points && the next points of the target pt
            We register the previous / nexts points if they have at least 1 neighbours. 
         */
        while stop_prev == false || stop_next == false {
            if let Some(prev_i)= prev_idx {
                let dist= (self.distance_function)(lidar_pts[target_pt_idx].clone(), lidar_pts[prev_i].clone());
                //IF the previous point is enought closer to the target point, this previous point is a neighbour
                if dist < self.distance_threshold {
                    //println!("Distance between {}:{:?} AND {}:{:?}= {}", target_pt_idx, lidar_pts[target_pt_idx].get_location(), prev_i, lidar_pts[prev_i].get_location(), dist);
                    idxs.push(prev_i);
                    if prev_i > 0 {
                        prev_idx= Some(prev_i - 1);
                    } else {
                        prev_idx= None;
                    }
                }
                //ELSE the previous point is to far from the target point 
                else {
                    //All the neighbours in previous lidar points are known, stop searching for others
                    stop_prev = true;
                    prev_idx = None;
                    //println!("==============================");
                }
            } else {
                stop_prev= true;
            }
            if let Some(next_i)= next_idx {   
                let dist= (self.distance_function)(lidar_pts[target_pt_idx].clone(), lidar_pts[next_i].clone());
                //IF the next point is enought closer to the target point, this next point is a neighbour
                if dist < self.distance_threshold {
                    //println!("Distance between {}:{:?} AND {}:{:?}= {}", target_pt_idx, lidar_pts[target_pt_idx].get_location(), next_i, lidar_pts[next_i].get_location(), dist);
                    idxs.push(next_i);
                    if next_i < lidar_pts.len()-1 {
                        next_idx= Some(next_i + 1);
                    } else {
                        next_idx= None
                    }
                } else {
                    //All the neighbours in nexts lidar points are known, stop searching for others
                    stop_next = true;
                    next_idx= None;
                    //println!("==============================");
                }
            } else {
                stop_next= true;
            }
        }
        return idxs;
    }


    pub fn get_clustered_lidar_pts_indexes(&self, target_pt_idx: usize, neighbours_idxs: Vec<usize>, lidar_pts: Vec<LidarPoint>) -> Vec<usize> {
        let mut idxs= neighbours_idxs.clone();
        idxs.push(target_pt_idx);
        let mut prev_idx= None;     //index to iterate through the previous lidar points(in the lidar scan)
        let mut next_idx= None;     //index to iterate through the nexts lidar points(in the lidar scan)
        //Stop searching if indexes will be out of bound
        if target_pt_idx > 0 {
            prev_idx= Some(target_pt_idx - 1);
        }
        if target_pt_idx < lidar_pts.len()-1 {
            next_idx= Some(target_pt_idx +1);
        }
        let mut stop_prev= false;
        let mut stop_next= false;

        /* To get ALL the lidar_pts_contained in the lidar object
            We iterate through the previous points && the next points of the target pt
            We register the previous / nexts points if they have at least 1 neighbours. 
         */
        while stop_prev == false || stop_next == false {
            //IF the previous point is a known neighbour
            if let Some(prev_i)= prev_idx && idxs.contains(&prev_i) {
                let neighbours_idx= self.get_neighbours_idxs(prev_i, lidar_pts.clone());
                //println!("_________________________________");
                //IF this known neighbour have neighbours
                if neighbours_idx.len() > 0{
                    for idx in neighbours_idx {
                        if idxs.contains(&idx) == false {
                            idxs.push(idx);
                        }
                    }
                    if prev_i > 0 {
                        prev_idx= Some(prev_i - 1);
                    } else {
                        prev_idx= None;
                    }

                } else {
                    //The neighbour point have no neighbours, stop searching for others
                    stop_prev = true;
                    prev_idx = None;
                }
            } else {
                stop_prev= true;
            }
            //IF the next point is a known neighbour
            if let Some(next_i)= next_idx  && idxs.contains(&next_i) {   
                let neighbours_idx= self.get_neighbours_idxs(next_i, lidar_pts.clone());
                //println!("_________________________________");
                //IF this known neighbour have neighbours
                if neighbours_idx.len() > 0 {
                    for idx in neighbours_idx {
                        if idxs.contains(&idx) == false {
                            idxs.push(idx);
                        }
                    }
                    if next_i < lidar_pts.len()-1 {
                        next_idx= Some(next_i + 1);
                    } else {
                        next_idx= None
                    }
                } else {
                    //The neighbour point have no neighbours, stop searching for others
                    stop_next = true;
                    next_idx= None;
                }
            } else {
                stop_next= true;
            }
        }
        return idxs;
    }

}

impl SegmentationAlgorithm for ClassicSolver {
    fn detect_objects(&self, measurements: &mut LidarMeasurements, robot_pose: Pose) -> Vec<LidarObject> {
        let mut objects: Vec<LidarObject> = Vec::new();
        let mut cluster_id= 1;
        for i in 0..measurements.len() {
            //Detection of the core points of objects
            if let Some(target_pt)= measurements.get_measurement_by_index(i) && target_pt.clone().get_id() == 0 {
                //Get the indexs of the neighbours lidar points
                let neighbours_indexes= self.get_neighbours_idxs(i, measurements.get_all_measurements());
                //Core point detected
                //  Trying to expand the cluster to find all the inner points of the object
                if neighbours_indexes.len() >= self.min_pts_object { 
                    let object_pts_indexs= self.get_clustered_lidar_pts_indexes(i, neighbours_indexes.clone(), measurements.get_all_measurements());
                    let mut inner_pts= Vec::new()
;                    measurements.set_pt_id(i, cluster_id);
                    let mut bound_index= 0;
                    for idx in object_pts_indexs {
                        if idx > bound_index {
                            bound_index= idx;
                        }
                        measurements.set_pt_id(idx, cluster_id);
                        if let Some(mut pt) = measurements.get_measurement_by_index(idx).clone() {
                            let mut lidar_pt= LidarPoint::new_from_pose(pt.get_angle(), pt.get_distance(), robot_pose.clone());
                            lidar_pt.set_id(cluster_id);
                            
                            inner_pts.push(lidar_pt);
                        }
                    }
                    objects.push(LidarObject::new(cluster_id, inner_pts, robot_pose.clone()));
                    cluster_id+= 1;
                }
            }
        }
        return objects;
    }
    
}

pub fn contain_id(id: u32, pts: Vec<LidarPoint>) -> bool {
    for pt in pts {
        if pt.get_id() == id {
            return true;
        }
    }
    return false;
}

pub fn contains_lidar_point(pt: LidarPoint, pts: Vec<LidarPoint>) -> bool {
    for p in pts {
        if p.get_angle() == pt.get_angle() && p.get_location() == pt.get_location() {
            return true;
        }
    }
    return false;
}