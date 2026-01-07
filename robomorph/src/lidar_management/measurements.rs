use crate::core::{messages::{DataChunk, Translatable}, utils};



pub const LIDAR_ANGLES_CONFIG: u16= 0x000a;
pub const LIDAR_MEASUREMENT: u16= 0x000b;
pub const LIDAR_OBSTACLES: u16=0x000c;

#[derive(Debug, Clone, Copy)]
pub struct LidarPoint {
    angle: f32,
    distance: f32,
    location: (f32, f32),
    cluster_id: u32,
}



impl LidarPoint{
    pub fn new(angle: f32, distance: f32) -> Self {
        let x= distance * angle.cos();
        let y= distance * angle.sin();
        return Self { angle, distance, location: (x, y), cluster_id: 0 };
    }

    pub fn set_id(&mut self, id: u32) {
        self.cluster_id= id;
    }

    pub fn get_id(&self) -> u32 {
        return self.cluster_id;
    }
    pub fn get_location(&self) -> (f32, f32) {
        return self.location;
    }
    pub fn get_angle(&self) -> f32 {
        return self.angle;
    }
}

#[derive(Debug, Clone)]
pub struct LidarObject {
    inner_pts: Vec<LidarPoint>,
    cluster_id: u32,
    pub bound_index: usize
    
}
impl LidarObject {
    fn new(cluster_id: u32, inner_pts: Vec<LidarPoint>) -> Self {
        return Self { inner_pts: inner_pts, cluster_id: cluster_id, bound_index: 0}
    }

    pub fn new_empty(cluster_id: u32) -> Self {
        return Self { inner_pts: Vec::new(), cluster_id: cluster_id, bound_index: 0}
    }

    pub fn add_inner_points(&mut self, mut pts: Vec<LidarPoint>) {
        self.inner_pts.append(&mut pts);
    }
    
    pub fn add_inner_point(&mut self, pt: LidarPoint) {
        self.inner_pts.push(pt);
    }

    pub fn get_id(&self) -> u32 {
        return self.cluster_id;
    }

    pub fn get_inner_points(&self) -> Vec<LidarPoint> {
        return self.inner_pts.clone();
    }

    pub fn get_bound_index(&self) -> usize {
        return self.bound_index;
    }
}

#[derive(Debug, Clone)]
pub struct LidarMeasurements {
    pub lidar_pts: Vec<LidarPoint>,
    pub angle_in_deg: bool,
    pub lidar_objects: Vec<LidarObject>
}

impl LidarMeasurements {
    pub fn new_from_bytes(bytes: Vec<u8>) -> Self {
        let mut mes= LidarMeasurements::new(false);
        mes.fill_from_bytes(bytes);
        return mes;
    }

    pub fn new(angle_in_deg: bool) ->Self {
        return Self { lidar_pts: Vec::new(), angle_in_deg, lidar_objects: Vec::new() };
    }

    pub fn new_from_measurements(angle: Vec<f32>, distance: Vec<f32>, angle_in_deg: bool) -> Self {
        let mut measurements= Vec::new();
        for i in 0..angle.len() {
            measurements.push(LidarPoint::new(angle[i], distance[i]));
        }
        return Self { lidar_pts: measurements, angle_in_deg, lidar_objects: Vec::new() };
    }

    pub fn insert(&mut self, angle: f32, distance: f32) {
        self.lidar_pts.push(LidarPoint::new(angle, distance));
    }

    pub fn add_lidar_objects(&mut self, mut lidar_objects: Vec<LidarObject>) {
        self.lidar_objects.append(&mut lidar_objects);
    }

    pub fn len(&self) -> usize {
        return self.lidar_pts.len();
    }

    pub fn set_pt_id(&mut self, index: usize, id: u32) {
        self.lidar_pts[index].cluster_id= id
    }

    pub fn get_lidar_point_index(&self, pt: LidarPoint) -> i32 {
        let mut idx= -1;
        for i in 0..self.lidar_pts.len() {
            if pt.get_angle() == self.lidar_pts[i].get_angle() {
                idx= i as i32;
            }
        }
        return idx;
    }

    pub fn get_measurement_by_index(&self, index: usize) -> Option<LidarPoint> {
        if index < self.lidar_pts.len() {
            return Some(self.lidar_pts[index].clone());
            
        }
        return None;
    }

    pub fn get_closest_measurement(&self, angle: f32) -> Option<LidarPoint> {
        if self.lidar_pts.len() == 0 {
            return None;
        }
        let mut closest_index= 0;
        let mut closest_diff= f32::MAX;
        for i in 0..self.lidar_pts.len() {
            let diff= (self.lidar_pts[i].angle - angle).abs();
            if diff < closest_diff {
                closest_diff= diff;
                closest_index= i;
            }
        }
        return Some(self.lidar_pts[closest_index].clone());
    }

    pub fn get_all_measurements(&self) -> Vec<LidarPoint> {
        return self.lidar_pts.clone();
    }

}


impl Translatable for LidarMeasurements {
    fn fill_from_bytes(&mut self, bytes: Vec<u8>) -> usize {
        let mut buffer= Vec::new();
        let mut current_angle= 0.0;
        let mut angle_step= 0.0;
        let mut i= 0;
        let mut j=0;
        let mut data_size= -1;
        let mut remain_bytes= 0;
        let mut id= 0;
        let mut cluster_id= 1;
        let mut bound_index= 0;
        while i < bytes.len() {
            buffer.push(bytes[i]);
            if id == LIDAR_ANGLES_CONFIG {
                if let Ok(arr) = bytes[i..i+4].try_into() {
                    current_angle= f32::from_be_bytes(arr);
                    i+=4;
                    if let Ok(arr) = bytes[i..i+4].try_into() {
                        angle_step= f32::from_be_bytes(arr);
                        current_angle -= angle_step;
                    }
                    i+=4;
                }
                buffer.clear();
                id = 0;
            } else if id == LIDAR_MEASUREMENT {
                if data_size < 0 {
                    if let Ok(arr) = bytes[i..i+2].try_into() {
                        data_size= u16::from_be_bytes(arr) as i32;
                        remain_bytes= u16::from_be_bytes(arr);
                        i+= 2;
                    }
                } else {
                    if let Ok(arr) = bytes[i..i+4].try_into() {
                        current_angle += angle_step;
                        self.insert(current_angle, f32::from_be_bytes(arr));
                        j+=1;
                        i+=4;
                        remain_bytes-=1;
                        if remain_bytes == 0 {
                            id = 0;
                            data_size= -1;
                        }
                    }
                }
                buffer.clear();
            } else if id == LIDAR_OBSTACLES {
                if let Ok(arr) = bytes[i..i+2].try_into() {
                    if data_size < 0 {
                        data_size= u16::from_be_bytes(arr) as i32;
                    } else {
                        cluster_id += 1;
                        let mut obj= LidarObject::new_empty(cluster_id);
                        obj.bound_index= u16::from_be_bytes(arr) as usize;
                        for n in bound_index..=obj.bound_index {
                            self.lidar_pts[n].cluster_id= cluster_id;
                            obj.add_inner_point(self.lidar_pts[n]);
                        }
                        bound_index= obj.bound_index+1;
                        self.lidar_objects.push(obj);
                    }
                }
                i+= 2;
                buffer.clear();
            } else {
                if utils::contain_bytes(buffer.clone(), LIDAR_ANGLES_CONFIG.to_be_bytes().to_vec()) >= 0{
                    id= LIDAR_ANGLES_CONFIG;
                    buffer.clear();
                    
                } if utils::contain_bytes(buffer.clone(), LIDAR_MEASUREMENT.to_be_bytes().to_vec()) >= 0{
                    id= LIDAR_MEASUREMENT;
                    buffer.clear();
                    
                } if utils::contain_bytes(buffer.clone(), LIDAR_OBSTACLES.to_be_bytes().to_vec()) >= 0{
                    id= LIDAR_OBSTACLES;
                    buffer.clear();
                    
                }
                i+=1;
            }

        }
        return i as usize;
    }

    fn to_bytes(&mut self) -> Vec<u8> {
        let mut frame:Vec<u8>= (DataChunk::LidarScanChunk as u16).to_be_bytes().to_vec();
        //Let 2 bytes for the frame size
        //frame.append(&mut (0 as u16).to_be_bytes().to_vec());
        frame.append(&mut LIDAR_ANGLES_CONFIG.to_be_bytes().to_vec());
        let angle_step= self.lidar_pts[1].angle - self.lidar_pts[0].angle;
        //let last_angle = utils::modulo_2pi(angles[angles.len()-1].into_inner());
        if self.angle_in_deg {
            frame.append(&mut f32::to_be_bytes(self.lidar_pts[0].angle.to_radians()).to_vec());
            frame.append(&mut f32::to_be_bytes(angle_step.to_radians()).to_vec());
            frame.append(&mut LIDAR_MEASUREMENT.to_be_bytes().to_vec());
        }
        else {
            frame.append(&mut f32::to_be_bytes(self.lidar_pts[0].angle).to_vec());
            frame.append(&mut f32::to_be_bytes(angle_step).to_vec());
            frame.append(&mut LIDAR_MEASUREMENT.to_be_bytes().to_vec());
        }
        frame.append(&mut u16::to_be_bytes(self.lidar_pts.len() as u16).to_vec());
        for i in 0..self.lidar_pts.len() {
            frame.append(&mut f32::to_be_bytes(self.lidar_pts[i].distance).to_vec());
        }
        frame.append(&mut LIDAR_OBSTACLES.to_be_bytes().to_vec());
        frame.append(&mut u16::to_be_bytes(self.lidar_objects.clone().len() as u16).to_vec());
        for obj in self.lidar_objects.clone() {
            frame.append(&mut u16::to_be_bytes(obj.bound_index as u16).to_vec());
        }
        //let frame_size= u16::to_be_bytes(frame.len() as u16);
        //frame[2]= frame_size[0];
        //frame[3]= frame_size[1];
        return frame;
    }
}