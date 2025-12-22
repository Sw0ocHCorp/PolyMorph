use std::collections::HashMap;

use ordered_float::OrderedFloat;

use crate::{messages::{DataChunk, Translatable}, utils, worker::Module};

pub const LIDAR_ANGLE_RANGE: u16= 0x000a;
pub const LIDAR_MEASUREMENT: u16= 0x000b;

pub struct LidarMeasurements {
    measurements: HashMap<OrderedFloat<f32>, f32>
}

impl LidarMeasurements {
    pub fn new_from_bytes(bytes: Vec<u8>) -> Self {
        let mut mes= LidarMeasurements::new();
        mes.fill_from_bytes(bytes);
        return mes;
    }

    pub fn new() ->Self {
        return Self { measurements: HashMap::new() }
    }

    pub fn new_from_measurements(measurements: HashMap<OrderedFloat<f32>, f32>) -> Self {
        return Self {measurements: measurements};
    }

    pub fn insert(&mut self, angle: f32, distance: f32) {
        self.measurements.insert(OrderedFloat(angle), distance);
    }

    pub fn order_by_angle(&mut self) -> HashMap<OrderedFloat<f32>, f32> {
        let mut angles: Vec<_> = self.measurements.keys().cloned().collect();
        let mut new_mes:HashMap<OrderedFloat<f32>, f32>= HashMap::new();
        let mes= self.measurements.clone();
        for angle in angles {
            new_mes.insert(angle, mes[&angle]);
        }
        return new_mes;
    }

    pub fn len(&self) -> usize {
        return self.measurements.len();
    }
}

impl Translatable for LidarMeasurements {
    fn fill_from_bytes(&mut self, bytes: Vec<u8>) -> usize {
        let mut buffer= Vec::new();
        let mut first_angle= 0.0;
        let mut last_angle= 0.0;
        let mut i= 0;
        let mut data_size= -1;
        let mut remain_bytes= 0;
        let mut id= 0;
        while i < bytes.len() {
            buffer.push(bytes[i]);
            if id == LIDAR_ANGLE_RANGE {
                if let Ok(arr) = bytes[i..i+4].try_into() {
                    first_angle= f32::from_be_bytes(arr);
                    i+=4;
                    if let Ok(arr) = bytes[i..i+4].try_into() {
                        last_angle= f32::from_be_bytes(arr);
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
                        buffer.clear();
                    }
                } else {
                    if let Ok(arr) = bytes[i..i+4].try_into() {
                        let angle= first_angle + (1.0 - ((remain_bytes-1) as f32 / (data_size-1) as f32))*(last_angle - first_angle);
                        self.insert(angle, f32::from_be_bytes(arr));
                        i+=4;
                        remain_bytes-=1;
                        buffer.clear();
                        if remain_bytes == 0 {
                            break;
                        }
                    }
                }

            } else {
                if utils::contain_bytes(buffer.clone(), LIDAR_ANGLE_RANGE.to_be_bytes().to_vec()) >= 0{
                    id= LIDAR_ANGLE_RANGE;
                    buffer.clear();
                    
                }if utils::contain_bytes(buffer.clone(), LIDAR_MEASUREMENT.to_be_bytes().to_vec()) >= 0{
                    id= LIDAR_MEASUREMENT;
                    buffer.clear();
                    
                }
                i+=1;
            }

        }
        return i as usize;
    }

    fn to_bytes(&mut self) -> Vec<u8> {
        let mut frame:Vec<u8>= (DataChunk::LIDAR_SCAN_CHUNK as u16).to_be_bytes().to_vec();
        //Let 2 bytes for the frame size
        frame.append(&mut (0 as u16).to_be_bytes().to_vec());
        frame.append(&mut LIDAR_ANGLE_RANGE.to_be_bytes().to_vec());
        let mut angles: Vec<_>= self.measurements.keys().cloned().collect();
        angles.sort();
        let first_angle= angles[0];
        
        let last_angle = angles[angles.len()-1];
        frame.append(&mut f32::to_be_bytes(first_angle.into_inner()).to_vec());
        frame.append(&mut f32::to_be_bytes(last_angle.into_inner()).to_vec());
        frame.append(&mut LIDAR_MEASUREMENT.to_be_bytes().to_vec());
        frame.append(&mut u16::to_be_bytes(angles.len() as u16).to_vec());
        for angle in angles {
            frame.append(&mut f32::to_be_bytes(self.measurements[&angle]).to_vec());
        }
        let frame_size= u16::to_be_bytes(frame.len() as u16);
        frame[2]= frame_size[0];
        frame[3]= frame_size[1];
        return frame;
    }
}

pub struct LidarPerceptionManager {
    
}

impl Module for LidarPerceptionManager {
    fn exec_main_task(&self) {
        
    }
}

