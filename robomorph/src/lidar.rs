use std::collections::HashMap;

use ordered_float::OrderedFloat;

use crate::{messages::{DataChunk, Translatable}, utils, worker::Module};

pub const LIDAR_ANGLES_CONFIG: u16= 0x000a;
pub const LIDAR_MEASUREMENT: u16= 0x000b;

#[derive(Debug)]
pub struct LidarMeasurements {
    angle_measurements: Vec<f32>,
    distance_measurements: Vec<f32>,
    angle_in_deg: bool,
}

impl LidarMeasurements {
    pub fn new_from_bytes(bytes: Vec<u8>) -> Self {
        let mut mes= LidarMeasurements::new(false);
        mes.fill_from_bytes(bytes);
        return mes;
    }

    pub fn new(angle_in_deg: bool) ->Self {
        return Self { angle_measurements: Vec::new(), distance_measurements: Vec::new(), angle_in_deg };
    }

    pub fn new_from_measurements(angle: Vec<f32>, distance: Vec<f32>, angle_in_deg: bool) -> Self {
        return Self { angle_measurements: angle, distance_measurements: distance, angle_in_deg };
    }

    pub fn insert(&mut self, angle: f32, distance: f32) {
        self.angle_measurements.push(angle);
        self.distance_measurements.push(distance);
    }

    pub fn len(&self) -> usize {
        return self.angle_measurements.len();
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
                        buffer.clear();
                    }
                } else {
                    if let Ok(arr) = bytes[i..i+4].try_into() {
                        current_angle += angle_step;
                        self.insert(current_angle, f32::from_be_bytes(arr));
                        j+=1;
                        i+=4;
                        remain_bytes-=1;
                        buffer.clear();
                        if remain_bytes == 0 {
                            break;
                        }
                    }
                }

            } else {
                if utils::contain_bytes(buffer.clone(), LIDAR_ANGLES_CONFIG.to_be_bytes().to_vec()) >= 0{
                    id= LIDAR_ANGLES_CONFIG;
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
        //frame.append(&mut (0 as u16).to_be_bytes().to_vec());
        frame.append(&mut LIDAR_ANGLES_CONFIG.to_be_bytes().to_vec());
        let angle_step= self.angle_measurements[1] - self.angle_measurements[0];
        //let last_angle = utils::modulo_2pi(angles[angles.len()-1].into_inner());
        if self.angle_in_deg {
            frame.append(&mut f32::to_be_bytes(self.angle_measurements[0].to_radians()).to_vec());
            frame.append(&mut f32::to_be_bytes(angle_step.to_radians()).to_vec());
            frame.append(&mut LIDAR_MEASUREMENT.to_be_bytes().to_vec());
            frame.append(&mut u16::to_be_bytes(self.angle_measurements.len() as u16).to_vec());
            for i in 0..self.angle_measurements.len() {
                frame.append(&mut f32::to_be_bytes((self.distance_measurements[i])).to_vec());
            }
        }
        else {
            frame.append(&mut f32::to_be_bytes(self.angle_measurements[0]).to_vec());
            frame.append(&mut f32::to_be_bytes(angle_step).to_vec());
            frame.append(&mut LIDAR_MEASUREMENT.to_be_bytes().to_vec());
            frame.append(&mut u16::to_be_bytes(self.angle_measurements.len() as u16).to_vec());
            for i in 0..self.angle_measurements.len() {
                frame.append(&mut f32::to_be_bytes((self.distance_measurements[i])).to_vec());
            }
        }
        //let frame_size= u16::to_be_bytes(frame.len() as u16);
        //frame[2]= frame_size[0];
        //frame[3]= frame_size[1];
        return frame;
    }
}

pub struct LidarPerceptionManager {
    
}

impl Module for LidarPerceptionManager {
    fn exec_main_task(&self) {
        
    }
}

