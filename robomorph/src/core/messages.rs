use std::fmt::Debug;

use downcast_rs::{Downcast, impl_downcast};

use crate::{lidar_management::measurements::LidarMeasurements, positionning::pose::{IMUData, Pose}};

#[derive(Clone)]
pub enum ParsingState {
    NONE,
    SOF, 
    FrameSize,
    CHUNK
}

pub const SOF: u16= 0xabcd;
pub enum DataChunk {
    CommandChunk= 0x0001,
    TeleoperationChunk= 0x0002,
    GnssPosChunk= 0x0003,
    InternalPerceptionChunk= 0x0004,
    LidarScanChunk= 0x0005,
    DebugChunk= 0xfeef,
} 


pub trait Translatable : Debug + Downcast {
    fn fill_from_bytes(&mut self, bytes: Vec<u8>) -> usize;

    fn to_bytes(&mut self) -> Vec<u8>;
}
impl_downcast!(Translatable);

pub fn convert_to_frame(translatables: Vec<Box<dyn Translatable>>) -> Vec<u8> {
    let mut frame= SOF.to_be_bytes().to_vec();
    frame.append(&mut (0 as u16).to_be_bytes().to_vec());
    for mut translatable in translatables {
        frame.append(&mut translatable.to_bytes());
    }
    let frame_size= u16::to_be_bytes(frame.len() as u16);
    frame[2]= frame_size[0];
    frame[3]= frame_size[1];
    return frame;
}

pub fn parse_frame(frame: Vec<u8>) -> Vec<Box<dyn Translatable>> {
    let mut translatables:Vec<Box<dyn Translatable>>= Vec::new();
    let mut parsing_state= ParsingState::SOF;
    let mut frame_size= 1;
    let mut buffer:Vec<u8>= Vec::new();
    let mut chunk_size= 0;
    let mut raw_frame= frame.clone();
    while raw_frame.len() > 0 {
        buffer.push(raw_frame[0]);
        raw_frame.remove(0);
        if buffer.len() >= 2{
            match &buffer[buffer.len() - 2..] {
                            [hi, lo] => {
                        if parsing_state.clone() as u16 == ParsingState::SOF as u16 {
                            if u16::from_be_bytes([*hi, *lo]) == SOF  {
                                parsing_state= ParsingState::FrameSize;
                                buffer.clear();
                            } 
                        } else if parsing_state.clone() as u16 == ParsingState::FrameSize as u16 {
                            frame_size= u16::from_be_bytes([*hi, *lo]);
                            parsing_state= ParsingState::CHUNK;
                            buffer.clear();
                        }
                        else {
                            if u16::from_be_bytes([*hi, *lo]) == DataChunk::LidarScanChunk as u16 {
                                let mut lidar_measurements= LidarMeasurements::new(false);
                                chunk_size= lidar_measurements.fill_from_bytes(raw_frame.clone());
                                translatables.push(Box::new(lidar_measurements));
                                raw_frame.drain(..chunk_size);
                                buffer.clear();
                            } else if u16::from_be_bytes([*hi, *lo]) == DataChunk::InternalPerceptionChunk as u16 {
                                let mut imu_data= IMUData::new();
                                chunk_size= imu_data.fill_from_bytes(raw_frame.clone());
                                translatables.push(Box::new(imu_data));
                                if chunk_size > raw_frame.len() {
                                    let test= 1;
                                }
                                raw_frame.drain(..chunk_size);
                                buffer.clear();
                            }
                        }
                    }
                    _ => {
                    }
            }
        }        
    }
    return translatables;
}

