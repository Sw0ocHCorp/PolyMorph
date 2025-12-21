use std::collections::{HashMap, HashSet};

use crate::lidar::LidarMeasurements;

#[derive(Clone)]
pub enum ParsingState {
    NONE,
    SOF, 
    FRAME_SIZE,
    CHUNK
}

pub const SOF: u16= 0xabcd;
pub enum DataChunk {
    COMMAND_CHUNK= 0x0001,
    TELEOPERATION_CHUNK= 0x0002,
    GNSS_POS_CHUNK= 0x0003,
    INTERNAL_PERCEPTION_CHUNK= 0x0004,
    LIDAR_SCAN_CHUNK= 0x0005,
} 



pub trait Translatable  {
    fn fill_from_bytes(&mut self, bytes: Vec<u8>) -> usize;

    fn to_bytes(&mut self) -> Vec<u8>;
}

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
    for mut i in 0..frame.len() {
        buffer.push(frame[i]);
        if frame_size > 0 {
            if buffer.len() >= 2{
                match &buffer[buffer.len() - 2..] {
                    [hi, lo] => {
                        if parsing_state.clone() as u16 == ParsingState::SOF as u16 {
                            if u16::from_be_bytes([*hi, *lo]) == SOF  {
                                parsing_state= ParsingState::FRAME_SIZE;
                                buffer.clear();
                            } 
                        } else if parsing_state.clone() as u16 == ParsingState::FRAME_SIZE as u16 {
                            frame_size= u16::from_be_bytes([*hi, *lo]);
                            parsing_state= ParsingState::CHUNK;
                            buffer.clear();
                        }
                        else {
                            if u16::from_be_bytes([*hi, *lo]) == DataChunk::LIDAR_SCAN_CHUNK as u16 {
                                let mut lidar_measurements= LidarMeasurements::new();
                                let chunk_size= lidar_measurements.fill_from_bytes(frame[i+1..].to_vec());
                                translatables.push(Box::new(lidar_measurements));
                                //let chunk_size= LidarMeasurements::new_from_bytes(frame[i+1..].to_vec());
                                i += chunk_size;
                                buffer.clear();
                            }
                        }
                    }
                    _ => {
                    }
                }
            }
        } else {
            break;
        }
    }
    return translatables;
}

