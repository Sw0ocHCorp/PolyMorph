use downcast_rs::{Downcast, impl_downcast};
use ordered_float::OrderedFloat;
use std::{any::Any, collections::HashMap};

/**
 * Translatable trait
 * Defines how to Serializa / Deserializa an object (To send it via a Socket) 
 */
pub trait Translatable : Downcast + Send + Sync {
    fn translate_to_frame(&mut self) -> Vec<u8>;
    fn fill_from_frame(&mut self, raw_frame: Vec<u8>);
}
//The implementation of DownCast for the Translatable Trait means that an instanceof check is possible to know which specific translatable it is
impl_downcast!(Translatable);

//Enum to store the available features properties used for Serialization / Deserialization
pub enum FeatureProperties {
    LidarScan(LidarMessageProperties),
}

//Enum to store the data relatives to the features
#[derive(Clone)]
pub enum FeatureMessage {
    Command(String),
    LidarScan(LidarMessage),

}

//Enum to track parsing step during Deserialization
pub enum ParsingState {
    ID, 
    SIZE, 
    DATA
}

//Lidar properties for parsing (IDs of the differents lidar data)
#[derive(Clone)]
pub struct LidarMessageProperties {
    pub range_id: Vec<u8>,
    pub measurements_id: Vec<u8>,
}

//The Message relative to the lidar measurements
#[derive(Clone)]
pub struct LidarMessage {
    pub properties_ids: LidarMessageProperties,
    pub measurements: HashMap<OrderedFloat<f32>, f32>
}

impl Translatable for LidarMessage {
    fn translate_to_frame(&mut self) -> Vec<u8> {
        let mut frame:Vec<u8>= Vec::new();
        frame.append(&mut self.properties_ids.range_id);
        let mut angles: Vec<_> = self.measurements.keys().cloned().collect();
        angles.sort();
        let first_angle= angles[0];
        let last_angle = angles[angles.len() -1];
        let angles_size: u16= (first_angle.into_inner().to_be_bytes().to_vec().len() * 2) as u16;
        frame.append(&mut angles_size.to_be_bytes().to_vec());
        frame.append(&mut first_angle.into_inner().to_be_bytes().to_vec());
        frame.append(&mut last_angle.into_inner().to_be_bytes().to_vec());
        frame.append(&mut self.properties_ids.measurements_id);
        let measurement_size= ((self.measurements.clone().len()) * first_angle.to_be_bytes().to_vec().len()) as u16;
        frame.append(&mut measurement_size.to_be_bytes().to_vec());
        for angle in angles {
            frame.append(&mut self.measurements.clone()[&angle].to_be_bytes().to_vec());
        }
        let size= frame.len() as u16 + 2;
        frame.insert(0, (size).to_be_bytes().to_vec()[1]);
        frame.insert(0, (size).to_be_bytes().to_vec()[0]);
        return frame;
    }

    fn fill_from_frame(&mut self, raw_frame: Vec<u8>) {
        let mut status= ParsingState::ID;
        let mut buffer: Vec<u8>= Vec::new();
        let mut current_id: Vec<u8>= Vec::new();
        let mut data_size: u16= 0;
        let mut min_angle: Option<f32>= None;
        let mut max_angle:Option<f32>= None;
        let mut measurements: Vec<f32>= Vec::new();
        for data in raw_frame {
            buffer.push(data);
            match status {
                ParsingState::ID => {
                    if buffer == self.properties_ids.range_id || buffer == self.properties_ids.measurements_id {
                        current_id= buffer.clone();
                        status= ParsingState::SIZE;
                        buffer.clear();
                    }
                }, 
                ParsingState::SIZE =>{
                    if buffer.len() == 2 && let Ok(data)= buffer.as_slice().try_into() {
                        data_size= u16::from_be_bytes(data);
                        status= ParsingState::DATA;
                        buffer.clear();
                    }
                }, 
                ParsingState::DATA => {
                    if data_size > 0 {
                        if buffer.len() == 4 {
                            //Save the bound values of the lidar angles range
                            if current_id == self.properties_ids.range_id {
                                if min_angle == None && let Ok(data)= TryInto::<[u8; 4]>::try_into(buffer.clone().as_slice())  {
                                    min_angle= Some(f32::from_be_bytes(data));
                                    buffer.clear();
                                } else if let Ok(data)= TryInto::<[u8; 4]>::try_into(buffer.as_slice()) {
                                    max_angle= Some(f32::from_be_bytes(data));
                                    buffer.clear();
                                    status= ParsingState::ID;
                                }
                            }
                            //Storing the measurement (distance) values of the lidar
                            if current_id == self.properties_ids.measurements_id && let Ok(data)= TryInto::<[u8; 4]>::try_into(buffer.clone().as_slice()){
                                measurements.push(f32::from_be_bytes(data));
                            }
                            data_size -= buffer.len() as u16;
                            buffer.clear();
                        }
                    } else {
                        break;
                    }
                },
            }
        }
        //Compute the angle associate with the distance measurement
        if let Some(mn_angle) = min_angle && let Some(mx_angle) = max_angle {
            for i in 0..measurements.len()  {
                let angle= mn_angle + (i as f32 / (measurements.len()-1) as f32) * (mx_angle - mn_angle);

                self.measurements.insert(OrderedFloat(angle), measurements[i]);
            }
        }  
    }
}

impl LidarMessage {
    pub fn from_frame(properties_ids: LidarMessageProperties, frame: Vec<u8>) -> Self {
        let mut msg= LidarMessage{properties_ids: properties_ids, measurements: HashMap::new()};
        msg.fill_from_frame(frame);
        return msg;
    }
}