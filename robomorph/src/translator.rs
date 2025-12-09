use std::{collections::HashMap, io::Error};

use crate::{messages::{FeatureMessage, FeatureProperties, LidarMessage, Translatable}, translator};

/**
 * Translator to convert raw frame data to FeatureMessage structs
 * Allow to manipulate the incoming frame data to known data struct
 */
pub struct Translator {
    translation_patterns: HashMap<u16, FeatureProperties>,
    sof: Vec<u8>
}

impl Default for Translator {
    fn default() -> Self {
        return Self {sof: Vec::new(), translation_patterns: HashMap::new() };
    }
}

impl Translator {
    pub fn new(sof:Vec<u8>, translation_patterns: HashMap<u16, FeatureProperties>) -> Self {
        return Translator { sof: sof, translation_patterns: translation_patterns };
    }
    //Translate a given frame to a list of Translatable object
    pub fn translate_to_messages(&mut self, raw_frame: Vec<u8>) -> Option<Vec<Box<dyn Translatable>>> {
        let mut result_msgs: Vec<Box<dyn Translatable>>= Vec::new();
        let mut frame= raw_frame.clone();
        let mut buffer:Vec<u8>= Vec::new();
        let mut is_frame= false;
        println!("Len data received= {}",raw_frame.len());
        while frame.len() > 0 {
            let byte= frame[0];
            buffer.push(byte);
            frame.remove(0);
            if buffer == self.sof {
                is_frame= true;
                buffer.clear();
            }
            if is_frame {
                for (id, translation) in &self.translation_patterns {
                    if buffer == id.to_be_bytes().to_vec() {
                        match translation {
                            FeatureProperties::LidarScan(lidar_message_properties) => {
                                if frame.len() > 2 {
                                    let size_to_remove= u16::from_be_bytes([frame[0],frame[1]]) as usize;
                                    let msg= LidarMessage::from_frame(lidar_message_properties.clone(), frame.clone()[2..].to_vec());
                                    result_msgs.push(Box::new(msg));
                                    buffer.clear();
                                    frame.drain(0..size_to_remove);
                                }
                                break;
                            },
                        }
                    }
                }
            }
        }
        println!();
        if result_msgs.len() == 0 {
            return None;
        }
        else {
            return Some(result_msgs);
        }
        
    }

    //Translate a list of 
    pub fn translate_to_frame(&mut self, translatables: Vec<Box<dyn Translatable>>) -> Vec<u8> {
        let mut frame= Vec::from(self.sof.clone());
        for mut translatable in translatables {
            if let Some(lidar_msg) = translatable.downcast_mut::<LidarMessage>() {
                for (id, translation) in &self.translation_patterns {
                    if let FeatureProperties::LidarScan(translat) = translation {
                        frame.append(&mut Vec::from(id.to_be_bytes()));
                        frame.append(&mut lidar_msg.translate_to_frame());
                        println!("{:?}", frame);
                    }
                        
                }
            }
        }
        return frame;
    }
}