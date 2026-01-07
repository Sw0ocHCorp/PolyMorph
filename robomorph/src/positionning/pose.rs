use crate::core::{messages::{DataChunk, Translatable}, utils};


pub const IMU_ACCEL: u16= 0x000a;
pub const IMU_GYRO: u16= 0x000b;
pub const IMU_MAGNETIC_FIELD: u16= 0x000c;

#[derive(Debug, Clone)]
pub struct IMUData {
    pub accel:  [f32; 3],
    pub gyro: [f32; 3],
    pub magnetic_field: [f32; 3],
}

impl IMUData {
    pub fn new() -> Self {
        return Self { accel: [0.0, 0.0, 0.0], gyro: [0.0, 0.0, 0.0], magnetic_field: [0.0, 0.0, 0.0] };
    }
}

impl Translatable for IMUData {
    fn fill_from_bytes(&mut self, bytes: Vec<u8>) -> usize {
        let mut buffer= Vec::new();
        let mut id= 0;
        let mut i= 0;
        let mut remain_values= 3;
        while i < bytes.len() {
            buffer.push(bytes[i]);
            if id == IMU_ACCEL {
                for j in 0..3 {
                    if let Ok(arr) = bytes[i..i+4].try_into() {
                        self.accel[j]= f32::from_be_bytes(arr);
                        i+= 4;
                    }
                }
                id = 0;
                buffer.clear();
            } else if id == IMU_GYRO {
                for j in 0..3 {
                    if let Ok(arr) = bytes[i..i+4].try_into() {
                        self.gyro[j]= f32::from_be_bytes(arr);
                        i+= 4;
                    }
                }
                id = 0;
                buffer.clear();
            } else if id == IMU_MAGNETIC_FIELD {
                for j in 0..3 {
                    if let Ok(arr) = bytes[i..i+4].try_into() {
                        self.magnetic_field[j]= f32::from_be_bytes(arr);
                        i+= 4;
                    }
                }
                id = 0;
                buffer.clear();
            } else {
                if utils::contain_bytes(buffer.clone(), IMU_ACCEL.to_be_bytes().to_vec()) >= 0{
                    id= IMU_ACCEL;
                    buffer.clear();
                }
                if utils::contain_bytes(buffer.clone(), IMU_GYRO.to_be_bytes().to_vec()) >= 0{
                    id= IMU_GYRO;
                    buffer.clear();
                }
                if utils::contain_bytes(buffer.clone(), IMU_MAGNETIC_FIELD.to_be_bytes().to_vec()) >= 0{
                    id= IMU_MAGNETIC_FIELD;
                    buffer.clear();
                }
                i+= 1;
            }
        }
        return i;
    }

    fn to_bytes(&mut self) -> Vec<u8> {
        let mut frame= (DataChunk::InternalPerceptionChunk as u16).to_be_bytes().to_vec();
        frame.append(&mut IMU_ACCEL.to_be_bytes().to_vec());
        for i in 0..3 {
            frame.append(&mut f32::to_be_bytes(self.accel[i]).to_vec());
        }
        frame.append(&mut IMU_GYRO.to_be_bytes().to_vec());
        for i in 0..3 {
            frame.append(&mut f32::to_be_bytes(self.gyro[i]).to_vec());
        }
        frame.append(&mut IMU_MAGNETIC_FIELD.to_be_bytes().to_vec());
        for i in 0..3 {
            frame.append(&mut f32::to_be_bytes(self.magnetic_field[i]).to_vec());
        }
        return frame;
    }
}

pub const POSE_LOCATION: u16= 0x000d;
pub const POSE_ORIENTATION: u16= 0x000e;
pub const POSE_LINEAR_VELOCITY: u16= 0x000f;
pub const POSE_ANGULAR_VELOCITY: u16= 0x001a;

#[derive(Debug, Clone)]
pub struct Pose {
    location: [f32; 3],
    orientation: [f32; 3],
    linear_velocity: [f32; 3],
    angular_velocity: [f32; 3], 
}

impl Pose {
    pub fn new(location: [f32; 3], orientation: [f32; 3],
                    linear_velocity: [f32; 3], angular_velocity: [f32; 3]) -> Self{
        return Self { location, orientation, linear_velocity, angular_velocity };
    }
}

impl Translatable for Pose {
    fn fill_from_bytes(&mut self, bytes: Vec<u8>) -> usize {
        let mut buffer= Vec::new();
        let mut id= 0;
        let mut i= 0;
        let mut remain_values= 3;
        while i < bytes.len() {
            buffer.push(bytes[i]);
            if id == POSE_LOCATION {
                for j in 0..3 {
                    if let Ok(arr) = bytes[i..i+4].try_into() {
                        self.location[j]= f32::from_be_bytes(arr);
                        i+= 4;
                    }
                }
                id = 0;
                buffer.clear();
            } else if id == POSE_ORIENTATION {
                for j in 0..3 {
                    if let Ok(arr) = bytes[i..i+4].try_into() {
                        self.orientation[j]= f32::from_be_bytes(arr);
                        i+= 4;
                    }
                }
                id = 0;
                buffer.clear();
            } else if id == POSE_LINEAR_VELOCITY {
                for j in 0..3 {
                    if let Ok(arr) = bytes[i..i+4].try_into() {
                        self.linear_velocity[j]= f32::from_be_bytes(arr);
                        i+= 4;
                    }
                }
                id = 0;
                buffer.clear();
            } else if id == POSE_ANGULAR_VELOCITY {
                for j in 0..3 {
                    if let Ok(arr) = bytes[i..i+4].try_into() {
                        self.angular_velocity[j]= f32::from_be_bytes(arr);
                        i+= 4;
                    }
                }
                id = 0;
                buffer.clear();
            } else {
                if utils::contain_bytes(buffer.clone(), IMU_ACCEL.to_be_bytes().to_vec()) >= 0{
                    id= IMU_ACCEL;
                    buffer.clear();
                }
                if utils::contain_bytes(buffer.clone(), IMU_GYRO.to_be_bytes().to_vec()) >= 0{
                    id= IMU_GYRO;
                    buffer.clear();
                }
                if utils::contain_bytes(buffer.clone(), IMU_MAGNETIC_FIELD.to_be_bytes().to_vec()) >= 0{
                    id= IMU_MAGNETIC_FIELD;
                    buffer.clear();
                }
                i+= 1;
            }
        }
        return 0;
    }

    fn to_bytes(&mut self) -> Vec<u8> {
        let mut frame= (DataChunk::InternalPerceptionChunk as u16).to_be_bytes().to_vec();
        frame.append(&mut POSE_LOCATION.to_be_bytes().to_vec());
        for i in 0..3 {
            frame.append(&mut f32::to_be_bytes(self.location[i]).to_vec());
        }
        frame.append(&mut POSE_ORIENTATION.to_be_bytes().to_vec());
        for i in 0..3 {
            frame.append(&mut f32::to_be_bytes(self.orientation[i]).to_vec());
        }
        frame.append(&mut POSE_LINEAR_VELOCITY.to_be_bytes().to_vec());
        for i in 0..3 {
            frame.append(&mut f32::to_be_bytes(self.linear_velocity[i]).to_vec());
        }
        frame.append(&mut POSE_ANGULAR_VELOCITY.to_be_bytes().to_vec());
        for i in 0..3 {
            frame.append(&mut f32::to_be_bytes(self.angular_velocity[i]).to_vec());
        }
        return frame;
    }
}