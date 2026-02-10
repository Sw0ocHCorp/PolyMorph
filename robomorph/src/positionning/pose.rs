use faer::col;
use num_quaternion::{EulerAngles, Q64, Quaternion, UQ64};

use crate::{core::{messages::{DataChunk, Translatable}, utils}};


pub const IMU_ACCEL: u16= 0x000a;
pub const IMU_GYRO: u16= 0x000b;
pub const IMU_MAGNETIC_FIELD: u16= 0x000c;
pub const GNSS_LONGITUDE: u16= 0x000d;
pub const GNSS_LATITUDE: u16= 0x000e;

#[derive(Debug, Clone, Default)]
pub struct GPSData {
    pub longitude: f64,
    pub latitude: f64
}

impl Translatable for GPSData {
    fn fill_from_bytes(&mut self, bytes: Vec<u8>) -> usize {
        let mut buffer= Vec::new();
        let mut id= 0;
        let mut i= 0;
        let mut remain_values= 3;
        while i < bytes.len() {
            buffer.push(bytes[i]);
            if id == GNSS_LONGITUDE {
                if let Ok(arr) = bytes[i..i+4].try_into() {
                    self.longitude= f32::from_be_bytes(arr) as f64;
                    i+= 4;
                }
                buffer.clear();
                id= 0;
            } else if id == GNSS_LATITUDE {
                if let Ok(arr) = bytes[i..i+4].try_into() {
                    self.latitude= f32::from_be_bytes(arr) as f64;
                    i+= 4;
                }
                buffer.clear();
                id= 0;
            } else {
                if utils::contain_bytes(buffer.clone(), GNSS_LONGITUDE.to_be_bytes().to_vec()) >= 0{
                    id= GNSS_LONGITUDE;
                } else if utils::contain_bytes(buffer.clone(), GNSS_LATITUDE.to_be_bytes().to_vec()) >= 0{
                    id= GNSS_LATITUDE;
                }
            }
            
            i+=1;
        }
        return i;
    }

    fn to_bytes(&mut self) -> Vec<u8> {
        let mut frame= (DataChunk::GnssPosChunk as u16).to_be_bytes().to_vec();
        frame.append(&mut GNSS_LONGITUDE.to_be_bytes().to_vec());
        frame.append(&mut f32::to_be_bytes(self.longitude as f32).to_vec());
        frame.append(&mut GNSS_LATITUDE.to_be_bytes().to_vec());
        frame.append(&mut f32::to_be_bytes(self.latitude as f32).to_vec());
        return frame;
    }
}

#[derive(Debug, Clone)]
pub struct IMUData {
    pub accel:  [f64; 3],
    pub gyro: [f64; 3],
    pub magnetic_field: [f64; 3],
}

impl IMUData {
    pub fn new() -> Self {
        return Self { accel: [0.0, 0.0, 0.0], gyro: [0.0, 0.0, 0.0], magnetic_field: [0.0, 0.0, 0.0]};
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
                    if i+4 <= bytes.len()-1 && let Ok(arr) = bytes[i..i+4].try_into() {
                        self.accel[j]= f32::from_be_bytes(arr) as f64;
                        i+= 4;
                    }
                }
                id = 0;
                buffer.clear();
            } else if id == IMU_GYRO {
                for j in 0..3 {
                    if i+4 <= bytes.len()-1 && let Ok(arr) = bytes[i..i+4].try_into() {
                        self.gyro[j]= f32::from_be_bytes(arr) as f64;
                        i+= 4;
                    }
                }
                id = 0;
                buffer.clear();
            } else if id == IMU_MAGNETIC_FIELD {
                for j in 0..3 {
                    if i+4 <= bytes.len()-1 && let Ok(arr) = bytes[i..i+4].try_into() {
                        self.magnetic_field[j]= f32::from_be_bytes(arr) as f64;
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
            frame.append(&mut f32::to_be_bytes(self.accel[i] as f32).to_vec());
        }
        frame.append(&mut IMU_GYRO.to_be_bytes().to_vec());
        for i in 0..3 {
            frame.append(&mut f32::to_be_bytes(self.gyro[i] as f32).to_vec());
        }
        frame.append(&mut IMU_MAGNETIC_FIELD.to_be_bytes().to_vec());
        for i in 0..3 {
            frame.append(&mut f64::to_be_bytes(self.magnetic_field[i] as f64).to_vec());
        }
        return frame;
    }
}

pub const POSE_LOCATION: u16= 0x000d;
pub const POSE_ORIENTATION: u16= 0x000e;
pub const POSE_LINEAR_VELOCITY: u16= 0x000f;
pub const POSE_ANGULAR_VELOCITY: u16= 0x001a;

#[derive(Debug, Clone, Default)]
pub struct Pose {
    absolute_location: GPSData,
    location: [f64; 3],
    orientation: UQ64,
    linear_velocity: [f64; 3],
    angular_velocity: [f64; 3], 
}

impl Pose {
    pub fn new(absolute_location: GPSData, location: [f64; 3], orientation: UQ64,
                    linear_velocity: [f64; 3], angular_velocity: [f64; 3]) -> Self{
        return Self {absolute_location, location, orientation, linear_velocity, angular_velocity };
    }

    pub fn get_location(&self) -> [f64; 3] {
        return self.location;
    }

    pub fn get_orientation(&self) -> UQ64 {
        return self.orientation;
    }

    pub fn get_euler_orientation(&self) -> [f64; 3] {
        let euler_angles= self.orientation.to_euler_angles();
        return [euler_angles.roll, euler_angles.pitch, euler_angles.yaw];
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
                        self.location[j]= f32::from_be_bytes(arr) as f64;
                        i+= 4;
                    }
                }
                id = 0;
                buffer.clear();
            } else if id == POSE_ORIENTATION {
                let mut raw_angles= [0.0, 0.0, 0.0];
                for j in 0..3 {
                    if let Ok(arr) = bytes[i..i+4].try_into() {
                        raw_angles[j]= f32::from_be_bytes(arr) as f64;
                        i+= 4;
                    }
                }
                self.orientation= UQ64::from_euler_angles(raw_angles[0], raw_angles[1], raw_angles[2]);
                id = 0;
                buffer.clear();
            } else if id == POSE_LINEAR_VELOCITY {
                for j in 0..3 {
                    if let Ok(arr) = bytes[i..i+4].try_into() {
                        self.linear_velocity[j]= f32::from_be_bytes(arr) as f64;
                        i+= 4;
                    }
                }
                id = 0;
                buffer.clear();
            } else if id == POSE_ANGULAR_VELOCITY {
                for j in 0..3 {
                    if let Ok(arr) = bytes[i..i+4].try_into() {
                        self.angular_velocity[j]= f32::from_be_bytes(arr) as f64;
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
            frame.append(&mut f32::to_be_bytes(self.location[i] as f32).to_vec());
        }
        frame.append(&mut POSE_ORIENTATION.to_be_bytes().to_vec());
        let euler_angles= self.orientation.to_euler_angles();
        let orientation= [euler_angles.roll, euler_angles.pitch, euler_angles.yaw];
        for i in 0..3 {
            frame.append(&mut f32::to_be_bytes(orientation[i] as f32).to_vec());
        }
        frame.append(&mut POSE_LINEAR_VELOCITY.to_be_bytes().to_vec());
        for i in 0..3 {
            frame.append(&mut f32::to_be_bytes(self.linear_velocity[i] as f32).to_vec());
        }
        frame.append(&mut POSE_ANGULAR_VELOCITY.to_be_bytes().to_vec());
        for i in 0..3 {
            frame.append(&mut f32::to_be_bytes(self.angular_velocity[i] as f32).to_vec());
        }
        return frame;
    }
}

/*impl KalmanState for Pose {
    fn as_state(&self) -> faer::Col<f64> {
        return col![self.absolute_location.latitude, self.absolute_location.longitude, 
                        self.location[0], self.location[1], self.location[2],
                        self.orientation[0], self.orientation[1], self.orientation[2],
                        self.linear_velocity[0], self.linear_velocity[1], self.linear_velocity[2],
                        self.angular_velocity[0], self.angular_velocity[1], self.angular_velocity[2]];
    }
}*/