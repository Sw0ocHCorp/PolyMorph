use std::f64::consts::PI;

use faer::{Col, col};

use crate::positionning::pose::GPSData;

const LAT_TO_METER: f64= 111.320;


pub fn contain_bytes(src_bytes: Vec<u8>, target_bytes: Vec<u8>) -> i128{
    let mut start_index= -1;
    if src_bytes.len() >= target_bytes.len() {
        for i in 0..src_bytes.len() - (target_bytes.len() - 1) {
            if src_bytes[i] == target_bytes[0] {
                start_index= i as i128;
                for j in 0..target_bytes.len() {
                    if src_bytes[i+j] != target_bytes[j] {
                        start_index= -1;
                    }
                }
            }
        }
    }
    return start_index;
}

pub fn modulo_2pi(x: f64) -> f64 {
    return x.rem_euclid(PI*2.0);
}

pub fn modulo_pi(x: f64) -> f64 {
    // Use rem_euclid to do a mod in [0, 2π)
    let m = modulo_2pi(x);

    // Shift range from [0, 2π) to [-π, π)
    if m >= PI {
        m - PI*2.0
    } else {
        m
    }
}

pub fn modulo_2pi_f64(x: f64) -> f64 {
    return x.rem_euclid(std::f64::consts::PI*2.0);
}

pub fn modulo_pi_f64(x: f64) -> f64 {
    // Use rem_euclid to do a mod in [0, 2π)
    let m = modulo_2pi_f64(x);

    // Shift range from [0, 2π) to [-π, π)
    if m >= std::f64::consts::PI {
        m - std::f64::consts::PI*2.0
    } else {
        m
    }
}

pub fn euclidean_distance(pt1: Vec<f64>, pt2: Vec<f64>) -> f64 {
    if pt1.len() == pt2.len() {
        let mut cumul= 0.0;
        for i in 0..pt1.len() {
            cumul += (pt2[i] - pt1[i]).powf(2.0);
        }
        return cumul.sqrt();
    } else {
        println!("Points must have the same dimension");
        return -1.0;
    }

}

pub fn compute_cross_product(a: Vec<f64>, b: Vec<f64>) -> Vec<f64> {
    if a.len() == 3 && b.len() == 3 {
        return vec![
                        a[1] * b[2] - a[2] * b[1],
                        a[2] * b[0] - a[0] * b[2],
                        a[0] * b[1] - a[1] * b[0],
                    ];
    } else {
        return vec![0.0];
    }
}

pub fn local_to_global_frame(origin_gps_data: GPSData, x_pose: f64, y_pose: f64) -> GPSData{
    let d_lon= x_pose / (LAT_TO_METER*origin_gps_data.latitude.to_radians().cos());
    let d_lat= y_pose / LAT_TO_METER;
    return GPSData { longitude: origin_gps_data.longitude + d_lon, latitude: origin_gps_data.latitude + d_lat };
}

pub fn global_to_local_frame(origin_deg_lon: f64, origin_deg_lat: f64, deg_lon: f64, deg_lat: f64) -> [f64; 2] {
    let d_lon= (deg_lon - origin_deg_lon) * (LAT_TO_METER*origin_deg_lat.to_radians().cos());
    let d_lat= (deg_lat - origin_deg_lat) * LAT_TO_METER;
    return [d_lon, d_lat];
}