use std::f32::consts::PI;


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

pub fn modulo_2pi(x: f32) -> f32 {
    return x.rem_euclid(PI*2.0);
}

pub fn modulo_pi(x: f32) -> f32 {
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

pub fn euclidean_distance(pt1: Vec<f32>, pt2: Vec<f32>) -> f32 {
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