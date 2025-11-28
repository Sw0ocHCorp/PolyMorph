pub fn normalize_angle(rad_angle: f32) -> f32 {
    let t1= (rad_angle + 180_f32.to_radians());
    let t2= (360_f32.to_radians());
    let mut zero2pi_angle= (rad_angle + 180_f32.to_radians()) % (360_f32.to_radians());
    if zero2pi_angle < 0.0 {
        zero2pi_angle += 360_f32.to_radians();
    }
    return zero2pi_angle - 180_f32.to_radians();
}