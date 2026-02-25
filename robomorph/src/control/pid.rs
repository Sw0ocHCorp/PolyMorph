pub struct PIDController {
    p: f64,
    i: f64,
    d: f64,
    previous_error: f64,
    integral_error: f64,
    max_integral_error: f64,
    pub min_error_threshold: f64,
    leak_factor: f64
}

impl PIDController {
    pub fn new(p: f64, i: f64, d: f64, max_integral_error: f64, min_error_threshold: f64, leak_factor: f64) -> Self {
        return Self{p, i, d, 
                        previous_error: 0.0, integral_error: 0.0, 
                        max_integral_error, min_error_threshold, leak_factor};
    }

    pub fn compute_output_value(&mut self, setpoint_value: f64, measurement: f64, dt: f64) -> f64 {
        return self.compute_output_value_from_error(setpoint_value - measurement, dt);
    }

    pub fn compute_output_value_from_error(&mut self, error: f64, dt: f64) -> f64 {
        if error.abs() >= self.min_error_threshold {
            self.integral_error= f64::clamp((self.integral_error + error*dt)*self.leak_factor, -self.max_integral_error, self.max_integral_error);
            let p= self.p*error;
            let i= self.i*self.integral_error;
            let d= self.d*(error - self.previous_error);
            let output= p + i + d;
            //println!("P= {} | I= {} | D= {}", p, i, d);
            self.previous_error = error;
            return output;
        } else {
            return 0.0;
        }
        
    }

    pub fn get_min_error_threshold(&self) -> f64 {
        return self.min_error_threshold;
    }

    pub fn get_integral_error(&self) -> f64 {
        return self.integral_error;
    }
}

pub fn copy(pid: &PIDController) -> PIDController {
        return PIDController::new(pid.p, pid.i, pid.d, 
            pid.max_integral_error, pid.min_error_threshold, pid.leak_factor
        );
    }