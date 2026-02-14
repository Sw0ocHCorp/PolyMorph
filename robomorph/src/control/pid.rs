pub struct PIDController {
    p: f64,
    i: f64,
    d: f64,
    previous_error: f64,
    integral_error: f64,
    max_integral_error: f64,
    min_error_threshold: f64,
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
        self.integral_error= f64::clamp((self.integral_error + error*dt)*self.leak_factor, -self.max_integral_error, self.max_integral_error);
        /*if error < self.min_error_threshold.abs() {
            self.integral_error= 0.0;
            self.previous_error= 0.0;
        }*/
        //println!("Integral Error= {}", self.integral_error);
        let p= self.p*error;
        let i= self.i*self.integral_error;
        let d= self.d*(error - self.previous_error);
        let output= p + i + d;
        //println!("P= {} | I= {} | D= {}", p, i, d);
        self.previous_error = error;
        return output;
    }
}