#[derive(Debug)]
pub struct PIDController {
    p: f64,
    i: f64, 
    d: f64,
    error_accumulator: f64,
    prev_error: f64,
    min_correction_error: f64,
    max_error_accum: f64,
}

impl PIDController {
    pub fn new_default() -> Self {
        return Self { p: 1.0, i: 0.0, d: 0.0, 
                        error_accumulator: 0.0, prev_error: 0.0, 
                        min_correction_error: 0.0, max_error_accum: f64::INFINITY 
                    };
    }

    pub fn new(p: f64, i: f64, d: f64, min_correction_error: f64, max_error_accum: f64) -> Self {
        return Self { p, i, d, 
                        error_accumulator: 0.0, prev_error: 0.0, 
                        min_correction_error, max_error_accum };
    }

    pub fn compute_output_from_error(&mut self, error: f64, dt: f64) -> f64 {
        let mut output= 0.0;
        if error.abs() < self.min_correction_error && self.prev_error.abs() < self.min_correction_error {
            self.reset();
        } else {
            self.error_accumulator= f64::clamp(self.error_accumulator + error * dt, -self.max_error_accum, self.max_error_accum);
            let p_val= error * self.p;
            let i_val= self.error_accumulator * self.i;
            let d_val= ((error - self.prev_error) * self.d) / dt;
            output= p_val + i_val + d_val;
            self.prev_error= error;
        }
        return output;
    }

    pub fn compute_output(&mut self, current_value: f64, setpoint_value: f64, dt: f64) -> f64 {
        return self.compute_output_from_error(setpoint_value - current_value, dt);
    }

    pub fn reset(&mut self) {
        self.prev_error= 0.0;
        self.error_accumulator= 0.0;
    }

    pub fn set_params(&mut self, p: f64, i: f64, d: f64, min_correction_error: f64, max_error_accum: f64) {
        self.p= p;
        self.i= i;
        self.d= d;
        self.min_correction_error= min_correction_error;
        self.max_error_accum= max_error_accum;
    }

    pub fn get_p(&self) -> f64 {
        return self.p;
    }

    pub fn get_i(&self) -> f64 {
        return self.i;
    }

    pub fn get_d(&self) -> f64 {
        return self.d;
    }

    pub fn get_max_error_accumulator(&self) -> f64 {
        return self.max_error_accum;
    }

    pub fn get_min_correction_error(&self) -> f64 {
        return self.min_correction_error;
    }
}