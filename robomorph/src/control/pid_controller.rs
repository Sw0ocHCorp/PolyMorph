//! Scalar textbook PID controller.
//!
//! `output = p * e + i * clamp(integral(e dt)) + d * (e - e_prev) / dt`, with a dead band that
//! resets the state when the error stays small. Where it sits in the stack:
//! * NOT used by the attitude loop, which is a state feedback (P on the angle error + P on the
//!   measured gyro rate, no integrator);
//! * intended for the per-motor servoing (a `PIDController` is held by every `MotorController`
//!   and its gains travel as `motor_messages::PIDConfig`) and for the future velocity loop, where
//!   the only integrator of the cascade lives.

/// Scalar PID with derivative-on-error, clamped integrator and dead band. Stateful
/// (`error_accumulator`, `prev_error`): one instance per controlled quantity.
///
/// Units: the error is in the unit of the servoed quantity (e.g. rad for a joint), `dt` in
/// seconds, and the output is in whatever unit the gains map it to.
#[derive(Debug, Clone, Copy)]
pub struct PIDController {
    /// Proportional gain.
    p: f64,
    /// Integral gain.
    i: f64, 
    /// Derivative gain.
    d: f64,
    /// Integral of the error (error-unit * s), clamped to `+/- max_error_accum`.
    error_accumulator: f64,
    /// Error of the previous call, for the numerical derivative and the dead-band test.
    prev_error: f64,
    /// Dead band: when `|error|` and `|prev_error|` are both below it, the state is reset and the
    /// output is zero. `0.0` disables it.
    min_correction_error: f64,
    /// Clamp of `error_accumulator` (crude anti-windup). `f64::INFINITY` disables it.
    max_error_accum: f64,
}

impl PIDController {
    /// Pure proportional controller with unit gain, no dead band, no integrator clamp.
    pub fn new_default() -> Self {
        return Self { p: 1.0, i: 0.0, d: 0.0, 
                        error_accumulator: 0.0, prev_error: 0.0, 
                        min_correction_error: 0.0, max_error_accum: f64::INFINITY 
                    };
    }

    /// Build with explicit gains, dead band and integrator clamp; the state starts at zero.
    pub fn new(p: f64, i: f64, d: f64, min_correction_error: f64, max_error_accum: f64) -> Self {
        return Self { p, i, d, 
                        error_accumulator: 0.0, prev_error: 0.0, 
                        min_correction_error, max_error_accum };
    }

    /// One control step from an already computed error (`setpoint - measurement`), `dt` in seconds.
    ///
    /// * Dead band: if both the current and the previous `|error|` are below
    ///   `min_correction_error`, the state is reset and `0.0` is returned.
    /// * Otherwise the integral is advanced by `error * dt` and clamped, and
    ///   `p * error + i * integral + d * (error - prev_error) / dt` is returned.
    ///
    /// The derivative is taken on the ERROR with a first-order difference: it is noisy on a noisy
    /// measurement and produces a kick when the setpoint steps.
    // NOTE: with `i == 0` the accumulator still integrates (and clamps) internally: harmless for the
    // output but stateful, so a later `set_params` with `i != 0` starts from the accumulated history.
    // NOTE: `dt == 0` divides by zero in the derivative term (inf / NaN output).
    pub fn compute_output_from_error(&mut self, error: f64, dt: f64) -> f64 {
        let mut output= 0.0;
        if error.abs() < self.min_correction_error && self.prev_error.abs() < self.min_correction_error {
            self.reset();
        } else {
            self.error_accumulator= f64::clamp(self.error_accumulator + error * dt, -self.max_error_accum, self.max_error_accum);
            let p_val= error * self.p;
            let i_val= self.error_accumulator * self.i;
            // after a reset `prev_error` is 0, so the first derivative is taken against zero rather than
            // the real previous error
            let d_val= ((error - self.prev_error) * self.d) / dt;
            output= p_val + i_val + d_val;
            self.prev_error= error;
        }
        return output;
    }

    /// One control step from a measurement and a setpoint (`error = setpoint - current`), `dt` in s.
    pub fn compute_output(&mut self, current_value: f64, setpoint_value: f64, dt: f64) -> f64 {
        return self.compute_output_from_error(setpoint_value - current_value, dt);
    }

    /// Clear the state (integral and previous error). Gains are kept.
    pub fn reset(&mut self) {
        self.prev_error= 0.0;
        self.error_accumulator= 0.0;
    }

    /// Replace the gains, dead band and clamp. The state is NOT reset (see the NOTE on
    /// `compute_output_from_error`).
    pub fn set_params(&mut self, p: f64, i: f64, d: f64, min_correction_error: f64, max_error_accum: f64) {
        self.p= p;
        self.i= i;
        self.d= d;
        self.min_correction_error= min_correction_error;
        self.max_error_accum= max_error_accum;
    }

    /// Proportional gain.
    pub fn get_p(&self) -> f64 {
        return self.p;
    }

    /// Integral gain.
    pub fn get_i(&self) -> f64 {
        return self.i;
    }

    /// Derivative gain.
    pub fn get_d(&self) -> f64 {
        return self.d;
    }

    /// Integrator clamp (`max_error_accum`).
    pub fn get_max_error_accumulator(&self) -> f64 {
        return self.max_error_accum;
    }

    /// Dead band (`min_correction_error`).
    pub fn get_min_correction_error(&self) -> f64 {
        return self.min_correction_error;
    }
}