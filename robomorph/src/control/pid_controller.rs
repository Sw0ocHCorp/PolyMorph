//! Scalar textbook PID controller.
//!
//! `output = p * e + i * clamp(integral(e dt)) + d * (e - e_prev) / dt`, with a dead band that
//! resets the state when the error stays small. Where it sits in the stack:
//! * NOT used by the attitude loop, which is a state feedback (P on the angle error + P on the
//!   measured gyro rate, no integrator);
//! * intended for the per-motor servoing (a `PIDController` is held by every `MotorController`
//!   and its gains travel as `motor_messages::PIDConfig`) and for the future velocity loop, where
//!   the only integrator of the cascade lives.

use std::{fmt::Debug, marker::PhantomData, ops::Add, time::Duration};
pub trait PIDValue: Copy + Default + Debug + Add<Output = Self> {
    fn error_from(&self, setpoint: Self) -> Self;  // useful to compute the error for the PID
    fn compute_p(&self, setpoint: Self, k_p: f64) -> Self;   // useful to compute the error for the P term of the PID
    fn compute_i(&mut self, error: Self, max_accum_error: f64, k_i: f64, dt: Duration) -> Self;  // useful to compute the I term of the PID
    fn compute_d(&self, setpoint: Self, prev_error: Self, k_d: f64, dt: Duration) -> Self;  // useful to compute the D term of the PID
}

/// Scalar PID with derivative-on-error, clamped integrator and dead band. Stateful
/// (`error_accumulator`, `prev_error`): one instance per controlled quantity.
///
/// Units: the error is in the unit of the servoed quantity (e.g. rad for a joint), `dt` in
/// seconds, and the output is in whatever unit the gains map it to.
#[derive(Debug, Clone, Copy)]
pub struct PIDController<T: PIDValue> {
    /// Proportional gain.
    p: f64,
    /// Integral gain.
    i: f64, 
    /// Derivative gain.
    d: f64,
    /// Integral of the error (error-unit * s), clamped to `+/- max_error_accum`.
    error_accumulator: T,
    /// Error of the previous call, for the numerical derivative and the dead-band test.
    prev_error: T,
    /// Dead band: when `|error|` and `|prev_error|` are both below it, the state is reset and the
    /// output is zero. `0.0` disables it.
    min_correction_error: f64,
    /// Clamp of `error_accumulator` (crude anti-windup). `f64::INFINITY` disables it.
    max_error_accum: f64,
}

impl<T: PIDValue> PIDController<T> {
    /// Pure proportional controller with unit gain, no dead band, no integrator clamp.
    pub fn new_default() -> Self {
        return Self { p: 1.0, i: 0.0, d: 0.0, 
                        error_accumulator: T::default(), prev_error: T::default(), 
                        min_correction_error: 0.0, max_error_accum: f64::INFINITY 
                    };
    }

    /// Build with explicit gains, dead band and integrator clamp; the state starts at zero.
    pub fn new(p: f64, i: f64, d: f64, min_correction_error: f64, max_error_accum: f64) -> Self {
        return Self { p, i, d,
                        error_accumulator: T::default(), prev_error: T::default(),
                        min_correction_error, max_error_accum};
    }

    /// One control step from a measurement and a setpoint (`error = setpoint - current`), `dt` in s.
    pub fn compute_output(&mut self, current_value: T, setpoint_value: T, dt: Duration) -> T {
        let p= current_value.compute_p(setpoint_value, self.p);
        
        let i= self.error_accumulator.compute_i(current_value.error_from(setpoint_value), 
                                                                self.max_error_accum, self.i, dt);
        
        let d= current_value.compute_d(setpoint_value, self.prev_error, self.d, dt);
        println!("P: {:?} | I: {:?} | D: {:?}", p, i, d);
        let output_value=  p + i + d;
        self.prev_error= current_value.error_from(setpoint_value);
        return output_value;
    }

    /// Clear the state (integral and previous error). Gains are kept.
    pub fn reset(&mut self) {
        self.prev_error= T::default();
        self.error_accumulator= T::default();
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