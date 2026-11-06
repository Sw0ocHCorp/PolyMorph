//! Textbook PID controller, generic over the type of the controlled value (`PIDValue`: `Vec3` for
//! the velocity loop, one PID per component).
//!
//! `output = p * e + i * clamp(integral(e dt)) + d * (e - e_prev) / dt`, with a dead band that
//! resets the state when the error stays small. Where it sits in the stack:
//! * NOT used by the attitude loop, which is a state feedback (P on the angle error + P on the
//!   measured gyro rate, no integrator);
//! * used by the velocity loop, where the only integrator of the cascade lives.
//!
//! A time step `dt` of zero is legitimate (a stalled simulator delivers the same pose twice): it
//! brings no new information, so nothing is integrated and the D term is zero, never a division by
//! zero.

use std::{fmt::Debug, ops::Add, time::Duration};
pub trait PIDValue: Copy + Default + Debug + Add<Output = Self> {
    fn error_from(&self, setpoint: Self) -> Self;  // useful to compute the error for the PID
    fn compute_p(&self, setpoint: Self, k_p: f64) -> Self;   // useful to compute the error for the P term of the PID
    fn compute_i(&mut self, error: Self, max_accum_error: f64, k_i: f64, dt: Duration) -> Self;  // useful to compute the I term of the PID
    fn compute_d(&self, setpoint: Self, prev_error: Self, k_d: f64, dt: Duration) -> Self;  // useful to compute the D term of the PID
    fn magnitude(&self) -> f64;  // size of an error, compared with the dead band
}

/// PID with derivative-on-error, clamped integrator and dead band. Stateful
/// (`error_accumulator`, `prev_error`): one instance per controlled quantity.
///
/// Units: the error is in the unit of the servoed quantity (e.g. m/s for the velocity loop), `dt`
/// in seconds, and the output is in whatever unit the gains map it to.
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
    /// Error of the previous call, for the numerical derivative and the dead-band test. `None` before
    /// the first call and after a reset: no derivative can be taken yet.
    prev_error: Option<T>,
    /// Dead band: when `|error|` and `|prev_error|` are both below it, the state is reset and the
    /// output is zero. `0.0` disables it.
    min_correction_error: f64,
    /// Clamp of `error_accumulator` (crude anti-windup). `f64::INFINITY` disables it. Must be
    /// positive: a negative bound or a NaN makes the clamp panic.
    max_error_accum: f64,
}

impl<T: PIDValue> PIDController<T> {
    /// Pure proportional controller with unit gain, no dead band, no integrator clamp.
    pub fn new_default() -> Self {
        return Self { p: 1.0, i: 0.0, d: 0.0, 
                        error_accumulator: T::default(), prev_error: None, 
                        min_correction_error: 0.0, max_error_accum: f64::INFINITY 
                    };
    }

    /// Build with explicit gains, dead band and integrator clamp; the state starts at zero.
    pub fn new(p: f64, i: f64, d: f64, min_correction_error: f64, max_error_accum: f64) -> Self {
        return Self { p, i, d,
                        error_accumulator: T::default(), prev_error: None,
                        min_correction_error, max_error_accum};
    }

    /// One control step from a measurement and a setpoint (`error = setpoint - current`), over `dt`:
    ///   1. the error;
    ///   2. inside the dead band twice in a row: the integral is reset and the output is zero;
    ///   3. P on the error;
    ///   4. I: the integral grows only if `integrate` (false = frozen, for an anti-windup), with a
    ///      gain, over the time step; otherwise it is only read (integrating over zero seconds adds
    ///      nothing);
    ///   5. D: only with a previous error, a gain and a time step that is not zero, otherwise zero
    ///      (no kick on the first call, no division by zero);
    ///   6. the error is kept for the next call.
    pub fn compute_output(&mut self, current_value: T, setpoint_value: T, dt: Duration, integrate: bool) -> T {
        // 1. setpoint - current
        let error= current_value.error_from(setpoint_value);
        // 2. a small error, twice in a row: nothing to correct
        if self.min_correction_error > 0.0 && error.magnitude() < self.min_correction_error
                && self.prev_error.is_some_and(|prev| prev.magnitude() < self.min_correction_error) {
            self.error_accumulator= T::default();
            self.prev_error= Some(error);
            return T::default();
        }
        // 3. proportional
        let p= current_value.compute_p(setpoint_value, self.p);
        // 4. integral, accumulated over the step only when it should grow
        let step= if integrate && self.i != 0.0 { dt } else { Duration::ZERO };
        let i= self.error_accumulator.compute_i(error, self.max_error_accum, self.i, step);
        // 5. derivative of the error
        let d= match self.prev_error {
            Some(prev_error) if self.d != 0.0 && !dt.is_zero() => current_value.compute_d(setpoint_value, prev_error, self.d, dt),
            _ => T::default(),
        };
        // 6. for the next derivative and dead-band test
        self.prev_error= Some(error);
        return p + i + d;
    }

    /// The integral of the error so far (error-unit * s), clamped.
    pub fn integral(&self) -> T {
        return self.error_accumulator;
    }

    /// Clear the state (integral and previous error). Gains are kept.
    pub fn reset(&mut self) {
        self.prev_error= None;
        self.error_accumulator= T::default();
    }

    /// Replace the gains, dead band and clamp. The state is NOT reset: call `reset` for that.
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

/// Hand-predicted values on `Vec3`, the type of the velocity loop. The error is always
/// `setpoint - current`, component by component.
#[cfg(test)]
mod tests {
    use crate::messages::registered_message::Vec3;

    use super::*;

    const DT: Duration= Duration::from_millis(10);

    fn close(a: Vec3, b: Vec3) -> bool {
        return (a.0 - b.0).norm() < 1e-12;
    }

    #[test]
    fn the_p_term_is_the_gain_times_the_error_component_by_component() {
        // setpoint - current = (1, -2, 0.5), p = 2: (2, -4, 1)
        let mut pid= PIDController::<Vec3>::new(2.0, 0.0, 0.0, 0.0, f64::INFINITY);
        assert_eq!(pid.compute_output(Vec3::new(0.0, 2.0, 0.0), Vec3::new(1.0, 0.0, 0.5), DT, true), Vec3::new(2.0, -4.0, 1.0));
    }

    #[test]
    fn the_i_term_accumulates_the_error_over_time_within_its_bound() {
        // 1. a constant error (1, 0, 0), i = 3, steps of 10 ms: integral 0.01 then 0.02, I = 0.03 then 0.06
        let mut pid= PIDController::<Vec3>::new(0.0, 3.0, 0.0, 0.0, f64::INFINITY);
        let (current, setpoint)= (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
        assert!(close(pid.compute_output(current, setpoint, DT, true), Vec3::new(0.03, 0.0, 0.0)));
        assert!(close(pid.compute_output(current, setpoint, DT, true), Vec3::new(0.06, 0.0, 0.0)));
        assert!(close(pid.integral(), Vec3::new(0.02, 0.0, 0.0)));
        // 2. bounded at 0.015: the second step stops at the bound
        let mut bounded= PIDController::<Vec3>::new(0.0, 3.0, 0.0, 0.0, 0.015);
        bounded.compute_output(current, setpoint, DT, true);
        bounded.compute_output(current, setpoint, DT, true);
        assert!(close(bounded.integral(), Vec3::new(0.015, 0.0, 0.0)));
    }

    #[test]
    fn the_d_term_has_no_kick_on_its_first_call() {
        // d = 0.5: no previous error on the first call, D = 0; then the error goes from 1 to 2 in
        // 10 ms, a derivative of 100 /s, D = 50
        let mut pid= PIDController::<Vec3>::new(0.0, 0.0, 0.5, 0.0, f64::INFINITY);
        let current= Vec3::new(0.0, 0.0, 0.0);
        assert_eq!(pid.compute_output(current, Vec3::new(1.0, 0.0, 0.0), DT, true), Vec3::new(0.0, 0.0, 0.0));
        assert!(close(pid.compute_output(current, Vec3::new(2.0, 0.0, 0.0), DT, true), Vec3::new(50.0, 0.0, 0.0)));
        // after a reset, no kick either
        pid.reset();
        assert_eq!(pid.compute_output(current, Vec3::new(5.0, 0.0, 0.0), DT, true), Vec3::new(0.0, 0.0, 0.0));
    }

    #[test]
    fn a_zero_time_step_changes_nothing_and_gives_no_nan() {
        // a stalled simulator: the same pose twice, so dt = 0. The D term would divide by 0: it is 0;
        // the integral does not move; the output is the P term plus the I term of the integral so far
        let mut pid= PIDController::<Vec3>::new(1.0, 1.0, 1.0, 0.0, f64::INFINITY);
        let (current, setpoint)= (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
        pid.compute_output(current, setpoint, DT, true);
        let output= pid.compute_output(current, setpoint, Duration::ZERO, true);
        assert!(output.iter().all(|c| c.is_finite()), "{:?}", output);
        assert!(close(output, Vec3::new(1.0 + 0.01, 0.0, 0.0)), "{:?}", output);
        assert!(close(pid.integral(), Vec3::new(0.01, 0.0, 0.0)));
    }

    #[test]
    fn the_integral_only_grows_with_a_gain_and_when_asked() {
        let (current, setpoint)= (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
        // 1. i = 0: nothing is accumulated, so no stale integral can come back after set_params
        let mut no_gain= PIDController::<Vec3>::new(1.0, 0.0, 0.0, 0.0, f64::INFINITY);
        no_gain.compute_output(current, setpoint, DT, true);
        assert_eq!(no_gain.integral(), Vec3::new(0.0, 0.0, 0.0));
        // 2. integrate = false (anti-windup): the integral is frozen, its term still applies
        let mut pid= PIDController::<Vec3>::new(0.0, 1.0, 0.0, 0.0, f64::INFINITY);
        pid.compute_output(current, setpoint, DT, true);
        let frozen= pid.compute_output(current, setpoint, DT, false);
        assert!(close(pid.integral(), Vec3::new(0.01, 0.0, 0.0)));
        assert!(close(frozen, Vec3::new(0.01, 0.0, 0.0)));
    }

    #[test]
    fn the_dead_band_resets_the_state_and_outputs_zero() {
        // band of 0.1: two errors of 0.05 in a row. The first one (no previous error yet) still acts
        // and integrates; the second one is inside the band twice over: zero, integral reset
        let mut pid= PIDController::<Vec3>::new(1.0, 1.0, 0.0, 0.1, f64::INFINITY);
        let (current, setpoint)= (Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.05, 0.0, 0.0));
        assert!(close(pid.compute_output(current, setpoint, DT, true), Vec3::new(0.05 + 0.0005, 0.0, 0.0)));
        assert_eq!(pid.compute_output(current, setpoint, DT, true), Vec3::new(0.0, 0.0, 0.0));
        assert_eq!(pid.integral(), Vec3::new(0.0, 0.0, 0.0));
    }
}