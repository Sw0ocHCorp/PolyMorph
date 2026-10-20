//! Scripted scenarios of the Gazebo test bench. A scenario is a timeline of setpoints played on the
//! SIMULATED clock, together with the predictions its report judges a run against [P10].
//! Procedure, catalogue and predictions: `docs/docs/simulation/running-scenarios.mdx`.
//!
//! One file per piece:
//!   - `player`:   the `ScenarioPlayer` process, the single source of the mission setpoint;
//!   - `recorder`: the `Recorder` process, one CSV row per pass of the processes pipeline;
//!   - `report`:   reads a recording back, measures it and judges it against the predictions.
//!
//! The predictions below are computed for the tuning wired in `main.rs`: attitude tau = 0.2 s,
//! zeta = 1, processes pipeline at 100 Hz (T = 10 ms), and for the OSPRAI as identified in free flight
//! by `docs/scripts/identify_vehicle.py`. Change one of them and the predictions must be recomputed
//! with `docs/scripts/delay_budget_reference.py`. The predictions are DATA of the catalogue: the
//! checks themselves know nothing about the vehicle.

use nalgebra::{UnitQuaternion, Vector3};
use prost_types::Timestamp;
use robomorph::messages::{pose_messages::{IMUMeasurements, Pose}, registered_message::{UnitQuat, Vec3}};

pub mod player;
pub mod recorder;
pub mod report;

/// Names accepted by `by_name`, for the command-line help.
pub const NAMES: [&str; 2]= ["hover", "roll_step"];

/// One piece of a timeline: from `start_s` on, the setpoint is constant. Timelines are piecewise
/// constant (steps only), which is exactly what a step response needs.
#[derive(Clone, Debug)]
pub struct Segment {
    /// Start of the segment, in simulated seconds since the scenario started.
    pub start_s: f64,
    /// q_d, the attitude setpoint body->world. Read by the attitude stage today; it becomes the
    /// heading preference q_pref of the resolver in brick 3.
    pub orientation: UnitQuaternion<f64>,
    /// v_d, the velocity setpoint in the WORLD frame (m/s) [P7]. Read by the velocity loop (brick 5),
    /// ignored in stabilize.
    pub velocity: Vector3<f64>,
}

/// A prediction the report checks, written down BEFORE the run [P10].
#[derive(Clone, Debug)]
pub enum Check {
    /// The vehicle holds level: its tilt (angle between the body z axis and the world z axis)
    /// stays below `max_tilt_deg` between `from_s` and `to_s` (scenario time).
    HoverTilt { from_s: f64, to_s: f64, max_tilt_deg: f64 },
    /// Every roll edge of the setpoint is a step response, measured by the fraction of the step
    /// covered at tau, 2 tau and 3 tau and by its overshoot. `edges[n]` is the prediction for edge
    /// n + 1. With `tolerance: Some(t)` every fraction must be within +/- t of its prediction and the
    /// overshoot at most t above its own; with `None` the edges are shown next to their predictions
    /// for information only, and never fail.
    RollSteps { tau_s: f64, edges: Vec<EdgePrediction>, tolerance: Option<f64> },
    /// The roll measured at `at_s` (scenario time, linear interpolation) is within
    /// +/- `tolerance_deg` of `expected_deg`. Used at the end of a hold: the attitude law has no
    /// integrator, so a constant disturbing moment (the drag of a sliding vehicle) leaves a steady
    /// error that the reference model predicts.
    HoldRoll { at_s: f64, expected_deg: f64, tolerance_deg: f64 },
}

/// What the reference model predicts for one edge of a step: fraction of the step covered at tau,
/// 2 tau, 3 tau, and the overshoot (all as fractions of the step, not percents).
#[derive(Clone, Debug)]
pub struct EdgePrediction {
    pub fractions: [f64; 3],
    pub overshoot: f64,
}

/// A timeline of setpoints and the checks of its report.
#[derive(Clone, Debug)]
pub struct Scenario {
    pub name: &'static str,
    pub description: &'static str,
    /// Sorted by `start_s`, the first one starting at 0.
    pub segments: Vec<Segment>,
    /// End of the scripted part (s). `None`: the timeline is held forever.
    pub duration_s: Option<f64>,
    pub checks: Vec<Check>,
}

impl Scenario {
    /// The setpoint at scenario time `t_s`: the one of the last segment that has started. After the
    /// end of the timeline the last setpoint is held (zero-order hold, like every stage).
    pub fn setpoint_at(&self, t_s: f64) -> Pose {
        // 1. the active segment: the last one whose start has passed (the segments are sorted)
        let mut active= &self.segments[0];
        for segment in &self.segments {
            if segment.start_s <= t_s {
                active= segment;
            }
        }
        // 2. the setpoint: q_d, omega_d = 0 (a step holds a new attitude, it does not ask for a
        //    rotation rate) and v_d. The timestamp is left to the player, which knows the clock.
        let mut setpoint= Pose::default();
        setpoint.orientation= Some(UnitQuat::from(active.orientation));
        setpoint.imu_measurement= Some(IMUMeasurements { a_velocity: Some(Vec3::new(0.0, 0.0, 0.0)), ..Default::default() });
        setpoint.l_velocity= Some(Vec3(active.velocity));
        return setpoint;
    }

    /// True once the scripted part is over (never for a timeline held forever).
    pub fn is_finished(&self, t_s: f64) -> bool {
        return self.duration_s.is_some_and(|duration_s| t_s >= duration_s);
    }
}

/// The catalogue. Each prediction says where its number comes from.
pub fn by_name(name: &str) -> Option<Scenario> {
    // the two attitudes the scenarios use: level, and a 10 deg roll about the body x axis
    let level= UnitQuaternion::identity();
    let rolled= UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 10f64.to_radians());
    // the hover check: tilt below 0.5 deg once the start-up transient is over (1 s after Play).
    // Measured at rung C: +/- 0.002 deg, so 0.5 deg is a loose bound that still catches a drift.
    let hover_tilt= |to_s: f64| Check::HoverTilt { from_s: 1.0, to_s, max_tilt_deg: 0.5 };
    match name {
        // S0: hold level forever. The default scenario, i.e. the former fixed setpoint of the bridge
        //     (identity attitude), with v_d = 0 instead of the old unused (0, 0, 1).
        "hover" => Some(Scenario {
            name: "hover",
            description: "S0: hold level, forever",
            segments: vec![Segment { start_s: 0.0, orientation: level, velocity: Vector3::zeros() }],
            duration_s: None,
            checks: vec![hover_tilt(f64::INFINITY)],
        }),
        // S1: level for 3 s, roll +10 deg at 3 s, back to level at 6 s, end at 9 s. Each edge leaves
        //     3 s = 15 tau to settle (2 % settling is ~6 tau).
        //     Predictions: docs/scripts/delay_budget_reference.py --scenario roll_step --drag 0.1122
        //     --drag-arm 0.0833 --rotor-tau 0.0115 0.0245: the roll axis with the actuation-delay budget
        //     (one period + rotor lag) and the lateral drag of free flight, every parameter identified
        //     by docs/scripts/identify_vehicle.py on runs A and B (runs/roll_step_A.csv, _B.csv).
        //       - the CHECK: the roll held at the end of the hold, 5.99 s. Tilted, the vehicle slides
        //         sideways; the drag on the rotors, above the CoM, pushes the roll back, and the law has
        //         no integrator: it holds 6.43 deg instead of 10. +/- 0.5 deg covers the drag known to
        //         +/- 15 % (0.3 deg) and the moment gain to +/- 3 % (0.1 deg); the control period does
        //         not matter (6.43 deg from 7.5 to 12.5 ms). The "red" run (bug #9, 62.5 % of the
        //         moment; same script with the measured --moment-gain 0.626) predicts 4.78 deg, and
        //         runs A and B measured 4.8 deg: 1.6 deg outside, so the check FAILs as it must;
        //       - for INFORMATION, the edges: the roll-yaw coupling through the tilting arms (the
        //         gyroscopic torque of a tilted spinning rotor) is not in the model and moves the
        //         fractions by up to 7 points (runs A and B). The second edge starts from 6.4 deg and
        //         the drag carries the roll past level, hence fractions above 100 %.
        "roll_step" => Some(Scenario {
            name: "roll_step",
            description: "S1: roll step of +10 deg at 3 s, back to level at 6 s",
            segments: vec![
                Segment { start_s: 0.0, orientation: level,  velocity: Vector3::zeros() },
                Segment { start_s: 3.0, orientation: rolled, velocity: Vector3::zeros() },
                Segment { start_s: 6.0, orientation: level,  velocity: Vector3::zeros() },
            ],
            duration_s: Some(9.0),
            checks: vec![
                hover_tilt(3.0),
                Check::HoldRoll { at_s: 5.99, expected_deg: 6.43, tolerance_deg: 0.5 },
                Check::RollSteps { tau_s: 0.2, tolerance: None, edges: vec![
                    EdgePrediction { fractions: [0.262, 0.619, 0.802], overshoot: 0.0 },
                    EdgePrediction { fractions: [0.442, 1.026, 1.347], overshoot: 0.555 },
                ]},
            ],
        }),
        _ => None,
    }
}

/// A protobuf timestamp in seconds (f64).
pub fn timestamp_to_s(timestamp: &Timestamp) -> f64 {
    return timestamp.seconds as f64 + timestamp.nanos as f64 * 1e-9;
}

/// Hand-predicted values: the roll_step timeline switches exactly at 3 s and 6 s, and holds its
/// last setpoint after its end.
#[cfg(test)]
mod tests {
    use super::*;

    /// Roll angle (rad) of the attitude setpoint at time t.
    fn roll_at(scenario: &Scenario, t_s: f64) -> f64 {
        let orientation= UnitQuaternion::from(scenario.setpoint_at(t_s).orientation.unwrap());
        return orientation.euler_angles().0;
    }

    #[test]
    fn setpoint_follows_the_segments() {
        let scenario= by_name("roll_step").unwrap();
        let ten_degrees= 10f64.to_radians();
        // 1. before the first edge: level
        assert!(roll_at(&scenario, 2.999).abs() < 1e-12);
        // 2. from the first edge on: 10 deg of roll
        assert!((roll_at(&scenario, 3.0) - ten_degrees).abs() < 1e-12);
        assert!((roll_at(&scenario, 5.999) - ten_degrees).abs() < 1e-12);
        // 3. from the second edge on, and after the end: level again (held)
        assert!(roll_at(&scenario, 6.0).abs() < 1e-12);
        assert!(roll_at(&scenario, 100.0).abs() < 1e-12);
        assert!(!scenario.is_finished(8.99) && scenario.is_finished(9.0));
        // 4. the velocity setpoint is zero and present (Some), never missing
        assert_eq!(scenario.setpoint_at(4.0).l_velocity.map(|v| v.norm()), Some(0.0));
    }

    #[test]
    fn every_roll_edge_has_a_prediction() {
        // every scenario that measures its roll steps predicts each edge of its timeline: one
        // prediction per change of the roll setpoint
        for name in NAMES {
            let scenario= by_name(name).unwrap();
            let edges= scenario.segments.windows(2)
                            .filter(|pair| (pair[0].orientation.euler_angles().0 - pair[1].orientation.euler_angles().0).abs() > 1e-9).count();
            for check in &scenario.checks {
                if let Check::RollSteps { edges: predictions, .. }= check {
                    assert_eq!(predictions.len(), edges, "scenario {}", name);
                }
            }
        }
    }
}
