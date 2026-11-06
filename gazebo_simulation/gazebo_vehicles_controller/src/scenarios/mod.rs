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
//! zeta = 1, velocity tau_v = 2 s, zeta = 1, processes pipeline at 100 Hz (T = 10 ms), and for the
//! OSPRAI as identified in free flight by `docs/scripts/identify_vehicle.py`. Change one of them and
//! the predictions must be recomputed with `docs/scripts/delay_budget_reference.py` (roll) and
//! `docs/scripts/velocity_step_reference.py` (velocity). The predictions are DATA of the catalogue: the
//! checks themselves know nothing about the vehicle.

use nalgebra::{UnitQuaternion, Vector3};
use prost_types::Timestamp;
use robomorph::messages::{pose_messages::{IMUMeasurements, Pose}, registered_message::{UnitQuat, Vec3}};

pub mod player;
pub mod recorder;
pub mod report;

/// Names accepted by `by_name`, for the command-line help.
pub const NAMES: [&str; 5]= ["hover", "roll_step", "roll_refused", "velocity_hold", "velocity_step_y"];

/// One piece of a timeline: from `start_s` on, the setpoint is constant. Timelines are piecewise
/// constant (steps only), which is exactly what a step response needs.
#[derive(Clone, Debug)]
pub struct Segment {
    /// Start of the segment, in simulated seconds since the scenario started.
    pub start_s: f64,
    /// q_d, the attitude setpoint body->world. Read by the attitude stage today; it becomes the
    /// heading preference q_pref of the resolver in brick 3.
    pub orientation: UnitQuaternion<f64>,
    /// v_d, the velocity setpoint in the WORLD frame (m/s) [P7], read by the velocity loop. `None`:
    /// stabilize, the loop does not run and only the weight is carried.
    pub velocity: Option<Vector3<f64>>,
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
    /// Every edge of one component of the velocity setpoint (`axis`: 0 = x, 1 = y, 2 = z, world
    /// frame) is a step response of the velocity loop, measured and judged like `RollSteps`, with
    /// `tau_s` = tau_v.
    VelocitySteps { axis: usize, tau_s: f64, edges: Vec<EdgePrediction>, tolerance: Option<f64> },
    /// The measured speed |v| (world, m/s) stays below `max_speed` between `from_s` and `to_s`
    /// (scenario time): the velocity loop holds v_d = 0.
    HoldVelocity { from_s: f64, to_s: f64, max_speed: f64 },
}

/// The `axis` component of a segment's velocity setpoint (m/s), 0 without one (stabilize).
pub fn velocity_component(segment: &Segment, axis: usize) -> f64 {
    return segment.velocity.map_or(0.0, |velocity| velocity[axis]);
}

/// What the reference model predicts for one edge of a step: fraction of the step covered at tau,
/// 2 tau, 3 tau, and the overshoot (all as fractions of the step, not percents).
#[derive(Clone, Debug)]
pub struct EdgePrediction {
    pub fractions: [f64; 3],
    pub overshoot: f64,
}

/// Which stages the setpoint goes through between the scenario player and the attitude stage. A
/// choice of the BENCH, made per scenario: the stages themselves know nothing about it, any process
/// may stand before the attitude stage as long as it fills the fields it reads.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Cascade {
    /// player -> velocity -> attitude: the mission's attitude preference is tracked as it is. Tests
    /// the attitude stage alone.
    AttitudeOnly,
    /// player -> velocity -> force -> attitude resolver -> attitude: the attitude tracked is the one
    /// that makes the desired force producible, the closest to the preference.
    WithResolver,
}

/// A timeline of setpoints and the checks of its report.
#[derive(Clone, Debug)]
pub struct Scenario {
    pub name: &'static str,
    pub description: &'static str,
    /// The stages the setpoint goes through (see `Cascade`).
    pub cascade: Cascade,
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
        setpoint.l_velocity= active.velocity.map(Vec3);
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
        //     (identity attitude), without a velocity setpoint: stabilize, the velocity loop does
        //     not run.
        "hover" => Some(Scenario {
            name: "hover",
            description: "S0: hold level, forever",
            cascade: Cascade::AttitudeOnly,
            segments: vec![Segment { start_s: 0.0, orientation: level, velocity: None }],
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
            // the attitude stage alone: through the resolver the roll would be refused (see S2)
            cascade: Cascade::AttitudeOnly,
            segments: vec![
                Segment { start_s: 0.0, orientation: level,  velocity: None },
                Segment { start_s: 3.0, orientation: rolled, velocity: None },
                Segment { start_s: 6.0, orientation: level,  velocity: None },
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
        // S2: S1 itself (same timeline, same duration), through the force -> attitude resolver, with
        //     its own check. The force stays the weight,
        //     vertical, and the OSPRAI can only push in its body xz plane: rolled by 10 deg, that plane
        //     no longer contains the vertical. The resolver turns the preference back by the smallest
        //     rotation that makes the force producible, -10 deg about x: q_d stays level, the roll is
        //     REFUSED. Prediction: the vehicle stays level all along, the S0 tilt check from 1 s to the
        //     end, while the mission asks for 10 deg between 3 and 6 s (resolver unit test
        //     `the_osprai_serves_in_plane_forces_and_refuses_the_roll`).
        "roll_refused" => by_name("roll_step").map(|s1| Scenario {
            name: "roll_refused",
            description: "S2: roll preference of +10 deg at 3 s, back to level at 6 s, through the resolver: refused",
            cascade: Cascade::WithResolver,
            checks: vec![hover_tilt(9.0)],
            ..s1
        }),
        // S3: hold v_d = 0 from Play on, through the resolver: the velocity loop's first run. The
        //     rotors start at rest, so the vehicle first sinks (-0.36 m/s measured in stabilize) before
        //     its thrust builds up; the loop then brings the speed back to 0, and its integral absorbs
        //     the ~0.1 % thrust surplus that makes a stabilized vehicle climb. The altitude comes back
        //     to the spawn point (information): the integral of the error v_d - v = -v IS the
        //     altitude lost, and it ends where it cancels the surplus, ki I = -0.0098 m/s^2, i.e.
        //     I = -0.04 m: the vehicle settles about 4 cm ABOVE the spawn point, whatever the sink.
        //     Prediction (the PI of tau_v = 2 s from -0.36 m/s at 0.25 s): |v| = 0.023 m/s at 8 s,
        //     0.011 m/s at 10 s. The check, |v| <= 0.05 m/s from 8 s to the end, leaves room for the
        //     start-up transient, which depends on the rotor spin-up; and the tilt stays below the S0
        //     bound all along (no lateral demand).
        "velocity_hold" => Some(Scenario {
            name: "velocity_hold",
            description: "S3: hold v_d = 0 from Play on, through the resolver",
            cascade: Cascade::WithResolver,
            segments: vec![Segment { start_s: 0.0, orientation: level, velocity: Some(Vector3::zeros()) }],
            duration_s: Some(20.0),
            checks: vec![
                hover_tilt(20.0),
                Check::HoldVelocity { from_s: 8.0, to_s: 20.0, max_speed: 0.05 },
            ],
        }),
        // S4: a lateral velocity step, through the resolver: v_d = (0, 0.5, 0) m/s from 3 s to 13 s,
        //     then 0 again, end at 24 s. The chain velocity -> resolver -> attitude -> mixer: the
        //     loop asks for a lateral force, the resolver rolls the OSPRAI by about -atan(a_d / g)
        //     (first a_d = kp 0.5 = 0.5 m/s^2, -2.9 deg), the tilted thrust accelerates it.
        //     Predictions: docs/scripts/velocity_step_reference.py (the PI with one tick of delay,
        //     the attitude loop as a second order of 0.2 s, the identified lateral drag 0.1122 N per
        //     m/s, and the roll the attitude loop fails to hold against the drag moment, 1.26 deg per
        //     m/s of sliding): edge 1 95.3 / 96.0 / 97.4 % at tau_v / 2 tau_v / 3 tau_v, no overshoot;
        //     edge 2 96.1 / 96.4 / 97.6 %, no overshoot. That lost roll brakes like a second drag,
        //     2.3 times the aerodynamic one: the loop is overdamped, it rises fast to ~95 % then creeps
        //     to the target while the integral asks for the missing roll. The textbook PI (`--ideal`)
        //     would give 100 / 113.6 / 109.9 %, and the first prediction, without the lost roll,
        //     105.9 / 110.6 / 105.2 % with 13.9 % of overshoot: run velocity_step_y1 measured
        //     94.7 / 95.7 / 97.1 % and 95.3 / 96.6 / 97.7 %, no overshoot (stopped at 19 s, invalid,
        //     but its edges were complete). The tolerance, 5 points: the measured roll error (1.41
        //     instead of 1.26 deg per m/s) moves the fractions by about 1 point, the rotor lag and the
        //     roll -> yaw -> arms coupling by a few. After the second edge the speed is held below
        //     0.05 m/s from 21 s (model: 0.007 m/s).
        "velocity_step_y" => Some(Scenario {
            name: "velocity_step_y",
            description: "S4: lateral velocity step of 0.5 m/s at 3 s, back to 0 at 13 s, through the resolver",
            cascade: Cascade::WithResolver,
            segments: vec![
                Segment { start_s: 0.0,  orientation: level, velocity: Some(Vector3::zeros()) },
                Segment { start_s: 3.0,  orientation: level, velocity: Some(Vector3::new(0.0, 0.5, 0.0)) },
                Segment { start_s: 13.0, orientation: level, velocity: Some(Vector3::zeros()) },
            ],
            duration_s: Some(24.0),
            checks: vec![
                Check::VelocitySteps { axis: 1, tau_s: 2.0, tolerance: Some(0.05), edges: vec![
                    EdgePrediction { fractions: [0.953, 0.960, 0.974], overshoot: 0.0 },
                    EdgePrediction { fractions: [0.961, 0.964, 0.976], overshoot: 0.0 },
                ]},
                Check::HoldVelocity { from_s: 21.0, to_s: 24.0, max_speed: 0.05 },
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
        // 4. no velocity setpoint: stabilize, the velocity loop does not run
        assert_eq!(scenario.setpoint_at(4.0).l_velocity, None);
    }

    #[test]
    fn roll_refused_replays_roll_step_through_the_resolver() {
        // the timeline of S1 by construction: what differs is the resolver, and a check of the tilt
        // over the whole scenario (the roll is expected to be refused)
        let (s1, s2)= (by_name("roll_step").unwrap(), by_name("roll_refused").unwrap());
        assert_eq!((s1.cascade, s2.cascade), (Cascade::AttitudeOnly, Cascade::WithResolver));
        assert!(matches!(s2.checks.as_slice(), [Check::HoverTilt { from_s, to_s, .. }] if *from_s == 1.0 && *to_s == 9.0));
    }

    #[test]
    fn the_velocity_scenarios_go_through_the_resolver_with_a_velocity_setpoint() {
        // the velocity loop asks for lateral forces: without the resolver, the OSPRAI would be asked
        // for a force it cannot produce (a body y force), and the integral would wind up
        for name in ["velocity_hold", "velocity_step_y"] {
            let scenario= by_name(name).unwrap();
            assert_eq!(scenario.cascade, Cascade::WithResolver, "scenario {}", name);
            assert!(scenario.segments.iter().all(|segment| segment.velocity.is_some()), "scenario {}", name);
        }
        // the step: 0.5 m/s along y between 3 s and 13 s, 0 before and after
        let step= by_name("velocity_step_y").unwrap();
        assert_eq!(step.setpoint_at(2.99).l_velocity, Some(Vec3::new(0.0, 0.0, 0.0)));
        assert_eq!(step.setpoint_at(3.0).l_velocity, Some(Vec3::new(0.0, 0.5, 0.0)));
        assert_eq!(step.setpoint_at(13.0).l_velocity, Some(Vec3::new(0.0, 0.0, 0.0)));
    }

    #[test]
    fn every_velocity_edge_has_a_prediction() {
        // one prediction per change of the judged component of the velocity setpoint
        for name in NAMES {
            let scenario= by_name(name).unwrap();
            for check in &scenario.checks {
                if let Check::VelocitySteps { axis, edges: predictions, .. }= check {
                    let edges= scenario.segments.windows(2)
                                    .filter(|pair| (velocity_component(&pair[0], *axis) - velocity_component(&pair[1], *axis)).abs() > 1e-9).count();
                    assert_eq!(predictions.len(), edges, "scenario {}", name);
                }
            }
        }
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
