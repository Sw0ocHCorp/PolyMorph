//! `report`: reads a recording back, measures it, and judges it against the predictions of its
//! scenario [P10].
//!
//! Run as `gazebo_vehicles_controller report <file.csv>` (Gazebo is not needed). Exit code:
//!   0  PASS         every check holds;
//!   1  FAIL         the run is valid and at least one check fails;
//!   2  INVALID RUN  the run cannot be compared with the predictions (control updates too far apart
//!                   in simulated time, vehicle near the ground, recording incomplete) - or the file
//!                   is unreadable.
//! A FAIL says something about the stack; an INVALID RUN only says the measurement must be redone.

use std::fs;

use nalgebra::{Quaternion, UnitQuaternion, Vector3};

use crate::scenarios::{Check, by_name};

/// Accepted mean simulated time between two control updates, relative to the nominal period T:
/// [0.75 T, 1.25 T]. Over that range the predictions move by 1.2 points at most
/// (`docs/scripts/delay_budget_reference.py` run with a 0.5 ms sub-step, which divides every period of
/// the range: 25.0 / 61.0 / 81.2 % at 7.5 ms, 26.2 / 62.7 / 82.0 % at 12.5 ms, against
/// 26.1 / 62.2 / 81.8 % at 10 ms; the script's default 1 ms sub-step gives 26.2 % at 10 ms).
const PERIOD_RANGE: (f64, f64)= (0.75, 1.25);
/// Longest accepted simulated gap between two control updates, in periods: one missed update at most.
const MAX_GAP_PERIODS: f64= 2.0;
/// Lowest accepted altitude (m): below it the vehicle may touch the ground, a contact the
/// predictions do not model.
const MIN_ALTITUDE_M: f64= 0.5;

/// One row of a recording, reduced to what the checks need.
#[derive(Clone, Debug)]
pub struct Row {
    /// Wall-clock time of the pass since the start of the recording (s).
    pub t_wall_s: f64,
    /// Simulated time of the pose of the pass (s).
    pub t_sim_s: f64,
    /// A new pose reached this pass (the simulated time advanced).
    pub fresh: bool,
    /// Measured roll (rad).
    pub roll: Option<f64>,
    /// Measured tilt: angle between the body z axis and the world z axis (rad), from q.
    pub tilt: Option<f64>,
    /// Roll of the setpoint q_d (rad).
    pub roll_d: Option<f64>,
    /// Altitude of the vehicle origin, world z (m), from the odometry.
    pub pz: Option<f64>,
    /// Number of non-finite values in the row (measurements, setpoint, commands).
    pub non_finite: usize,
}

/// A whole recording: its header and its rows.
#[derive(Debug)]
pub struct Recording {
    pub scenario: String,
    /// Nominal period of the processes pipeline (s).
    pub period_s: f64,
    pub rows: Vec<Row>,
}

/// Parses a recording written by `Recorder` (layout in `recorder.rs`).
pub fn parse(text: &str) -> Result<Recording, String> {
    let mut lines= text.lines();
    // 1. line 1: `# scenario=<name> period_s=<period>`
    let header= lines.next().ok_or("empty file")?;
    let (mut scenario, mut period_s)= (None, None);
    for pair in header.trim_start_matches('#').split_whitespace() {
        match pair.split_once('=') {
            Some(("scenario", value)) => scenario= Some(value.to_string()),
            Some(("period_s", value)) => period_s= value.parse::<f64>().ok(),
            _ => {}
        }
    }
    let (Some(scenario), Some(period_s))= (scenario, period_s) else {
        return Err(format!("line 1 is not a recording header: {:?}", header));
    };
    // 2. line 2: the column names. Columns are found by name, not by position.
    let names: Vec<&str>= lines.next().ok_or("no column line")?.split(',').collect();
    let index= |name: &str| names.iter().position(|column| *column == name).ok_or(format!("column {} missing", name));
    let (i_t, i_fresh, i_roll, i_roll_d)= (index("t_sim_s")?, index("fresh")?, index("roll")?, index("roll_d")?);
    let (i_wall, i_pz)= (index("t_wall_s")?, index("pz")?);
    let (i_qw, i_qx, i_qy, i_qz)= (index("qw")?, index("qx")?, index("qy")?, index("qz")?);
    let i_commands= index("motor_cmds")?;
    // motor feedbacks: absent from recordings made before they were logged, hence optional
    let i_feedbacks= names.iter().position(|column| *column == "motor_fb");
    // 3. one Row per line
    let mut rows= Vec::new();
    for (n, line) in lines.enumerate() {
        let fields: Vec<&str>= line.split(',').collect();
        if fields.len() != names.len() {
            return Err(format!("line {}: {} fields, {} expected", n + 3, fields.len(), names.len()));
        }
        // 3a. a numeric field: empty = missing (None) [P3]; "NaN" / "inf" parse as non-finite numbers
        let value= |i: usize| fields[i].parse::<f64>().ok();
        // 3b. the non-finite values of the row: every numeric column, then every command value
        let is_pair_list= |i: usize| i == i_commands || Some(i) == i_feedbacks;
        let mut non_finite= (0..fields.len()).filter(|i| !is_pair_list(*i))
                                            .filter(|i| value(*i).is_some_and(|v| !v.is_finite())).count();
        for i in (0..fields.len()).filter(|i| is_pair_list(*i)) {
            non_finite += fields[i].split('|').filter_map(|pair| pair.split_once('='))
                                    .filter(|(_, v)| v.parse::<f64>().is_ok_and(|v| !v.is_finite())).count();
        }
        // 3c. the tilt: the body z axis rotated into the world by q, and its angle to the world z axis.
        //     atan2(horizontal part, vertical part) rather than acos(z): well-conditioned near 0.
        let tilt= match (value(i_qw), value(i_qx), value(i_qy), value(i_qz)) {
            (Some(w), Some(x), Some(y), Some(z)) => {
                let z_body= UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z)) * Vector3::z();
                Some(f64::atan2(z_body.xy().norm(), z_body.z))
            },
            _ => None,
        };
        let Some(t_sim_s)= value(i_t) else {
            return Err(format!("line {}: no simulated time", n + 3));
        };
        rows.push(Row { t_wall_s: value(i_wall).unwrap_or(f64::NAN), t_sim_s, fresh: fields[i_fresh] == "1", roll: value(i_roll), tilt,
                        roll_d: value(i_roll_d), pz: value(i_pz), non_finite });
    }
    return Ok(Recording { scenario, period_s, rows });
}

/// What a step response is measured by.
#[derive(Debug)]
pub struct StepMeasure {
    /// Fraction of the step covered at tau, 2 tau, 3 tau (`None` if the window is shorter).
    pub fractions: [Option<f64>; 3],
    /// max(0, highest fraction - 1).
    pub overshoot: f64,
    /// Time from the edge until the response enters the +/- 2 % band for good (`None`: not settled
    /// within the window).
    pub settling_s: Option<f64>,
}

/// Measures a step response. `samples`: (t, y) with strictly increasing t, the first one AT the
/// edge (the value measured when the setpoint changes), the last one at the end of the window.
/// `target`: the new setpoint. `None` if there is no step to cover.
pub fn measure_step(samples: &[(f64, f64)], target: f64, tau_s: f64) -> Option<StepMeasure> {
    // 1. the step to cover: from the value measured at the edge to the new setpoint. Measured from
    //    y0, not from the old setpoint: the vehicle never sits exactly on it.
    let (t_edge_s, y0)= *samples.first()?;
    let step= target - y0;
    if step.abs() < 1e-9 {
        return None;
    }
    // 2. the fraction covered at each sample, f = (y - y0) / (target - y0): 0 at the edge, 1 on
    //    target, whatever the direction of the step (a falling edge divides by a negative step)
    let fraction: Vec<(f64, f64)>= samples.iter().map(|(t, y)| (t - t_edge_s, (y - y0) / step)).collect();
    // 3. f at a given time after the edge: linear interpolation between the two samples around it
    let at= |t: f64| interpolate(&fraction, t);
    // 4. overshoot: how far beyond the target the response went
    let highest= fraction.iter().map(|(_, f)| *f).fold(f64::NEG_INFINITY, f64::max);
    // 5. 2 % settling: the sample right after the LAST one outside the band. Taking the first
    //    entry into the band instead would call settled a response that leaves it again.
    let settling_s= match fraction.iter().rposition(|(_, f)| (f - 1.0).abs() > 0.02) {
        None => Some(0.0),
        Some(i) if i + 1 < fraction.len() => Some(fraction[i + 1].0),
        Some(_) => None,
    };
    return Some(StepMeasure { fractions: [at(tau_s), at(2.0 * tau_s), at(3.0 * tau_s)],
                                overshoot: (highest - 1.0).max(0.0), settling_s });
}

/// Linear interpolation of `samples` (t, y), t strictly increasing, at time `t`: `None` outside them.
pub fn interpolate(samples: &[(f64, f64)], t: f64) -> Option<f64> {
    return samples.windows(2).find(|pair| pair[0].0 <= t && t <= pair[1].0)
                    .map(|pair| pair[0].1 + (pair[1].1 - pair[0].1) * (t - pair[0].0) / (pair[1].0 - pair[0].0));
}

/// (simulated time, roll) of the rows that have a roll, with strictly increasing times: a pass
/// without a new pose repeats the time of the previous one and is skipped.
pub fn roll_samples(rows: &[Row]) -> Vec<(f64, f64)> {
    let mut samples: Vec<(f64, f64)>= Vec::new();
    for row in rows {
        if let Some(roll)= row.roll && samples.last().is_none_or(|(t, _)| row.t_sim_s > *t) {
            samples.push((row.t_sim_s, roll));
        }
    }
    return samples;
}

/// The rows of the scripted part of the scenario, from its start (t = 0, the first row) to the end of
/// its timeline. Rows recorded after it (controller left running, Gazebo paused or crashed) are not
/// judged. A timeline held forever (`None`) keeps every row.
pub fn scenario_window(rows: &[Row], duration_s: Option<f64>) -> &[Row] {
    // 1. no end: the whole recording is the scenario
    let Some(duration_s)= duration_s else {
        return rows;
    };
    // 2. otherwise: up to the last row whose scenario time is within the duration
    let t0_s= rows[0].t_sim_s;
    let end= rows.iter().position(|row| row.t_sim_s - t0_s > duration_s).unwrap_or(rows.len());
    return &rows[..end];
}

/// Simulated time between two consecutive CONTROL UPDATES, i.e. between two passes that received a
/// new pose: (mean, max) in seconds, `None` with fewer than two updates. Passes without a new pose are
/// skipped: they happen while Gazebo is stalled (rendering its sensors), the physics does not move
/// during them, so they delay nothing in simulated time. What the predictions assume is one update
/// per period of SIMULATED time.
pub fn control_update_gaps(rows: &[Row]) -> Option<(f64, f64)> {
    // 1. the simulated times of the passes that brought a new pose
    let updates: Vec<f64>= rows.iter().filter(|row| row.fresh).map(|row| row.t_sim_s).collect();
    if updates.len() < 2 {
        return None;
    }
    // 2. the gaps between consecutive updates, their mean and their maximum
    let gaps: Vec<f64>= updates.windows(2).map(|pair| pair[1] - pair[0]).collect();
    let mean= gaps.iter().sum::<f64>() / gaps.len() as f64;
    let max= gaps.iter().cloned().fold(0.0, f64::max);
    return Some((mean, max));
}

/// The edges of the roll setpoint: (index of the first row carrying the new setpoint, new roll).
pub fn roll_edges(rows: &[Row]) -> Vec<(usize, f64)> {
    let mut edges= Vec::new();
    for i in 1..rows.len() {
        if let (Some(before), Some(after))= (rows[i - 1].roll_d, rows[i].roll_d) && (after - before).abs() > 1e-9 {
            edges.push((i, after));
        }
    }
    return edges;
}

/// One line of the report table. `ok`: `Some(false)` marks a failed line.
fn print_line(label: &str, measured: String, expected: String, ok: Option<bool>) {
    let mark= match ok { Some(true) => "ok", Some(false) => "FAIL", None => "" };
    println!("  {:<42} {:>12}   {:<26} {}", label, measured, expected, mark);
}

/// Judges the recording at `path`; returns the exit code (see the module documentation).
pub fn run(path: &str) -> i32 {
    // 1. read and parse the recording
    let recording= match fs::read_to_string(path).map_err(|e| e.to_string()).and_then(|text| parse(&text)) {
        Ok(recording) => recording,
        Err(error) => {
            eprintln!("[REPORT] cannot read {}: {}", path, error);
            return 2;
        },
    };
    // 2. its scenario: the predictions to judge against
    let Some(scenario)= by_name(&recording.scenario) else {
        eprintln!("[REPORT] unknown scenario {:?} in {}", recording.scenario, path);
        return 2;
    };
    if recording.rows.len() < 2 {
        eprintln!("[REPORT] {} holds {} row(s): nothing to judge (was Gazebo started?)", path, recording.rows.len());
        return 2;
    }
    // 3. only the scripted part of the scenario is judged; scenario time = simulated time since the
    //    first row (the player and the recorder both start at the first stamped pose, same pass)
    let rows= scenario_window(&recording.rows, scenario.duration_s);
    let t0_s= rows[0].t_sim_s;
    let (mut invalid, mut failed)= (false, false);
    println!("Report of {} - scenario {} ({}), {} rows judged, {} after the end of the scenario ignored",
                path, scenario.name, scenario.description, rows.len(), recording.rows.len() - rows.len());

    // 4. validity of the run: can it be compared with the predictions at all?
    println!("Validity of the run {:>35}   {}", "measured", "expected");
    let period_s= recording.period_s;
    //    4a. one control update per period of SIMULATED time, as the predictions assume. Passes without
    //        a new pose (Gazebo stalled) are not counted: the physics did not move during them.
    match control_update_gaps(rows) {
        Some((mean_s, max_s)) => {
            let ok= mean_s >= PERIOD_RANGE.0 * period_s && mean_s <= PERIOD_RANGE.1 * period_s;
            invalid |= !ok;
            print_line("simulated time per control update (mean)", format!("{:.2} ms", mean_s * 1e3),
                        format!("{:.1} to {:.1} ms", PERIOD_RANGE.0 * period_s * 1e3, PERIOD_RANGE.1 * period_s * 1e3), Some(ok));
            //    4b. no long hole: at most one missed update between two
            let ok= max_s <= MAX_GAP_PERIODS * period_s;
            invalid |= !ok;
            print_line("longest simulated gap between two updates", format!("{:.2} ms", max_s * 1e3),
                        format!("<= {:.1} ms", MAX_GAP_PERIODS * period_s * 1e3), Some(ok));
        },
        None => {
            invalid= true;
            print_line("control updates", "< 2".to_string(), "a whole scenario".to_string(), Some(false));
        },
    }
    //    4c. in the air all along: a ground contact is outside what the predictions model
    let lowest= rows.iter().filter_map(|row| row.pz).fold(f64::INFINITY, f64::min);
    let ok= lowest.is_finite() && lowest >= MIN_ALTITUDE_M;
    invalid |= !ok;
    print_line("lowest altitude", if lowest.is_finite() { format!("{:.2} m", lowest) } else { "-".to_string() },
                format!(">= {} m", MIN_ALTITUDE_M), Some(ok));
    //    4d. information only: how fast Gazebo ran, and how many passes it left without a new pose
    let wall_s= rows[rows.len() - 1].t_wall_s - rows[0].t_wall_s;
    let stalled= rows.iter().filter(|row| !row.fresh).count();
    print_line("real-time factor (information)", format!("{:.2}", (rows[rows.len() - 1].t_sim_s - t0_s) / wall_s),
                "-".to_string(), None);
    print_line("passes without a new pose (information)", format!("{:.1} %", 100.0 * stalled as f64 / rows.len() as f64),
                "Gazebo stalls".to_string(), None);
    //    4e. a scenario with steps: the recording must contain every edge of its timeline, or it was
    //        stopped too early
    let edges= roll_edges(rows);
    if scenario.checks.iter().any(|check| matches!(check, Check::RollSteps { .. })) {
        let expected_edges= scenario.segments.windows(2)
                                .filter(|pair| (pair[0].orientation.euler_angles().0 - pair[1].orientation.euler_angles().0).abs() > 1e-9).count();
        let ok= edges.len() == expected_edges;
        invalid |= !ok;
        print_line("roll edges recorded", format!("{}", edges.len()), format!("{}", expected_edges), Some(ok));
    }

    // 5. the checks
    println!("Checks {:>48}   {}", "measured", "predicted");
    //    5a. always: no non-finite value anywhere (a NaN sent to Gazebo kills a rotor for the run)
    let non_finite: usize= rows.iter().map(|row| row.non_finite).sum();
    failed |= non_finite > 0;
    print_line("non-finite values", format!("{}", non_finite), "0".to_string(), Some(non_finite == 0));
    //    5b. the scenario's own checks
    for check in &scenario.checks {
        match check {
            // the hover: maximum tilt over the window
            Check::HoverTilt { from_s, to_s, max_tilt_deg } => {
                let tilt_max= rows.iter().filter(|row| row.t_sim_s - t0_s >= *from_s && row.t_sim_s - t0_s <= *to_s)
                                        .filter_map(|row| row.tilt).fold(f64::NAN, f64::max);
                let ok= tilt_max.to_degrees() <= *max_tilt_deg;
                failed |= !ok;
                print_line(&format!("hover tilt max, t in [{:.1} s, {:.1} s]", from_s, to_s.min(rows[rows.len() - 1].t_sim_s - t0_s)),
                            format!("{:.3} deg", tilt_max.to_degrees()), format!("<= {} deg", max_tilt_deg), Some(ok));
            },
            // the roll steps: every edge of the setpoint (found in 4e), measured until the next one, against
            // the prediction for that edge. Without a tolerance the lines are information: never a FAIL
            Check::RollSteps { tau_s, edges: predictions, tolerance } => {
                for (n, (i_edge, target)) in edges.iter().enumerate() {
                    // samples of this edge: from the edge row to the next edge, strictly increasing times
                    let i_end= edges.get(n + 1).map(|(i, _)| *i).unwrap_or(rows.len());
                    let samples= roll_samples(&rows[*i_edge..i_end]);
                    let (Some(measure), Some(prediction))= (measure_step(&samples, *target, *tau_s), predictions.get(n)) else {
                        invalid= true;
                        print_line(&format!("edge {}: not measurable, or no prediction", n + 1), "-".to_string(), "".to_string(), Some(false));
                        continue;
                    };
                    println!("  edge {} at t = {:.2} s: {:.1} -> {:.1} deg{}", n + 1, rows[*i_edge].t_sim_s - t0_s,
                                samples[0].1.to_degrees(), target.to_degrees(), if tolerance.is_some() { "" } else { " (information, not judged)" });
                    for (k, (measured, predicted)) in measure.fractions.iter().zip(prediction.fractions.iter()).enumerate() {
                        let label= format!("  fraction covered at {} tau", k + 1);
                        match measured {
                            Some(measured) => {
                                let ok= tolerance.map(|tolerance| (measured - predicted).abs() <= tolerance);
                                failed |= ok == Some(false);
                                let expected= match tolerance {
                                    Some(tolerance) => format!("{:.1} % +/- {:.1}", predicted * 100.0, tolerance * 100.0),
                                    None => format!("{:.1} %", predicted * 100.0),
                                };
                                print_line(&label, format!("{:.1} %", measured * 100.0), expected, ok);
                            },
                            // the recording stops before k tau: incomplete
                            None => {
                                invalid= true;
                                print_line(&label, "-".to_string(), "recording too short".to_string(), Some(false));
                            },
                        }
                    }
                    let ok= tolerance.map(|tolerance| measure.overshoot <= prediction.overshoot + tolerance);
                    failed |= ok == Some(false);
                    let expected= match tolerance {
                        Some(tolerance) => format!("<= {:.1} %", (prediction.overshoot + tolerance) * 100.0),
                        None => format!("{:.1} %", prediction.overshoot * 100.0),
                    };
                    print_line("  overshoot", format!("{:.1} %", measure.overshoot * 100.0), expected, ok);
                    print_line("  2 % settling (information)",
                                measure.settling_s.map(|s| format!("{:.2} s", s)).unwrap_or("not settled".to_string()), "-".to_string(), None);
                }
            },
            // the roll held at a given time (end of a hold), interpolated between the two control
            // updates around it
            Check::HoldRoll { at_s, expected_deg, tolerance_deg } => {
                let label= format!("roll at t = {:.2} s (end of the hold)", at_s);
                match interpolate(&roll_samples(rows), t0_s + at_s) {
                    Some(roll) => {
                        let ok= (roll.to_degrees() - expected_deg).abs() <= *tolerance_deg;
                        failed |= !ok;
                        print_line(&label, format!("{:.2} deg", roll.to_degrees()),
                                    format!("{:.2} deg +/- {:.1}", expected_deg, tolerance_deg), Some(ok));
                    },
                    // the recording stops before that time: incomplete
                    None => {
                        invalid= true;
                        print_line(&label, "-".to_string(), "recording too short".to_string(), Some(false));
                    },
                }
            },
        }
    }

    // 6. the verdict: an invalid run says nothing about the stack, whatever its checks say
    let (verdict, code)= if invalid { ("INVALID RUN (redo the measurement)", 2) } else if failed { ("FAIL", 1) } else { ("PASS", 0) };
    println!("VERDICT: {}", verdict);
    return code;
}

/// Hand-predicted values for the measurement code itself, on curves whose answer is known.
#[cfg(test)]
mod tests {
    use super::*;

    /// The critically damped response to a unit step, `1 - (1 + t/tau) e^(-t/tau)`, sampled every
    /// 10 ms for 2 s, scaled to go from `from` to `to`.
    fn closed_form(from: f64, to: f64, tau_s: f64) -> Vec<(f64, f64)> {
        return (0..=200).map(|i| {
            let t= i as f64 * 0.01;
            (t, from + (to - from) * (1.0 - (1.0 + t / tau_s) * (-t / tau_s).exp()))
        }).collect();
    }

    #[test]
    fn step_measures_on_the_closed_form() {
        // 1. a rising step 0 -> 10 deg: the milestones of docs/docs/math/cascaded-control.mdx,
        //    1 - 2/e = 26.42 %, 1 - 3/e^2 = 59.40 %, 1 - 4/e^3 = 80.09 %, no overshoot, and a 2 %
        //    settling at the first sample after 5.834 tau = 1.1668 s, i.e. 1.17 s
        let rising= measure_step(&closed_form(0.0, 10.0, 0.2), 10.0, 0.2).unwrap();
        let expected= [1.0 - 2.0 / 1f64.exp(), 1.0 - 3.0 / 2f64.exp(), 1.0 - 4.0 / 3f64.exp()];
        for k in 0..3 {
            assert!((rising.fractions[k].unwrap() - expected[k]).abs() < 1e-9, "rising, {} tau: {:?}", k + 1, rising.fractions[k]);
        }
        assert_eq!(rising.overshoot, 0.0);
        assert!((rising.settling_s.unwrap() - 1.17).abs() < 1e-9, "settling {:?}", rising.settling_s);
        // 2. the falling edge 10 -> 0 deg is the same response: same fractions
        let falling= measure_step(&closed_form(10.0, 0.0, 0.2), 0.0, 0.2).unwrap();
        for k in 0..3 {
            assert!((falling.fractions[k].unwrap() - expected[k]).abs() < 1e-9, "falling, {} tau: {:?}", k + 1, falling.fractions[k]);
        }
        // 3. a response that enters the band then leaves it is not settled at its first entry
        let mut bouncing= closed_form(0.0, 1.0, 0.2);
        bouncing[190].1= 1.05;
        assert!((measure_step(&bouncing, 1.0, 0.2).unwrap().settling_s.unwrap() - 1.91).abs() < 1e-9);
    }

    /// A CSV line in the order of `COLUMNS`, from (column, value) pairs; every other field is empty.
    fn csv_row(values: &[(&str, &str)]) -> String {
        return crate::scenarios::recorder::COLUMNS.iter()
            .map(|column| values.iter().find(|(name, _)| name == column).map(|(_, v)| v.to_string()).unwrap_or_default())
            .collect::<Vec<String>>().join(",");
    }

    /// A row at simulated time `t_ms` (ms), with or without a new pose; the other fields do not
    /// matter for the validity functions.
    fn row(t_ms: f64, fresh: bool) -> Row {
        Row { t_wall_s: 0.0, t_sim_s: t_ms / 1e3, fresh, roll: None, tilt: None, roll_d: None, pz: None, non_finite: 0 }
    }

    #[test]
    fn control_updates_skip_the_stalled_passes() {
        // updates at 0, 8, 16 and 28 ms, and two passes without a new pose at 8 ms (Gazebo stalled):
        // gaps of 8, 8 and 12 ms, so a mean of 28/3 = 9.333 ms and a max of 12 ms. Counting the
        // stalled passes as updates would give 28/5 = 5.6 ms and wrongly invalidate the run.
        let rows= [row(0.0, true), row(8.0, true), row(8.0, false), row(8.0, false), row(16.0, true), row(28.0, true)];
        let (mean_s, max_s)= control_update_gaps(&rows).unwrap();
        assert!((mean_s - 0.028 / 3.0).abs() < 1e-12, "mean {}", mean_s);
        assert!((max_s - 0.012).abs() < 1e-12, "max {}", max_s);
    }

    #[test]
    fn rows_after_the_end_of_the_scenario_are_not_judged() {
        // a 9 s scenario starting at 10 s of simulated time: 10, 13 and 19 s are inside (19 - 10 = 9 s
        // is still the scenario), 19.01 and 25 s are after its end
        let rows= [row(10_000.0, true), row(13_000.0, true), row(19_000.0, true), row(19_010.0, true), row(25_000.0, true)];
        assert_eq!(scenario_window(&rows, Some(9.0)).len(), 3);
        // a timeline held forever keeps every row
        assert_eq!(scenario_window(&rows, None).len(), 5);
    }

    /// A complete roll_step recording: 9 s at 10 ms, every row fresh, 20 m up. Level, then from 3 s a
    /// roll that settles on `hold_deg` (first order, 0.2 s), then from 6 s back to level.
    fn roll_step_recording(hold_deg: f64) -> String {
        let mut lines= vec!["# scenario=roll_step period_s=0.01".to_string(), crate::scenarios::recorder::COLUMNS.join(",")];
        for i in 0..=900 {
            // 1. the setpoint and the measured roll at this pass (index-based edges: no float drift)
            let t= i as f64 * 0.01;
            let roll_d: f64= if (300..600).contains(&i) { 10f64.to_radians() } else { 0.0 };
            let roll= match i {
                0..300 => 0.0,
                300..600 => hold_deg.to_radians() * (1.0 - (-(t - 3.0) / 0.2).exp()),
                _ => hold_deg.to_radians() * (-(t - 6.0) / 0.2).exp(),
            };
            // 2. the row, by column name; q and q_d are rotations about the body x axis
            let fields= [("t_wall_s", t.to_string()), ("t_sim_s", t.to_string()), ("fresh", "1".to_string()),
                         ("qw", (roll / 2.0).cos().to_string()), ("qx", (roll / 2.0).sin().to_string()), ("qy", "0".to_string()), ("qz", "0".to_string()),
                         ("roll", roll.to_string()), ("pitch", "0".to_string()), ("yaw", "0".to_string()),
                         ("qd_w", (roll_d / 2.0).cos().to_string()), ("qd_x", (roll_d / 2.0).sin().to_string()), ("qd_y", "0".to_string()), ("qd_z", "0".to_string()),
                         ("roll_d", roll_d.to_string()), ("pz", "20".to_string()), ("motor_cmds", "1=5|2=5".to_string())];
            let refs: Vec<(&str, &str)>= fields.iter().map(|(name, value)| (*name, value.as_str())).collect();
            lines.push(csv_row(&refs));
        }
        return lines.join("\n");
    }

    /// `run` on a recording written to a temporary file; returns the exit code.
    fn verdict_of(text: &str, tag: &str) -> i32 {
        let path= std::env::temp_dir().join(format!("report_test_{}_{}.csv", std::process::id(), tag));
        fs::write(&path, text).unwrap();
        let code= run(path.to_str().unwrap());
        let _= fs::remove_file(&path);
        return code;
    }

    #[test]
    fn verdict_judges_the_hold_roll() {
        // the prediction of the catalogue for the end of the hold
        let scenario= by_name("roll_step").unwrap();
        let expected_deg= scenario.checks.iter().find_map(|check| match check {
            Check::HoldRoll { expected_deg, .. } => Some(*expected_deg),
            _ => None,
        }).expect("roll_step judges the hold");
        // 1. a run that holds the predicted roll passes: the edges are information and cannot fail it
        assert_eq!(verdict_of(&roll_step_recording(expected_deg), "green"), 0);
        // 2. the red run of bug #9 (4.78 deg predicted, 4.8 deg measured on runs A and B) fails
        assert_eq!(verdict_of(&roll_step_recording(4.8), "red"), 1);
    }

    #[test]
    fn interpolation_between_two_samples() {
        // 1. halfway between (1, 10) and (2, 20): 15; on a sample: its value
        let samples= [(1.0, 10.0), (2.0, 20.0), (3.0, 0.0)];
        assert_eq!(interpolate(&samples, 1.5), Some(15.0));
        assert_eq!(interpolate(&samples, 2.0), Some(20.0));
        // 2. outside the samples: no value
        assert_eq!(interpolate(&samples, 3.5), None);
    }

    #[test]
    fn recording_is_parsed_by_column_name() {
        // 1. a hand-written recording: one complete row, one with a NaN command, a NaN feedback and a
        //    missing roll_d. Rows are built by column name, so the test survives new columns.
        let header= "# scenario=roll_step period_s=0.01";
        let columns= crate::scenarios::recorder::COLUMNS.join(",");
        let level= [("qw", "1"), ("qx", "0"), ("qy", "0"), ("qz", "0"), ("roll", "0"), ("pitch", "0"), ("yaw", "0")];
        let mut row_1: Vec<(&str, &str)>= vec![("t_wall_s", "0.0"), ("t_sim_s", "10.0"), ("fresh", "1"),
                                                ("qd_w", "1"), ("qd_x", "0"), ("qd_y", "0"), ("qd_z", "0"), ("roll_d", "0"),
                                                ("motor_cmds", "7=5.886|8=5.886"), ("motor_fb", "7=5.88|8=5.88")];
        row_1.extend(level);
        let mut row_2: Vec<(&str, &str)>= vec![("t_wall_s", "0.01"), ("t_sim_s", "10.01"), ("fresh", "0"),
                                                ("motor_cmds", "7=NaN|8=5.886"), ("motor_fb", "7=5.88|8=NaN")];
        row_2.extend(level);
        let text= [header.to_string(), columns, csv_row(&row_1), csv_row(&row_2)].join("\n");
        // 2. parsed: header values, both rows, the NaN counted, the missing roll_d kept as None
        let recording= parse(&text).unwrap();
        assert_eq!(recording.scenario, "roll_step");
        assert_eq!(recording.period_s, 0.01);
        assert_eq!(recording.rows.len(), 2);
        assert_eq!(recording.rows[0].roll_d, Some(0.0));
        assert_eq!(recording.rows[0].tilt, Some(0.0));
        assert_eq!(recording.rows[0].non_finite, 0);
        assert!(recording.rows[0].fresh && !recording.rows[1].fresh);
        assert_eq!(recording.rows[1].roll_d, None);
        assert_eq!(recording.rows[1].non_finite, 2);
    }
}
