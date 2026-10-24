//! `Recorder`: writes one CSV row per pass of the processes pipeline - the measured state, the
//! setpoint and the motor commands of that pass - for `report` to judge afterwards.
//!
//! Registered LAST in the main pipeline, so that a row holds everything the pass produced. Opened
//! with `create_new`: a recording never overwrites an existing file (no run, no log file is lost).
//!
//! File layout:
//!   line 1   `# scenario=<name> period_s=<nominal period of the pipeline>`
//!   line 2   the column names, `COLUMNS`
//!   then     one row per pass, starting at the first pass that has a measured pose
//! Every column is a quantity a real robot also has (IMU, velocity estimate, motor commands and
//! motor feedback): the identification of the vehicle (`docs/scripts/identify_vehicle.py`) only
//! uses these columns, never a parameter of the simulator.
//! A missing value is an EMPTY field, never 0 [P3]; a non-finite value is written as `NaN`/`inf`.

use std::{collections::BTreeMap, fs::{self, File, OpenOptions}, io::{self, LineWriter, Write}, path::Path, time::{Duration, Instant}};

use nalgebra::{Quaternion, UnitQuaternion};
use robomorph::{control::motion::motor::quaternion_to_euler, core::scheduler::Process, messages::{motor_messages::{MotorCommand, MotorFeedBack}, pose_messages::Pose, registered_message::{AnyMessage, UnitQuat, Vec3}}};
use tokio::sync::broadcast::{Receiver, Sender};

use crate::scenarios::timestamp_to_s;

/// The columns of a recording, in order. Frames: q and q_d body->world; omega and the accelerometer
/// (a, specific force: about (0, 0, +9.81) m/s^2 in a level hover) in the body frame; v, p and v_d
/// in the world frame [P7]. Angles in rad, times in s. Motor commands and motor feedbacks are in
/// effort units (N for a thruster, rad for a joint), as `id=value` pairs sorted by motor id.
pub const COLUMNS: [&str; 34]= [
    "t_wall_s", "t_sim_s", "fresh",
    "qw", "qx", "qy", "qz", "roll", "pitch", "yaw",
    "wx", "wy", "wz",
    "ax", "ay", "az",
    "vx", "vy", "vz",
    "px", "py", "pz",
    "qd_w", "qd_x", "qd_y", "qd_z", "roll_d", "pitch_d", "yaw_d",
    "vd_x", "vd_y", "vd_z",
    "motor_cmds", "motor_fb",
];

/// The recorder process.
pub struct Recorder {
    name: String,
    period: Duration,
    scenario_name: String,
    /// Flushed at every end of line: a row survives a Ctrl-C.
    writer: LineWriter<File>,
    header_written: bool,
    /// Wall-clock origin of the `t_wall_s` column.
    start_wall: Instant,
    telemetry_receiver: Option<Receiver<AnyMessage>>,
    setpoint_receiver: Option<Receiver<Pose>>,
    command_receiver: Option<Receiver<AnyMessage>>,
    /// Motor feedback channel (the one the bridge publishes the measured motor efforts on).
    feedback_receiver: Option<Receiver<MotorFeedBack>>,
    /// Latest values received (held between passes, like every stage).
    last_pose: Option<Pose>,
    last_setpoint: Option<Pose>,
    /// Latest measured effort of each motor, by id: a feedback is a STATE (held until the next
    /// one), unlike a command, which is the event of one pass. Sorted by id for the CSV.
    last_feedbacks: BTreeMap<u32, f64>,
    /// Simulated time of the previous row, to tell a fresh pose from a repeated one.
    previous_row_s: Option<f64>,
}

impl Recorder {
    /// Opens `path` for a NEW recording. Fails if the file already exists.
    pub fn new(name: String, path: &Path, scenario_name: &str) -> io::Result<Self> {
        // 1. the parent directory (e.g. `runs/`) is created when missing
        if let Some(parent)= path.parent() && !parent.as_os_str().is_empty() {
            fs::create_dir_all(parent)?;
        }
        // 2. create_new: an existing file is an error, never overwritten
        let file= OpenOptions::new().write(true).create_new(true).open(path)?;
        return Ok(Self { name, period: Duration::from_millis(0), scenario_name: scenario_name.to_string(),
                            writer: LineWriter::new(file), header_written: false, start_wall: Instant::now(),
                            telemetry_receiver: None, setpoint_receiver: None, command_receiver: None, feedback_receiver: None,
                            last_pose: None, last_setpoint: None, last_feedbacks: BTreeMap::new(), previous_row_s: None });
    }

    /// The setpoint channel (the same one the motion stages read).
    pub fn set_setpoint_receiver(&mut self, receiver: Receiver<Pose>) {
        self.setpoint_receiver= Some(receiver);
    }

    /// The motor command channel (the one the mixer publishes on).
    pub fn set_command_receiver(&mut self, receiver: Receiver<AnyMessage>) {
        self.command_receiver= Some(receiver);
    }

    /// The motor feedback channel (the one the bridge publishes the measured efforts on).
    pub fn set_feedback_receiver(&mut self, receiver: Receiver<MotorFeedBack>) {
        self.feedback_receiver= Some(receiver);
    }

    /// Writes one line, reporting (without stopping the controller) if the disk refuses it.
    fn write_line(&mut self, line: &str) {
        if let Err(error)= writeln!(self.writer, "{}", line) {
            eprintln!("[Recorder:ERROR] -> cannot write the recording: {}", error);
        }
    }
}

/// A number as a CSV field: empty when missing [P3]. `{}` prints the shortest exact representation.
fn field(value: Option<f64>) -> String {
    return value.map(|v| format!("{}", v)).unwrap_or_default();
}

/// Four fields (w, x, y, z) then three (roll, pitch, yaw) for an optional quaternion.
fn quaternion_fields(quaternion: Option<UnitQuat>) -> Vec<String> {
    // 1. missing quaternion: seven empty fields
    let Some(q)= quaternion else {
        return vec![String::new(); 7];
    };
    // 2. the raw components, as received
    let mut fields= vec![field(Some(q.w)), field(Some(q.x)), field(Some(q.y)), field(Some(q.z))];
    // 3. roll / pitch / yaw for reading the curves (display only: the report's tilt uses q itself)
    let euler= quaternion_to_euler(&UnitQuaternion::from_quaternion(Quaternion::new(q.w, q.x, q.y, q.z)));
    fields.extend(euler.iter().map(|angle| field(Some(*angle))));
    return fields;
}

/// Three fields for an optional vector.
fn vector_fields(vector: Option<Vec3>) -> Vec<String> {
    return match vector {
        Some(v) => vec![field(Some(v.x)), field(Some(v.y)), field(Some(v.z))],
        None => vec![String::new(); 3],
    };
}

impl Process for Recorder {
    /// One pass: one row with what the pass produced.
    fn exec(&mut self, _input: &Option<AnyMessage>, _dt: Duration) -> Option<AnyMessage> {
        // 1. the header, once (the period is only known after set_period_from_freq)
        if !self.header_written {
            let header= format!("# scenario={} period_s={}", self.scenario_name, self.period.as_secs_f64());
            self.write_line(&header);
            self.write_line(&COLUMNS.join(","));
            self.header_written= true;
        }
        // 2. drain the three channels, keep the latest of each. The commands are those of THIS pass
        //    only: an empty field means the mixer published nothing this pass.
        if let Some(telemetry_receiver)= &mut self.telemetry_receiver {
            let sz= telemetry_receiver.len();
            for _ in 0..sz {
                if let Ok(AnyMessage::PoseState(pose))= telemetry_receiver.try_recv() {
                    self.last_pose= Some(pose);
                }
            }
        }
        if let Some(setpoint_receiver)= &mut self.setpoint_receiver {
            let sz= setpoint_receiver.len();
            for _ in 0..sz {
                if let Ok(setpoint)= setpoint_receiver.try_recv() {
                    self.last_setpoint= Some(setpoint);
                }
            }
        }
        let mut commands: Vec<MotorCommand>= Vec::new();
        if let Some(command_receiver)= &mut self.command_receiver {
            let sz= command_receiver.len();
            for _ in 0..sz {
                if let Ok(AnyMessage::MotorCommands(motor_commands))= command_receiver.try_recv() {
                    commands= motor_commands;
                }
            }
        }
        //    The feedbacks: the latest effort of each motor. The channel is small and fed at up to
        //    1 kHz, so this receiver lags: `try_recv` then reports the skipped messages once, and
        //    the next calls return the most recent ones, which are the ones kept.
        if let Some(feedback_receiver)= &mut self.feedback_receiver {
            let sz= feedback_receiver.len();
            for _ in 0..sz {
                if let Ok(feedback)= feedback_receiver.try_recv() {
                    self.last_feedbacks.insert(feedback.id, feedback.current_value);
                }
            }
        }
        // 3. [P3] no stamped pose yet (Gazebo paused): nothing measured, no row
        let Some(pose)= self.last_pose.clone() else {
            return None;
        };
        let Some(stamp)= pose.timestamp else {
            return None;
        };
        // 4. fresh = the simulated time advanced since the previous row, i.e. a new sample reached
        //    this pass. A repeated pose means Gazebo is slower than the pipeline.
        let t_sim_s= timestamp_to_s(&stamp);
        let fresh= self.previous_row_s.is_none_or(|previous_s| t_sim_s > previous_s);
        self.previous_row_s= Some(t_sim_s);
        // 5. the row, in the order of COLUMNS
        let mut row= vec![format!("{:.6}", self.start_wall.elapsed().as_secs_f64()), field(Some(t_sim_s)),
                            if fresh { "1".to_string() } else { "0".to_string() }];
        //    5a. measured state: q (+ roll, pitch, yaw), gyro, accelerometer, world velocity, world position
        row.extend(quaternion_fields(pose.orientation));
        row.extend(vector_fields(pose.imu_measurement.clone().and_then(|imu| imu.a_velocity)));
        row.extend(vector_fields(pose.imu_measurement.and_then(|imu| imu.l_accel)));
        row.extend(vector_fields(pose.l_velocity));
        row.extend(vector_fields(pose.relative_location));
        //    5b. setpoint: q_d (+ its roll, pitch, yaw) and v_d
        let setpoint= self.last_setpoint.clone();
        row.extend(quaternion_fields(setpoint.as_ref().and_then(|s| s.orientation)));
        row.extend(vector_fields(setpoint.and_then(|s| s.l_velocity)));
        //    5c. motor commands, `id=value` sorted by id (no assumption on the number of motors [P1])
        commands.sort_by_key(|command| command.id);
        row.push(commands.iter().map(|command| format!("{}={}", command.id, command.setpoint_value))
                                .collect::<Vec<String>>().join("|"));
        //    5d. motor feedbacks, `id=value` sorted by id (the BTreeMap keeps them sorted)
        row.push(self.last_feedbacks.iter().map(|(id, value)| format!("{}={}", id, value))
                                .collect::<Vec<String>>().join("|"));
        self.write_line(&row.join(","));
        // 6. nothing on the pipe: the recorder is the last process of the pass
        return None;
    }

    fn set_receiver(&mut self, receiver: Receiver<AnyMessage>) {
        self.telemetry_receiver= Some(receiver);
    }

    /// The recorder publishes nothing.
    fn set_sender(&mut self, _sender: Sender<AnyMessage>) {
        println!("[WARNING] -> {} doesn't have any sender", self.name);
    }

    fn set_name(&mut self, name: String) {
        self.name= name;
    }

    fn get_name(&self) -> String {
        return self.name.clone();
    }

    fn set_period_from_freq(&mut self, frequency: u64) {
        self.period= Duration::from_nanos(1_000_000_000 / frequency);
    }

    fn get_period(&self) -> Duration {
        return self.period;
    }
}

/// The recorder and the report must agree on the layout. Hand-predicted values: a pose rolled by
/// 0.1 rad and stamped 12.5 s, a setpoint rolled by 0.2 rad and two commands sent out of order come
/// back from `report::parse` as t = 12.5 s, roll = tilt = 0.1 rad, roll_d = 0.2 rad, and the
/// commands sorted by id. The accelerometer (0.3, -0.4, 9.81) lands in `ax ay az`, and two feedbacks
/// of motor 8 then one of motor 7 leave the LATEST value of each motor, sorted by id.
#[cfg(test)]
mod tests {
    use nalgebra::Vector3;
    use prost_types::Timestamp;
    use robomorph::messages::pose_messages::IMUMeasurements;
    use tokio::sync::broadcast;

    use crate::scenarios::report;

    use super::*;

    #[test]
    fn a_row_is_read_back_by_the_report() {
        // 1. a recorder on a fresh file in the system temporary directory
        let path= std::env::temp_dir().join(format!("polymorph_recorder_test_{}.csv", std::process::id()));
        let _= fs::remove_file(&path);
        let mut recorder= Recorder::new("recorder".to_string(), &path, "roll_step").unwrap();
        recorder.set_period_from_freq(100);
        // 2. its four channels
        let (telemetry, _)= broadcast::channel(5);
        let (setpoints, _)= broadcast::channel(1);
        let (commands, _)= broadcast::channel(1);
        let (feedbacks, _)= broadcast::channel(4);
        recorder.set_receiver(telemetry.subscribe());
        recorder.set_setpoint_receiver(setpoints.subscribe());
        recorder.set_command_receiver(commands.subscribe());
        recorder.set_feedback_receiver(feedbacks.subscribe());
        // 3. one pass worth of messages, then one pass of the recorder
        let mut pose= Pose::default();
        pose.timestamp= Some(Timestamp { seconds: 12, nanos: 500_000_000 });
        pose.orientation= Some(UnitQuat::from(UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.1)));
        pose.imu_measurement= Some(IMUMeasurements { l_accel: Some(Vec3::new(0.3, -0.4, 9.81)), ..Default::default() });
        assert!(telemetry.send(AnyMessage::PoseState(pose)).is_ok());
        let mut setpoint= Pose::default();
        setpoint.orientation= Some(UnitQuat::from(UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.2)));
        assert!(setpoints.send(setpoint).is_ok());
        assert!(commands.send(AnyMessage::MotorCommands(vec![MotorCommand { id: 8, setpoint_value: 6.0, ..Default::default() },
                                                            MotorCommand { id: 7, setpoint_value: 5.0, ..Default::default() }])).is_ok());
        for (id, value) in [(8, 6.1), (7, 4.9), (8, 6.2)] {
            assert!(feedbacks.send(MotorFeedBack { id, current_value: value, ..Default::default() }).is_ok());
        }
        recorder.exec(&None, Duration::from_millis(10));
        drop(recorder);
        // 4. read back by the report's parser
        let text= fs::read_to_string(&path).unwrap();
        let _= fs::remove_file(&path);
        let recording= report::parse(&text).unwrap();
        assert_eq!((recording.scenario.as_str(), recording.period_s, recording.rows.len()), ("roll_step", 0.01, 1));
        let row= &recording.rows[0];
        assert!((row.t_sim_s - 12.5).abs() < 1e-12);
        assert!((row.roll.unwrap() - 0.1).abs() < 1e-12 && (row.tilt.unwrap() - 0.1).abs() < 1e-12);
        assert!((row.roll_d.unwrap() - 0.2).abs() < 1e-12);
        // 5. the raw fields: accelerometer, commands, and the latest feedback of each motor
        let fields: Vec<&str>= text.lines().nth(2).unwrap().split(',').collect();
        let column= |name: &str| fields[COLUMNS.iter().position(|c| *c == name).unwrap()];
        assert_eq!((column("ax"), column("ay"), column("az")), ("0.3", "-0.4", "9.81"));
        assert_eq!(column("motor_cmds"), "7=5|8=6");
        assert_eq!(column("motor_fb"), "7=4.9|8=6.2");
    }
}
