pub mod vehicle_controllers;
pub mod scenarios;
use std::path::Path;

use nalgebra::Matrix3;
use robomorph::{communications::udp_interface::UdpInterface, control::{joystick::{joystick_controller::JoyStickController, xbox_pad_controller::XboxPadControl}, motion::{attitude_controller::AttitudeController, force_to_attitude_resolver::ForceToAttitudeResolver, motion_controller::VehicleKinematicConfig, motors_mixer::MotorsMixer, velocity_controller::VelocityController}}, core::scheduler::{Process, Scheduler}, messages::registered_message::Vec3};
use tokio::sync::broadcast::channel;
use crate::{scenarios::{Cascade, player::ScenarioPlayer, recorder::Recorder}, vehicle_controllers::{osprai_controller::OspraiController, vehicle_controller::VehicleController}};

const USAGE: &str= "usage:
  gazebo_vehicles_controller [--scenario <name>] [--record <file.csv>]    fly a scenario (default: hover)
  gazebo_vehicles_controller report <file.csv>                            judge a recording
See docs/docs/simulation/running-scenarios.mdx.";

/// Wiring of the simulation binary. Rules this wiring obeys (each learnt from a bug, see
/// `docs/docs/pipelines/motor-control/lessons.mdx`):
///   - every stage of the control loop lives in the SAME processes pipeline (one clock) and is
///     registered in producer -> consumer order: osprai (publishes the pose) -> remote -> scenario
///     player (the mission setpoint) -> velocity -> [resolver] -> attitude -> mixer -> recorder. Two
///     processes pipelines at the same nominal frequency on two threads drift in phase and produce
///     empty / double ticks (beat) [P4];
///   - all stages run at the same frequency (100 Hz): the time-scale separation of the cascade
///     lives in the time constants, not in the tick rates. tau_attitude = 0.2 s >= 10 * T;
///   - ONE source of setpoints: the scenario player. The bridge only measures and actuates;
///   - the cascade scenario player -> velocity -> [resolver] -> attitude -> mixer goes through the
///     scheduler PIPE: ONE setpoint `Pose`, which each stage enriches (velocity: the force f_d;
///     resolver: q_d and omega_d), then the wrench. It is reliable only because each producer and
///     its consumer run in the same pass. Every other link uses a broadcast channel. The resolver
///     is there only when the scenario's `cascade` asks for it: any process may stand before the
///     attitude stage, as long as it fills the fields that stage reads;
///   - `VehicleKinematicConfig` holds the TOTAL mass and inertia of the vehicle (base + arms +
///     rotors), taken from the SDF; the attitude time constant is a constructor argument of the
///     controller, not a field of the vehicle config.
/// Stabilize mode: the velocity stage's law is a_d = 0 until brick 5, so its force f_d = (0, 0, m g)
/// is the weight compensation. The resolver is in the cascade only when the scenario asks for it
/// (`Scenario::cascade`): a roll preference with a vertical force is refused by it, by design.
fn main() {
    // 1. the command line
    let args: Vec<String>= std::env::args().skip(1).collect();
    //    1a. `report <file.csv>`: judge a recording and exit; Gazebo is not involved
    if args.first().is_some_and(|arg| arg == "report") {
        let Some(path)= args.get(1) else {
            eprintln!("{}", USAGE);
            std::process::exit(2);
        };
        std::process::exit(scenarios::report::run(path));
    }
    //    1b. otherwise: `--scenario <name>` (default hover) and `--record <file.csv>` (optional),
    //        each flag followed by its value
    let (mut scenario_name, mut record_path)= ("hover".to_string(), None);
    let mut i= 0;
    while i < args.len() {
        match (args[i].as_str(), args.get(i + 1)) {
            ("--scenario", Some(value)) => scenario_name= value.clone(),
            ("--record", Some(value)) => record_path= Some(value.clone()),
            _ => {
                eprintln!("{}", USAGE);
                std::process::exit(2);
            },
        }
        i += 2;
    }
    let Some(scenario)= scenarios::by_name(&scenario_name) else {
        eprintln!("unknown scenario {:?}; available: {}", scenario_name, scenarios::NAMES.join(", "));
        std::process::exit(2);
    };

    // 2. the vehicle, as data [P1]
    let vehicle_config= VehicleKinematicConfig {
        // CoM from the SDF masses: base 1.0 kg at the origin, two arms and two rotors of 0.05 kg each
        // at z = 0.10 m, so z_com = (4 * 0.05 * 0.10) / 1.2 = 1/60 m (0.0167 m). The inertia below
        // is already taken about this point (parallel-axis theorem), so the two now agree.
        com_relative_location: Vec3::new(0.0, 0.0, 1.0 / 60.0),
        error_angular_factor: 0.25,
        error_linear_factor: 0.3,
        error_attitude_factor: 1.0,
        weight: 1.2,
        moments_matrix: Matrix3::from([[0.017, 0.0,    0.0],
                         [0.0,   0.0239, 0.0],
                         [0.0,   0.0,    0.0357]])
    };

    // 3. the processes, all at 100 Hz
    let mut remote_controller= XboxPadControl::new("remote_controller".to_string(), 5);
    let mut udp_interface= UdpInterface::new("127.0.0.1", 8080, "127.0.0.1", 8090, 10);
    let mut scenario_player= ScenarioPlayer::new("scenario_player".to_string(), scenario.clone());
    let mut velocity_controller= VelocityController::new("velocity_controller".to_string(), vehicle_config);
    let mut attitude_controller= AttitudeController::new("attitude_controller".to_string(), vehicle_config, 0.2, 1.0);
    let mut motor_mixer= MotorsMixer::new("motor_mixer".to_string(), vehicle_config.clone());
    let mut osprai_controller= OspraiController::default();
    //    3a. the recorder, only when asked for. It refuses an existing file: better to stop here
    //        than to fly a scenario whose recording is lost
    let mut recorder= None;
    if let Some(path)= record_path {
        match Recorder::new("recorder".to_string(), Path::new(&path), scenario.name) {
            Ok(new_recorder) => recorder= Some(new_recorder),
            Err(error) => {
                eprintln!("cannot create the recording {}: {} (an existing file is never overwritten)", path, error);
                std::process::exit(2);
            },
        }
    }
    remote_controller.connect();
    remote_controller.set_period_from_freq(100);
    osprai_controller.set_period_from_freq(100);
    scenario_player.set_period_from_freq(100);
    velocity_controller.set_period_from_freq(100);
    attitude_controller.set_period_from_freq(100);
    motor_mixer.set_period_from_freq(100);
    if let Some(recorder)= &mut recorder {
        recorder.set_period_from_freq(100);
    }

    // 4. the channels. Broadcast channels (tokio): non-blocking; a receiver that falls behind gets
    //    `Lagged` ("messages skipped, carry on"). Capacities are small on purpose: only the latest
    //    matters.
    let (vehicle_telemetry_sender, _)= channel(5);
    let (mc_setpoint_sender, _)= channel(1);
    let (command_sender, _)= channel(1);
    let (motor_config_sender, _)= channel(4);
    let (motor_feedback_sender, _)= channel(4);

    // 5. the wiring
    //    5a. the vehicle bridge publishes the measured state, the motor tree and the feedbacks, and
    //        applies the motor commands
    osprai_controller.set_sender(vehicle_telemetry_sender.clone());
    osprai_controller.set_motor_config_sender(motor_config_sender.clone());
    osprai_controller.set_motor_feedback_sender(motor_feedback_sender.clone());
    osprai_controller.set_receiver(command_sender.subscribe());
    //    5b. the scenario player reads the simulated time from the telemetry and publishes the mission
    //        setpoint (attitude preference, desired velocity)
    scenario_player.set_receiver(vehicle_telemetry_sender.subscribe());
    scenario_player.set_setpoint_sender(mc_setpoint_sender.clone());
    //    5c. the velocity stage needs no channel: the setpoint comes down the pipe, it adds the force
    //        and hands it on (a_d = 0 until brick 5). The resolver exists only when the scenario's
    //        cascade includes it: it reads the motor tree on the configuration channel, to derive
    //        the force subspace it can reach (no idle subscriber otherwise)
    let resolver= (scenario.cascade == Cascade::WithResolver).then(|| {
        let mut resolver= ForceToAttitudeResolver::new("force_to_attitude_resolver".to_string(), vehicle_config);
        resolver.set_period_from_freq(100);
        resolver.set_motor_config_receiver(motor_config_sender.subscribe());
        resolver
    });
    //    5d. the attitude stage reads the pose; its setpoint comes from the pipe only, and its wrench
    //        goes to the mixer by the pipe
    attitude_controller.set_receiver(vehicle_telemetry_sender.subscribe());
    //    5e. the mixer reads the motor tree and the feedbacks, and publishes the motor commands
    motor_mixer.set_motor_config_receiver(motor_config_sender.subscribe());
    motor_mixer.set_motor_feedback_receiver(motor_feedback_sender.subscribe());
    motor_mixer.set_sender(command_sender.clone());
    //    5f. the recorder listens to the pose, the mission setpoint, the commands and the motor feedbacks
    if let Some(recorder)= &mut recorder {
        recorder.set_receiver(vehicle_telemetry_sender.subscribe());
        recorder.set_setpoint_receiver(mc_setpoint_sender.subscribe());
        recorder.set_command_receiver(command_sender.subscribe());
        recorder.set_feedback_receiver(motor_feedback_sender.subscribe());
    }
    //    5g. the interface is wired like any other Process now that connect_process() is gone.
    //        connect() fails if this receiver is missing.
    udp_interface.set_sender(vehicle_telemetry_sender.clone());
    udp_interface.set_receiver(vehicle_telemetry_sender.subscribe());

    osprai_controller.start_listening_topics();

    // 6. the scheduler. Registration order = execution order within a pass [P4]: osprai first (fresh
    //    pose), then the cascade scenario player -> velocity -> [resolver] -> attitude -> mixer, each
    //    right after its producer (the pipe carries the setpoint, enriched stage after stage, then the
    //    wrench, within the pass), the recorder last (it sees the whole pass). The resolver, like the
    //    recorder, is registered only when it exists.
    let mut scheduler = Scheduler::new();
    scheduler.register_process(Box::new(osprai_controller));
    scheduler.register_process(Box::new(remote_controller));
    scheduler.register_process(Box::new(scenario_player));
    scheduler.register_process(Box::new(velocity_controller));
    if let Some(resolver)= resolver {
        scheduler.register_process(Box::new(resolver));
    }
    scheduler.register_process(Box::new(attitude_controller));
    scheduler.register_process(Box::new(motor_mixer));
    if let Some(recorder)= recorder {
        scheduler.register_process(Box::new(recorder));
    }
    scheduler.register_interface(Box::new(udp_interface));
    scheduler.start_all_interfaces();
    scheduler.start_all_side_pipelines();
    println!("[SCENARIO] {} ({}, cascade {:?}) ready: press Play in Gazebo", scenario.name, scenario.description, scenario.cascade);
    loop {
        scheduler.run_main_pipeline();
    }
}
