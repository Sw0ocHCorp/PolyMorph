pub mod vehicle_controllers;
use nalgebra::Matrix3;
use robomorph::{communications::{interface::HardwareInterface, udp_interface::UdpInterface}, control::{joystick::{joystick_controller::JoyStickController, xbox_pad_controller::XboxPadControl}, motion::{attitude_controller::AttitudeController, motion_controller::{MotionController, VehicleKinematicConfig}, motors_mixer::MotorsMixer}}, core::scheduler::{Process, Scheduler}, messages::registered_message::Vec3};
use tokio::sync::broadcast::{Sender, channel};
use crate::vehicle_controllers::{osprai_controller::{OspraiController}, vehicle_controller::VehicleController};


/// Wiring of the simulation binary. Rules this wiring obeys (each learnt from a bug, see
/// `docs/src/validation/lecons.md`):
///   - every stage of the control loop lives in the SAME chain (one clock) and is registered in
///     producer -> consumer order: osprai (publishes the pose) -> remote -> attitude -> mixer. Two
///     chains at the same nominal frequency on two threads drift in phase and produce empty /
///     double ticks (beat);
///   - all stages run at the same frequency (100 Hz): the time-scale separation of the cascade
///     lives in the time constants, not in the tick rates. tau_attitude = 0.2 s >= 10 * T;
///   - the wrench goes from the attitude stage to the mixer through the scheduler PIPE (the
///     attitude stage has no wrench sender), which is reliable only because both run in the same
///     pass; every other link uses a broadcast channel;
///   - `VehicleKinematicConfig` holds the TOTAL mass and inertia of the vehicle (base + arms +
///     rotors), taken from the SDF; the attitude time constant is a constructor argument of the
///     controller, not a field of the vehicle config.
fn main() {
    let vehicle_config= VehicleKinematicConfig {
        com_relative_location: Vec3::new(0.0, 0.0, 0.0),
        error_angular_factor: 0.25,
        error_linear_factor: 0.3,
        error_attitude_factor: 1.0,
        weight: 1.2,
        moments_matrix: Matrix3::from([[0.017, 0.0,    0.0], 
                         [0.0,   0.0239, 0.0], 
                         [0.0,   0.0,    0.0357]])
    };
    let mut remote_controller= XboxPadControl::new("remote_controller".to_string(), 5);
    let mut udp_interface= UdpInterface::new("127.0.0.1", 8080, "127.0.0.1", 8090, 10);
    let mut attitude_controller= AttitudeController::new("attitude_controller".to_string(), vehicle_config, 0.2, 1.0);
    let mut motor_mixer= MotorsMixer::new("motor_mixer".to_string(), vehicle_config.clone());
    let mut osprai_controller= OspraiController::default();
    remote_controller.connect();
    remote_controller.set_period_from_freq(100);
    osprai_controller.set_period_from_freq(100);
    attitude_controller.set_period_from_freq(100);
    motor_mixer.set_period_from_freq(100);

    //channel used to send  and receive data relative to the vehicle
    //input data for motion control
    //let (pad_controls_sender, _)= channel(1);
    // Broadcast channels (tokio): non-blocking; a receiver that falls behind gets `Lagged`
    // ("messages skipped, carry on"). Capacities are small on purpose: only the latest matters.
    let (vehicle_telemetry_sender, _)= channel(5);
    let (mc_setpoint_sender, _)= channel(1);
    let (command_sender, _)= channel(1);
    let (motor_config_sender, _)= channel(4);
    let (motor_feedback_sender, _)= channel(4);

    //remote_controller.set_sender(pad_controls_sender);

    //the vehicle controller send data relative to the vehicle state
    osprai_controller.set_sender(vehicle_telemetry_sender.clone());
    osprai_controller.set_motor_config_sender(motor_config_sender.clone());
    osprai_controller.set_motor_feedback_sender(motor_feedback_sender.clone());
    osprai_controller.set_motion_setpoint_sender(mc_setpoint_sender.clone());

    //the attitude controller receive data from the vehicle controller: pose (current pose)
    attitude_controller.set_receiver(vehicle_telemetry_sender.subscribe());
    attitude_controller.set_setpoint_receiver(mc_setpoint_sender.subscribe());
    /*attitude_controller.set_receiver(vehicle_telemetry_sender.subscribe());
    //the attitude controller receive data from the vehicle controller: vehicle attitude
    attitude_controller.set_input_data_receiver(vehicle_telemetry_sender.subscribe());
    //the attitude controller send attitude setpoint
    attitude_controller.set_sender(mc_setpoint_sender.clone());*/
    
    //the motor mixer receive data from the attitude controller: wrench setpoint
    motor_mixer.set_motor_config_receiver(motor_config_sender.subscribe());
    motor_mixer.set_motor_feedback_receiver(motor_feedback_sender.subscribe());
    //the motor mixer receive data from the vehicle controller: vehicle state to build wrench
    //motor_mixer.set_input_data_receiver(vehicle_telemetry_sender.subscribe());
    //the motor mixer send the motors command
    motor_mixer.set_sender(command_sender.clone());

    //the vehicle controller receive data from the motor mixer: motors command
    osprai_controller.set_receiver(command_sender.subscribe());

    
    // the interface is wired like any other Process now that connect_process() is gone.
    // connect() fails if this receiver is missing.
    udp_interface.set_sender(vehicle_telemetry_sender.clone());
    udp_interface.set_receiver(vehicle_telemetry_sender.subscribe());

    //osprai_controller.set_pid_controller(PIDController::new(0.5, 1.0, 0.0, 0.0, f64::INFINITY));
    osprai_controller.start_listening_topics();

    let mut scheduler = Scheduler::new();
    // Registration order = execution order within a pass. Keep osprai first (fresh pose for the
    // attitude) and mixer right after attitude (the pipe carries the wrench within the pass).
    scheduler.register_process(Box::new(osprai_controller));
    scheduler.register_process(Box::new(remote_controller));
    scheduler.register_process(Box::new(attitude_controller));
    scheduler.register_process(Box::new(motor_mixer));
    //scheduler.register_process(Box::new(attitude_controller));
    scheduler.register_interface(Box::new(udp_interface));
    scheduler.start_all_interfaces();
    scheduler.start_all_side_chains();
    loop {
        scheduler.run_main_chain();
    }   
}

