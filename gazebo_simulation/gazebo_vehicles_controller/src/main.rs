pub mod vehicle_controllers;
use robomorph::{communications::{interface::HardwareInterface, udp_interface::UdpInterface}, control::motion::motion_controller::VehicleKinematicConfig, core::scheduler::Scheduler, messages::registered_message::Vec3};
use crate::vehicle_controllers::{osprai_controller::{OspraiController}, vehicle_controller::VehicleController};


fn main() {
    let mut udp_interface= UdpInterface::new("127.0.0.1", 8080, "127.0.0.1", 8090, 10);
    let mut osprai_controller= OspraiController::default();
    /*VehicleKinematicConfig { error_linear_factor: 0.3,
                                                            error_angular_factor: 0.25,
                                                            vehicle_weight: 1.2,
                                                            thruster_constant: 1.2e-5,
                                                            max_thruster_rot_vel: 1200.0,
                                                            thruster_law: 2
                                                        }*/
    let vehicle_config= VehicleKinematicConfig {
        com_relative_location: Vec3::new(0.0, 0.0, 0.0),
        error_angular_factor: 0.25,
        error_linear_factor: 0.3,
        error_attitude_factor: 1.0,
        weight: 1.2,
        moments_matrix: [[0.017, 0.0,    0.0], 
                         [0.0,   0.0239, 0.0], 
                         [0.0,   0.0,    0.0357]],
    };
    osprai_controller.set_vehicle_params(vehicle_config);
    //osprai_controller.set_pid_controller(PIDController::new(0.5, 1.0, 0.0, 0.0, f64::INFINITY));
    osprai_controller.start_listening_topics();

    udp_interface.connect_process(&mut osprai_controller);

    let mut scheduler = Scheduler::new();
    scheduler.register_process(Box::new(osprai_controller), 50);
    scheduler.register_interface(Box::new(udp_interface));
    scheduler.start_all_interfaces();
    scheduler.start_all_side_chains();
    loop {
        scheduler.run_main_chain();
    }   
}

