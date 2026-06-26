pub mod vehicle_controllers;

use core::time;
use std::{ops::Add, os, sync::{Arc, Mutex}, thread::sleep, time::{Duration, Instant}, u8};

use chrono::Utc;
use gz::{msgs::{actuators::Actuators, double::Double, image::Image, imu::IMU, laserscan::LaserScan, model::Model, navsat::NavSat, quaternion::Quaternion}, transport::Node};
use prost::Message;
use robomorph::{communications::{interface::HardwareInterface, udp_interface::UdpInterface}, core::scheduler::Scheduler, messages::{lidar_messages::{LidarMeasurements, Ray}, motor_messages::{MotorCommandType, MotorFeedBack, MotorStatus}, pose_messages::{GNSSMeasurement, IMUMeasurements}}};
use crate::vehicle_controllers::{osprai_controller::{OspraiController}, vehicle_controller::VehicleController};


fn main() {
    let mut udp_interface= UdpInterface::new("127.0.0.1", 8080, "127.0.0.1", 8090, 10);
    let mut osprai_controller= OspraiController::default();
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

