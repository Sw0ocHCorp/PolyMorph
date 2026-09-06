//! robomorph: vehicle-agnostic control stack of the PolyMorph project.
//!
//! * `messages`: the message catalogue (`AnyMessage`, wire tags, `Vec3` / `UnitQuat`) and the
//!   protobuf structs exchanged between processes and with the hardware (`Pose`, IMU / GNSS /
//!   lidar samples, `MotorModel`, `MotorFeedBack`, `MotorCommand`, `WorkVec`).
//! * `core`: the scheduler (`Process`, `ProcessesChain`, `Scheduler`) that runs the processes at
//!   their period and chains them through the pipe.
//! * `communications`: the `HardwareInterface` trait, the wire codec and the UDP implementation.
//! * `control`: the scalar `PIDController`, the gamepad input, and the motion stack:
//!   `AttitudeController` (pose -> wrench setpoint), `MotorsMixer` (wrench -> per-motor efforts by
//!   control allocation over the motor graph) and `MotorController` (one motor of that graph).
//!
//! The vehicle-specific side (Gazebo bridge, motor graph description, actuator unit conversion)
//! lives in the `gazebo_vehicles_controller` binary, which wires these pieces together.

pub mod messages;
pub mod core;
pub mod communications;
pub mod control;


/// Placeholder test module: a single empty test that only checks the crate links.
#[cfg(test)]
mod test_utils_functions {
    #[test]
    fn it_works() {
        
    }
}

/// Regression test of `AttitudeController::compute_command_law` on a toy vehicle config.
/// Case 1: identity attitude and setpoint -> the wrench is pure weight compensation
/// (`fz = weight * g = 11.772 N`, no moment). Case 2: the current attitude is rotated by
/// 30 rad (not degrees) about body x -> the moment and the tilted force are checked against
/// full-precision reference values, i.e. values recorded from the implementation rather than
/// derived by hand (a characterisation test).
#[cfg(test)]
mod test_motion_controllers {
    use std::time::Duration;

use chrono::Utc;
use nalgebra::{Matrix3, UnitQuaternion, Vector3};
use prost_types::Timestamp;

use crate::{control::motion::{attitude_controller::AttitudeController, motion_controller::{MotionController, VehicleKinematicConfig}}, core::scheduler::Process, messages::{pose_messages::{IMUMeasurements, Pose}, registered_message::{AnyMessage, UnitQuat, Vec3}}};

use super::*;

    #[test]
    fn test_attitude_controller_computes() {
        let attitude_controller_freq= 250;
        let now= Timestamp {
                        seconds: Utc::now().timestamp(),
                        nanos: Utc::now().timestamp_subsec_nanos() as i32,
                    };
        let vehicle_config= VehicleKinematicConfig {
            com_relative_location: Vec3::new(0.0, 0.0, 0.0),
            error_angular_factor: 0.25,
            error_linear_factor: 0.3,
            error_attitude_factor: 1.0,
            weight: 1.2,
            moments_matrix: Matrix3::from([[0.017, 0.0,    0.0], 
                            [0.0,   0.0239, 0.0], 
                            [0.0,   0.0,    0.0357]]),
        };
        let mut att_controller= AttitudeController::new("AttitudeController".to_string(), vehicle_config, 0.2, 1.0);
        att_controller.set_period_from_freq(attitude_controller_freq);
        let mut input_pose= Pose::default();
        input_pose.orientation= Some(UnitQuat{w: 1.0, x: 0.0, y: 0.0, z: 0.0});
        input_pose.imu_measurement= Some(IMUMeasurements {timestamp: Some(now), l_accel: None, 
                                                            a_velocity: Some(Vec3::new(0.0, 0.0, 0.0)), 
                                                            magnetic_field: None });
        let mut pose_setpoint= Pose::default();
        pose_setpoint.orientation= Some(UnitQuat{w: 1.0, x: 0.0, y: 0.0, z: 0.0});
        pose_setpoint.imu_measurement= Some(IMUMeasurements {timestamp: Some(now), l_accel: None, 
                                                            a_velocity: Some(Vec3::new(0.0, 0.0, 0.0)), 
                                                            magnetic_field: None });
        println!("Vehicle Moments Matrix:\n{:?}", vehicle_config.moments_matrix);
        println!("Time Constant:\n{:?}", att_controller.get_time_constant());
        let result= att_controller.compute_command_law(Some(AnyMessage::PoseState(input_pose.clone())), 
                                                                    Some(AnyMessage::PoseState(pose_setpoint.clone())), 
                                                                    Duration::from_millis(20), false);
        if let Some(result_wrench) = result && let AnyMessage::VehicleWrench(wrench)= result_wrench {
            // cas 1 : identité
            assert!((wrench.fz - 11.772).abs() < 1e-6);
            assert!(wrench.mx.abs() < 1e-9 && wrench.mx.is_finite());
            /*println!("Input Pose= {:?}", input_pose);
            println!("Pose Setpoint= {:?}", pose_setpoint);
            println!("Result Wrench= {:?}", result_wrench);*/
        } else {
            
        }
        input_pose.orientation= Some(UnitQuat::from(UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 30.0)));
        
        input_pose.imu_measurement= Some(IMUMeasurements {timestamp: Some(now), l_accel: None, 
                                                            a_velocity: Some(Vec3::new(0.0, 0.0, 0.0)), 
                                                            magnetic_field: None });
        println!("Vehicle Moments Matrix:\n{:?}", vehicle_config.moments_matrix);
        println!("Time Constant:\n{:?}", att_controller.get_time_constant());
        let result= att_controller.compute_command_law(Some(AnyMessage::PoseState(input_pose.clone())), 
                                                                    Some(AnyMessage::PoseState(pose_setpoint.clone())), 
                                                                    Duration::from_millis(20), false);
        if let Some(result_wrench) = result && let AnyMessage::VehicleWrench(wrench)= result_wrench {
            assert!((wrench.mx - 0.6017687777566212).abs() < 1e-6);
            assert!((wrench.fy - (-11.631108278821168)).abs() < 1e-6);
            /*println!("Input Pose= {:?}", input_pose);
            println!("Pose Setpoint= {:?}", pose_setpoint);
            println!("Result Wrench= {:?}", result_wrench);*/
        }
    }
}