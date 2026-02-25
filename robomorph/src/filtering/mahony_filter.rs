use num_quaternion::{Quaternion, UnitQuaternion};

use crate::{control::pid::PIDController, core::utils, positionning::pose::IMUData};

pub struct MahonyFilter {
    controllers: [PIDController; 3],
    delta_time: f64,
    quaternion_orientation: UnitQuaternion<f64>,
}

impl MahonyFilter {
    pub fn new(p: f64, i:f64, d: f64, max_integral_error: f64, delta_time: f64, min_threshold_error: f64, leak_factor: f64) -> Self {
        let init_orientation= UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0);
        return Self { controllers: [PIDController::new(p, i, d, max_integral_error, min_threshold_error, leak_factor), 
                                        PIDController::new(p, i, d, max_integral_error, min_threshold_error, leak_factor), 
                                        PIDController::new(p, i, d, max_integral_error, min_threshold_error, leak_factor)], 
                        delta_time: delta_time, quaternion_orientation: init_orientation};
    }

    pub fn estimate_orientation(&mut self, imu_data: IMUData) -> [f64; 3] {
        /*
         * ==> Computing ROLL & PITCH error <==
         * Error is between
         * the robot orientation at T-1 nd the current Accelerometer measurement at T
         */
        //Normalize Accelerometer: We only care about the direction of gravity, 
        //not the magnitude of the force (G-force).
        let mut accel_measurement= imu_data.accel.to_vec();
        let norm= f64::sqrt(accel_measurement[0].powf(2.0) + accel_measurement[1].powf(2.0) + accel_measurement[2].powf(2.0));
        for i in 0..accel_measurement.len() {
            accel_measurement[i] /= norm;
        }
        //Estimate the gravity vector at the current orientation 
        //  What the gravity vector should be at the current orientation
        let q= self.quaternion_orientation.clone().into_quaternion();
        let expected_gravity_vec= vec![
                                            2.0*(q.x*q.z - q.w*q.y),
                                            2.0*(q.w*q.x + q.y*q.z),
                                            q.w.powf(2.0) - q.x.powf(2.0) - q.y.powf(2.0) + q.z.powf(2.0)
                                        ];
        //Computing the cross product between the measured gravity vector and the expected(theoric) gravity vector
        //  To get the error between them
        let  error_accel = utils::compute_cross_product(accel_measurement, expected_gravity_vec);
        
        /*
         * ==> Computing HEADING error <==
         * Error is between
         * the robot orientation at T-1 nd the current Magnetometer measurement at T
         */
        let mut magnetic_field= imu_data.magnetic_field.to_vec();
        let norm= f64::sqrt(magnetic_field[0].powf(2.0) + magnetic_field[1].powf(2.0) + magnetic_field[2].powf(2.0));
        for i in 0..magnetic_field.len() {
            magnetic_field[i] /= norm;
        }
        /* 
         * Pass the magnetic field in body frame (from the Magnetometer measurements)
         * To the world frame
         * The Magnetic field from the Magnetometer is express in Body Frame:
         * - X points toward the robot forward 
         * - Y points toward the left of the robot
         * - Z points toward the top of the robot
         * So this Local Magnetic Field gives a local North that rotate in the same way as the robot 
         * rotate this magnetic field to the world frame allows to get the True North because the world frame axis doesn't rotate relative to the robot
         * World Frame:
         * - X and Y parallel to the ground
         * - Z points toward the sky
        */
        let rotated_vec= self.quaternion_orientation.rotate_vector([magnetic_field[0], magnetic_field[1], magnetic_field[2]]);
        // rotated_vec = [hx, hy, hz] in world frame
        // horizontal magnitude
        let bx = (rotated_vec[0]*rotated_vec[0] + rotated_vec[1]*rotated_vec[1]).sqrt();
        // vertical component
        let bz = rotated_vec[2];
        /*
         * Magnetic Field in World Frame
         *  The Y component is removed to simplify the calculus
         *  It allow to consider the magnetic field as a plan (XZ axis) 
         *  knowing that the magnetic field is express as [1, 0, 0] so we need the X axis 
         *  and the Magnometer help to estimate the heading (angle around the Z axis), so we need the Z axis
         */
        let mag_ref_world = vec![bx, 0.0, bz];
        // then rotate back into body frame
        // it give the local North without taking to account the pitch angle of the robot (that give a wrong North)
        let mut expected_local_field = self.quaternion_orientation.conj().rotate_vector([
            mag_ref_world[0],
            mag_ref_world[1],
            mag_ref_world[2],
        ]).to_vec();
        let norm= f64::sqrt(expected_local_field[0].powf(2.0) + expected_local_field[1].powf(2.0) + expected_local_field[2].powf(2.0));
        for i in 0..expected_local_field.len() {
            expected_local_field[i] /= norm;
        }
        //Computing the cross product between the measured magnetic field vector and the True local North (without pitch angle error)
        //  To get the error between them
        let error_magnetic= utils::compute_cross_product(magnetic_field.clone(), expected_local_field);
        let mut error= Vec::new();
        for i in 0..3 {
            error.push(error_accel[i] + error_magnetic[i]);
        }

        /*
         * Computing the angular velocity of the robot
         * Based on Gyrometer measurement because 
         * it measure the angular velocity of the robot but it drift over time because of the measurements errors integration
         * Based also on the PI output that try to reduce the error of the orientation estimations caused by the Gyro drift
         */
        let correct_angular_velocity= [imu_data.gyro[0] + self.controllers[0].compute_output_value_from_error(error[0], self.delta_time), 
                                                    imu_data.gyro[1] + self.controllers[1].compute_output_value_from_error(error[1], self.delta_time),
                                                    imu_data.gyro[2] + self.controllers[2].compute_output_value_from_error(error[2], self.delta_time)];
        /*
         * Computing the rate of change Tthe orientation changes expressed in the Quaternion space) 
            1/2 comes due to the quaternion geometry characteristics (Quaternions cover 720° on 4 dimensions to describe orientations in 360° 3D space)
         */
        let rate_of_change= q*Quaternion::new(0.0, correct_angular_velocity[0], correct_angular_velocity[1], correct_angular_velocity[2])*0.5;
        //Computing the new orientation of the robot (after the rotation at current quaternion orientation)
        if let Some(new_current_orientation)= Quaternion::new(q.w + rate_of_change.w*self.delta_time, q.x + rate_of_change.x*self.delta_time,q.y + rate_of_change.y*self.delta_time, q.z + rate_of_change.z*self.delta_time).normalize() {
            self.quaternion_orientation= new_current_orientation
        }
        //Get the new euler orientation from the current orientation in Quaternion Frame
        let euler_angles= self.quaternion_orientation.to_euler_angles();
        //println!("Gyr,o Vector= {:?}", [imu_data.gyro[0], imu_data.gyro[1], imu_data.gyro[2]]);
        //println!("Global error {:?} Estimated Orientation= {:?}", error, [euler_angles.roll, euler_angles.pitch, euler_angles.yaw]);
        return [euler_angles.roll, euler_angles.pitch, euler_angles.yaw];
    }
}