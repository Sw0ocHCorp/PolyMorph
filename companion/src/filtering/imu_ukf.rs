use std::f64;

use faer::{Col, Mat, col, mat, matrix_free::LinOp, traits::math_utils::zero};
use num_quaternion::{Q64, Quaternion, UnitQuaternion};
use robomorph::filtering::kalman_filter::{self, KalmanMeasurements, UnscentedKalmanFilter};

const REF_WORLD_MAGNETIC_FIELD: [f64; 3] = [1.0, 0.0, 0.0];
const REF_REST_ACCEL: [f64; 3]= [0.0, 0.0, 1.0];

pub struct OrientationUKF {
    //The state estimation
    current_state: Col<f64>,
    //The covariance of the state
    //  Model the uncertainty of all the variables of the state
    //  The diagonal values model the variance of the state variables
    //  The off-diagonal values model the uncertainties of linked variables
    state_covariance: Mat<f64>,
    //The noise of the ref sensors measurements
    measurements_noise: Mat<f64>,
    //The noise of the state due to fluctuations during it's lifetime
    state_process_noise: Mat<f64>,
    //The Sigma Point spread
    //  The factor used to adjust the size of the distribution 
    //  How far the sigma points will be from the mean(current state, predicted state, estimated state)
    spread_factor: f64,
    //
    prior_knowledge: f64,
    
}

impl OrientationUKF {
    pub fn new (init_state: Col<f64>, measurements_noise: Mat<f64>, state_process_noise: Mat<f64>, 
                                                    spread_factor:f64, prior_knowledge: f64) -> Self {
        println!("State process noise= {:?} | Measurement noise= {:?}", state_process_noise, measurements_noise);
        println!("Spread factor= {:?} | Prior knowledge= {:?}", spread_factor, prior_knowledge);
        return Self { current_state: init_state.clone(), state_covariance: Mat::<f64>::identity(3, 3), 
                        measurements_noise, state_process_noise, spread_factor, prior_knowledge};
    }

    fn compute_mean_quaternion(&self, sigma_points: Vec<Col<f64>>) -> Col<f64> {
        println!("Sigma points= {:?}", sigma_points);
        let nrows= 3.0;
        let mut mean_point= Col::<f64>::zeros(nrows as usize);
        //The scale factor determine how far from the mean point the sigma point is placed
        //  IF the spread factor is small, the sigma points will be close to the mean point(center point)
        let scale_factor= self.get_spread_factor().powf(2.0)*(nrows + kalman_filter::KAPPA) - nrows;
        let scale= nrows + scale_factor;
        println!("Scale= {:?}", scale);
        let mut mean_point_mat= Mat::<f64>::zeros(sigma_points[0].nrows(), sigma_points[0].nrows());
        //Compute the mean point / state
        for i in 0..sigma_points.len() {
            //Compute the weights for the given point (weights for the center point is differents than the other points of the distribution)
            let mut weight= 1.0/(2.0*scale);
            if i == 0 {
                weight= (scale_factor / scale);
            }
            mean_point_mat += weight * &sigma_points[i] * &sigma_points[i].transpose()
        }
        println!("Mean point Mat= {:?}", mean_point_mat);
        if let Ok(eigen_decomp)  = mean_point_mat.self_adjoint_eigen(faer::Side::Lower) {
            println!("Eigen Decomposition= {:?}", eigen_decomp);
            let eigen_vectors= eigen_decomp.U();
            println!("Eigen Vectors= {:?}", eigen_vectors);
            let eigen_values= eigen_decomp.S();
            println!("Eigen Values= {:?}", eigen_values);
            let mut max_val= f64::MIN;
            for i in 0..eigen_values.nrows() {
                if eigen_values[i] > max_val {
                    max_val= eigen_values[i];
                    mean_point= eigen_vectors.col(i).to_owned()
                }
            }
        }
        let euclid_norm= mean_point.norm_l2();
        println!("Mean point not normalized= {:?} | Point norm= {:?}", mean_point, euclid_norm);
        if euclid_norm > 1e-12 {
            println!("Mean point normalized= {:?}", &mean_point / euclid_norm);
            return mean_point / euclid_norm;
        } else {
            // Fallback if something went wrong (return identity)
            println!("Mean point normalized= {:?}", col![1.0, 0.0, 0.0, 0.0]);
            return col![1.0, 0.0, 0.0, 0.0];
        }
    }
    
    /*fn compute_mean_sensor_measurements(&self, sigma_points: Vec<Col<f64>>) -> Col<f64> {
        let nrows= self.state_covariance.nrows() as f64;
        let mut mean_point= Col::<f64>::zeros(sigma_points[0].nrows());
        //The scale factor determine how far from the mean point the sigma point is placed
        //  IF the spread factor is small, the sigma points will be close to the mean point(center point)
        let scale_factor= self.get_spread_factor().powf(2.0)*(nrows + kalman_filter::KAPPA) - nrows;
        let scale= nrows + scale_factor;
        println!("Scale= {:?}", scale);
        //Compute the mean point / state
        for i in 0..sigma_points.len() {
            //Compute the weights for the given point (weights for the center point is differents than the other points of the distribution)
            let mut weight= 0.5 * (1.0/(sigma_points.len() as f64-1.0));
            if i == 0 {
                weight= 0.5
            }
            //Computation of the mean point / state with a weighted mean computation
            mean_point+= weight * &sigma_points[i];
            println!("Mean point= {:?} after processing sigma point= {:?}", mean_point, sigma_points[i]);
        }
        return mean_point;
    }*/

    fn compute_mean_sensor_measurements(&self, sigma_points: Vec<Col<f64>>) -> Col<f64> {
        let nrows= self.state_covariance.nrows() as f64;
        let mut mean_point= Col::<f64>::zeros(sigma_points[0].nrows());
        //The scale factor determine how far from the mean point the sigma point is placed
        //  IF the spread factor is small, the sigma points will be close to the mean point(center point)
        let scale_factor= self.get_spread_factor().powf(2.0)*(nrows + kalman_filter::KAPPA) - nrows;
        let scale= nrows + scale_factor;
        println!("Scale= {:?}", scale);
        //Compute the mean point / state
        for i in 0..sigma_points.len() {
            //Compute the weights for the given point (weights for the center point is differents than the other points of the distribution)
            let mut weight= 1.0/(2.0*scale);
            if i == 0 {
                weight= (scale_factor / scale);
            }
            println!("Weight= {:?}", weight);
            //Computation of the mean point / state with a weighted mean computation
            mean_point+= weight * &sigma_points[i];
            println!("Mean point= {:?} after processing sigma point= {:?}", mean_point, sigma_points[i]);
        }
        return mean_point;
    }

    pub fn estimate_true_state(&mut self, meas: KalmanMeasurements) -> Col<f64> {
        println!("Init State= {:?} | Sensor Input= {:?} | State Covariance= {:?}", self.current_state, meas.input_sensor_measurements, self.state_covariance);
        let (predicted_state, state_covariance)= self.predict(&self.current_state, &meas.input_sensor_measurements, meas.delta_time, &self.state_covariance);
        self.current_state= predicted_state.clone();
        println!("Predicted State= {:?} | State Covariance= {:?}", self.current_state, state_covariance);
        let (true_state, update_state_cov)= self.update_prediction(&predicted_state, &meas.ref_sensor_measurements, &state_covariance);
        self.state_covariance= update_state_cov;
        self.current_state= true_state.clone();
        println!("Updated State= {:?} | Ref Sensor= {:?} | State Covariance= {:?}", self.current_state, meas.ref_sensor_measurements, self.state_covariance);
        return predicted_state;
    }
}

impl UnscentedKalmanFilter for OrientationUKF {
    fn apply_transition_function(state: &Col<f64>, input_measurements: &Col<f64>, delta_time: f64) -> Col<f64> {
        //skew-symetric value that model the effect of the angular velocity from the gyro, to the orientation of the robot (in quaternion space)
        let quat_rate_mat= mat![[0.0, -*input_measurements.get(0), -*input_measurements.get(1), -*input_measurements.get(2)],
                                                        [*input_measurements.get(0), 0.0, *input_measurements.get(2), -*input_measurements.get(1)],
                                                        [*input_measurements.get(1), -*input_measurements.get(2), 0.0, *input_measurements.get(0)],
                                                        [*input_measurements.get(2), *input_measurements.get(1), -*input_measurements.get(0), 0.0]];
        let new_state= state + (delta_time/2.0)*quat_rate_mat*state;
        //delta/2 to compensate the rotation representation of the quaternion (rotations applied twice)
        match Q64::new(*new_state.get(3), *new_state.get(0), *new_state.get(1), *new_state.get(2)).normalize() {
            Some(quat) => {
                let q= quat.as_quaternion();
                return col![q.x, q.y, q.z, q.w];
            },
            None => return Col::<f64>::zeros(state.nrows()),
        }
    }

    fn apply_ref_measurements_function(state: Col<f64>, measurements_vec_size: usize) -> Col<f64> {
        match Q64::new(*state.get(3), *state.get(0), *state.get(1), *state.get(2)).normalize() {
            Some(quat_state) => {
                let q= quat_state.as_quaternion();
                let expected_accel_vec= quat_state.conj().rotate_vector(REF_REST_ACCEL);
                let expected_magnetic_field= quat_state.conj().rotate_vector(REF_WORLD_MAGNETIC_FIELD);           
                return col![expected_accel_vec[0], expected_accel_vec[1], expected_accel_vec[2],
                            expected_magnetic_field[0], expected_magnetic_field[1], expected_magnetic_field[2]];
            },
            None => return Col::<f64>::zeros(measurements_vec_size),
        } 
    }

    fn generate_sigma_points(current_state: &Col<f64>, scale: f64, state_covariance: &Mat<f64>) -> Vec<Col<f64>>{
        //current_state= The center of the distribution
        //The sigma points is representing the distribution of the system state
        let mut sigma_points= vec![current_state.clone()];
        //Performing the cholesky decomposition to retrieve the sigma points from it
        ///!\ The state covariance is 3x3 because it model the uncertainty in a 3D frame
        //      Quaternion is a 4D vector that model an orientation in 3D space (Quaternion have 3 DOFs) 
        if let Ok(cholesky_decomp) = (scale*state_covariance).llt(faer::Side::Lower) && 
                let Some(current_quat)= Q64::new(*current_state.get(0), *current_state.get(1), *current_state.get(2), *current_state.get(3)).normalize() {
            let datas= cholesky_decomp.L();
            println!("Choelsky decomposition= {:?}", cholesky_decomp);
            //Adding sigma points from each columns of the decomposition
            for i in 0..datas.ncols(){
                for sig in [-1.0, 1.0] {
                    //Because we are in the quaternion space, we can't create sigma point with 
                    // mean +/- (sqrt(scale)*cholesky_decomp(i))
                    //So we compute it like: +/-delta_q*mean
                    let error= sig*datas.col(i);
                    let total_error= error.norm_l2();
                    //Computing the distance of the sigma point from the mean point (based on the state covariance that model The state uncertainty)
                    let delta_quat= if total_error < 1e-8  {
                        Q64::ONE
                    } else {
                        let delta_rot= (error / (total_error)) * f64::sin(total_error/2.0);
                        Q64::new(f64::cos(total_error/2.0), *delta_rot.get(0), *delta_rot.get(1), *delta_rot.get(2))
                    };
                    println!("Delta quat= {:?}", delta_quat);
                    //Add the normalized quaternion sigma point to the list of sigma point
                    if let Some(sigma_point_dist)= delta_quat.normalize() {
                        let binding= (&sigma_point_dist * &current_quat);
                        let sigma_point= binding.as_quaternion();
                        println!("Sigma point: Delta quat normalized * Current State(mean state)= {:?}", sigma_point);
                        sigma_points.push(col![sigma_point.w, sigma_point.x, sigma_point.y, sigma_point.z]);
                    }
                }
            }
        }
        return sigma_points;
    }

    fn update_state_covariance_from_predict(&self, mean_point: &Col<f64>, sigma_points: &Vec<Col<f64>>, state_covariance: &Mat<f64>) -> Mat::<f64> {
        let nrows= state_covariance.nrows() as f64;
        let spread_factor= self.get_spread_factor();
        let prior_knowledge= self.get_prior_knowledge();
        let state_process_noise= self.get_state_process_noise();
        //The scale factor determine how far from the mean point the sigma point is placed
        //  IF the spread factor is small, the sigma points will be close to the mean point(center point)
        let scale_factor= spread_factor.powf(2.0)*(nrows + kalman_filter::KAPPA) - nrows;
        let scale= nrows + scale_factor;
        println!("Scale= {:?}", scale);
        //Compute the state covariance (uncertainty in the state variables)
        let mut state_cov= Mat::<f64>::zeros(state_covariance.nrows(), state_covariance.ncols());
        let mean_quat= Q64::new(mean_point[0], mean_point[1], mean_point[2], mean_point[3]);
        println!("Mean quat= {:?}", mean_quat);
        for i in 0..sigma_points.len() {
            let sigma_quat= Q64::new(sigma_points[i][0], sigma_points[i][1], sigma_points[i][2], sigma_points[i][3]);
            println!("Sigma quat= {:?}", sigma_quat);
            let mut weight= 1.0/(2.0*scale);
            if i == 0 {
                weight= (scale_factor / scale) + (1.0 - spread_factor.powf(2.0) + prior_knowledge)
            }
            println!("Weight= {:?}", weight);
            //Compute the "distance" between the sigma points and the center of the distribution
            if let Some(delta_q)= (sigma_quat*mean_quat.conj()).normalize() {
                let diff_quat= delta_q.as_quaternion();
                println!("Error Quaternion= {:?}", diff_quat);
                let vec_part= col![diff_quat.x, diff_quat.y, diff_quat.z];
                println!("Vector par of the Quaternion= {:?}", vec_part);
                let norm= vec_part.norm_l2();
                let error_vec= if norm < 1e-8 {
                    col![0.0, 0.0, 0.0]
                } else {
                    let angle = 2.0 * f64::atan2(norm, diff_quat.w);
                    vec_part * (angle / norm)
                };
                println!("Error vector= {:?}", error_vec);
                state_cov += weight * &error_vec*&error_vec.transpose();
            }
            println!("State covariance (state_cov += weight * &diff.as_mat()*&diff.as_mat().transpose())= {:?}", state_cov);
        }
        state_cov += state_process_noise;
        println!("Final State Covariance= {:?}", state_cov);
        return state_cov;
    }

    fn compute_mean_points(&self, sigma_points: Vec<Col<f64>>) -> Col<f64> {
        if sigma_points[0].nrows() == 4 {
            return self.compute_mean_quaternion(sigma_points);
        } else {
            return self.compute_mean_sensor_measurements(sigma_points);
        }
    }

    fn get_spread_factor(&self) -> f64 {
        return self.spread_factor;
    }

    fn get_prior_knowledge(&self) -> f64 {
        return self.prior_knowledge;
    }

    fn get_state_process_noise(&self) -> Mat<f64> {
        return self.state_process_noise.clone();
    }

    fn get_measurements_noise(&self) -> Mat<f64> {
        return self.measurements_noise.clone();
    }

    fn compute_cross_innov_covariances(&self, base_sigma_points: &Vec<Col<f64>>, transformed_sigma_points: &Vec<Col<f64>>, predicted_state: &Col<f64>, measured_state: &Col<f64>, state_covariance: &Mat<f64>) -> (Mat<f64>, Mat<f64>) {
        let nrows= state_covariance.nrows() as f64;
        let spread_factor= self.get_spread_factor();
        let prior_knowledge= self.get_prior_knowledge();
        let measurements_noise= self.get_measurements_noise();
        let scale_factor= spread_factor.powf(2.0)*(nrows + kalman_filter::KAPPA) - nrows;
        let scale= nrows + scale_factor;
        //Cross Covariance:
        //  Global uncertainties (in the state space and in the ref sensor measurements state)
        let mut cross_covariance= Mat::<f64>::zeros(state_covariance.nrows(), measured_state.nrows());
        //Innovation Covariance:
        //  The uncertainty of the theoric ref sensor measurements
        let mut innovation_covariance= Mat::<f64>::zeros(measured_state.nrows(), measured_state.nrows());
        let binding= Q64::new(predicted_state[0], predicted_state[1], predicted_state[2], predicted_state[3]);
        if let Some(predicted_quat)= binding.normalize() {
            //Computation of the covariances
            for i in 0..base_sigma_points.len() {
                let mut weight= 1.0/(2.0*scale);
                if i == 0 {
                    weight= (scale_factor / scale) + (1.0 - spread_factor.powf(2.0) + prior_knowledge)
                }
                innovation_covariance += weight * (&transformed_sigma_points[i] - measured_state)*(&transformed_sigma_points[i] - measured_state).transpose();
                let binding= Q64::new(base_sigma_points[i][0], base_sigma_points[i][1], base_sigma_points[i][2], base_sigma_points[i][3]);
                if let Some(sigma_quat)= binding.normalize() {
                    let binding= (sigma_quat*predicted_quat.conj());
                    let diff_quat= binding.as_quaternion();
                    let sgn = if diff_quat.w < 0.0 { -1.0 } else { 1.0 };
                    let error_vec = col![sgn * diff_quat.x, sgn * diff_quat.y, sgn * diff_quat.z];
                    cross_covariance += weight * error_vec *(&transformed_sigma_points[i] - measured_state).transpose()
                }
            }
            innovation_covariance += &measurements_noise;
            println!("Cross Covariance= {:?}", cross_covariance);
            println!("Innovation covariance= {:?}", innovation_covariance);
            return (innovation_covariance, cross_covariance);
        } else {
            return (mat![[0.0]], mat![[0.0]]);
        }
        
    }

    fn compute_final_state(&self, predicted_state: &Col<f64>, kalman_gain: &Mat<f64>, innovation: &Col<f64>) -> Col<f64> {
        let delta_vec= kalman_gain * innovation;
        let delta_quat= Q64::new(1.0, delta_vec[0] / 2.0, delta_vec[1] / 2.0, delta_vec[2] / 2.0);
        if let Some(dq_norm) = delta_quat.normalize() {
            let predicted_quat = Q64::new(predicted_state[0], predicted_state[1], predicted_state[2], predicted_state[3]);
            
            // Updated state = dq * q_pred (Post-multiply or pre-multiply depends on your frame convention)
            let updated_quat = dq_norm * predicted_quat;
            
            // Ensure it's normalized to prevent drift over time
            if let Some(final_quat) = updated_quat.normalize() {
                let final_quat= final_quat.as_quaternion();
                return col![final_quat.w, final_quat.x, final_quat.y, final_quat.z];
                // Now 'true_state' is 4x1 and valid!
            } else {
                return col![0.0];
            }
        } else {
            return col![0.0];
        }
    }
    
    fn update_state_covariance(&mut self) -> Mat::<f64> {
        todo!()
    }
}