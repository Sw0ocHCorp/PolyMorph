use std::f64;

use faer::{Col, Mat, col, mat, matrix_free::LinOp};
use num_quaternion::Q64;
use robomorph::{core::file_logger::FileLogger, filtering::kalman_filter::{self, KalmanMeasurements, UnscentedKalmanFilter}};

/*
  Specific Unscented Kalman Filter implementation
    For Orientation estimation (in Quaternion Space)
  =============================
  This UKF estimate the 3D orientation of the system using only 9 DOF IMU measurements.
  This is the core form of the estimator. It will be upgraded in the future to be able to estimate the orientation of the system when it moving
    When the accelerometer measure gravity AND linear acceleration

  Senors used:
    - 3-axis Gyroscope: Measures angular velocity (ωx, ωy, ωz) in body frame    => Used as control input    in predict step
    - 3-axis Accelerometer: Measures specific force (ax, ay, az) in body frame  => Used as ref sensor       in the update step
    - 3-axis Magnetometer: Measures magnetic field (mx, my, mz) in body frame   => Used as ref sensor       in the update step

  State Vector:
    x= [qw, qx, qy, qz] => Hamilton convention
    qw=             cos(θ/2)        -> scalar part
    [qx, qy, qz]=   sin(θ/2) * axis -> vector part
    Where:
        θ represent an axis rotation
    x is a unit quaternion because it represent a 3D orientation
  Measurements State:
    y= [ax, ay, az, mx, my, mz]

  Frames:
    World Frame:
        NED (North-East-Down):
            Gravity vector= [0.0, 0.0, 1] in G unit
            X= North, Y= East, Z= Down
    Body Frame:
        raw IMU measurements are express in the Body frame
    Frames transitions:
        Body Frame  -> World Frame= q               ⊗ v_body    ⊗ q.conjugate()
        World Frame -> Body Frame=  q.conjugate()   ⊗ v_world   ⊗ q
  Params:
    State covariance:
        3x3 matrix because even if Quaternion is 4D, it used to describe orientation in 3D, so there is 3 degrees of freedom
    Process noise covariance (Q matrix):
        Model the uncertainty of the model (gyrometer noise & unmodeled dynamics)
    Measurements Noise (R matrix):
        Noise of the ref sensors (Accelerometer and Magnetometer)
    Kalman Gain:
        3x6 matrix because the same dimension of the measurements state and the number of dimensions of the space described by the quaternion
*/

const REF_WORLD_MAGNETIC_FIELD: [f64; 3] = [15.3017, 0.4328527, -41.06483];
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
    logger: FileLogger,
    
}

impl OrientationUKF {
    pub fn new (init_state: Col<f64>, state_covariance: Mat<f64>, measurements_noise: Mat<f64>, state_process_noise: Mat<f64>, 
                                                    spread_factor:f64, prior_knowledge: f64) -> Self {
        let logger= FileLogger::new("Logs".to_string());
        return Self { current_state: init_state, state_covariance: state_covariance, 
                        measurements_noise, state_process_noise, spread_factor, prior_knowledge, logger: logger};
    }

    //Specific mean computation from the state distribution in the quaternion space
    fn compute_mean_quaternion(&self, sigma_points: Vec<Col<f64>>, w0: f64) -> Col<f64> {
        let mut mean_point= w0 * &sigma_points[0];
        //The mean point is computed 
        let mut mean_point_mat= w0 * &sigma_points[0] * &sigma_points[0].transpose();
        //Compute the mean point / state matrix. This matrix is used to get the mean quaternion
        for i in 1..sigma_points.len() {
            //Compute the weights for the given point (weights for the center point is differents than the other points of the distribution)
            mean_point_mat += (1.0-w0)/(sigma_points.len() as f64 - 1.0) * &sigma_points[i] * &sigma_points[i].transpose()
        }
        //The mean quaternion will be the eigen vector with the largest eigen value
        //  Eigen vector is a vector that not changed direction but change scale when linear transformation or matrix is apply to it
        //      This scale value is called the eigen value
        //      Exemple:
        //          A*v = Y*v
        //      Were:
        //          A is a matrix
        //          v is an eigen vector
        //          Y is an eigen value
        if let Ok(eigen_decomp)  = mean_point_mat.self_adjoint_eigen(faer::Side::Lower) {
            let eigen_vectors= eigen_decomp.U();
            let eigen_values= eigen_decomp.S();
            let mut max_val= f64::MIN;
            for i in 0..eigen_values.nrows() {
                if eigen_values[i] > max_val {
                    max_val= eigen_values[i];
                    mean_point= eigen_vectors.col(i).to_owned()
                }
            }
        }
        let euclid_norm= mean_point.norm_l2();
        //Ensure the resulting quaternion is Unit quaternion
        if euclid_norm > 1e-12 {
            return mean_point / euclid_norm;
        } else {
            // Fallback if something went wrong (return identity)
            return col![1.0, 0.0, 0.0, 0.0];
        }
    }

    //Specific mean computation from the state distribution in the measurements space
    fn compute_mean_sensor_measurements(&self, sigma_points: Vec<Col<f64>>, w0: f64) -> Col<f64> {
        let mut mean_point= w0*&sigma_points[0];
        //Compute the mean point / state
        //  The mean point is computed like a classic weighted mean because the measurements space is a classic space
        for i in 1..sigma_points.len() {
            mean_point+= (1.0 - w0)/(sigma_points.len() as f64 - 1.0) * &sigma_points[i];
        }
        return mean_point;
    }

    //Pipeline to compute the estimated state from all the sensors measurements
    pub fn estimate_true_state(&mut self, meas: KalmanMeasurements) -> Col<f64> {
        let (predicted_state, state_covariance)= self.predict(&self.current_state, &meas.input_sensor_measurements, meas.delta_time, &self.state_covariance);
        self.current_state= predicted_state.clone();
        let (true_state, update_state_cov)= self.update_prediction(&predicted_state, &meas.ref_sensor_measurements, &state_covariance);
        self.state_covariance= update_state_cov;
        self.current_state= true_state.clone();
        return predicted_state;
    }
}

impl UnscentedKalmanFilter for OrientationUKF {

    //Dynamics model of the system
    //  Describe the rotation of a quaternion based on gyrometer measurements
    fn apply_transition_function(state: &Col<f64>, input_measurements: &Col<f64>, delta_time: f64) -> Col<f64> {
        // 1. Extract components (State is [w, x, y, z])
        let w = state[0];
        let x = state[1];
        let y = state[2];
        let z = state[3];

        // Gyro input (assumed Local Frame)
        let gx = input_measurements[0];
        let gy = input_measurements[1];
        let gz = input_measurements[2];

        // 2. Compute the derivatives (dq/dt)
        let dw = 0.5 * (-x * gx - y * gy - z * gz);
        let dx = 0.5 * ( w * gx + y * gz - z * gy);
        let dy = 0.5 * ( w * gy - x * gz + z * gx);
        let dz = 0.5 * ( w * gz + x * gy - y * gx);

        // 3. Integrate (Euler Integration)
        let mut w_new = w + dw * delta_time;
        let mut x_new = x + dx * delta_time;
        let mut y_new = y + dy * delta_time;
        let mut z_new = z + dz * delta_time;

        // 4. Normalize to prevent numerical drift
        let magnitude = (w_new.powi(2) + x_new.powi(2) + y_new.powi(2) + z_new.powi(2)).sqrt();
        
        if magnitude > 1e-9 {
            w_new /= magnitude;
            x_new /= magnitude;
            y_new /= magnitude;
            z_new /= magnitude;
        }

        col![w_new, x_new, y_new, z_new]

    }

    //Conversion of a quaternion in the ref sensor measurements space
    //  Here, the ref sensors are the magnetometer & accelerometer
    fn apply_ref_measurements_function(state: Col<f64>, measurements_vec_size: usize) -> Col<f64> {
        match Q64::new(*state.get(0), *state.get(1), *state.get(2), *state.get(3)).normalize() {
            Some(quat_state) => {
                let expected_accel_vec= quat_state.conj().rotate_vector(REF_REST_ACCEL);
                let norm= f64::sqrt(REF_WORLD_MAGNETIC_FIELD[0].powf(2.0) + REF_WORLD_MAGNETIC_FIELD[1].powf(2.0) + REF_WORLD_MAGNETIC_FIELD[2].powf(2.0));
                let expected_magnetic_field= quat_state.conj().rotate_vector([REF_WORLD_MAGNETIC_FIELD[0] / norm, REF_WORLD_MAGNETIC_FIELD[1] / norm, REF_WORLD_MAGNETIC_FIELD[2] / norm]);           
                return col![expected_accel_vec[0], expected_accel_vec[1], expected_accel_vec[2],
                            expected_magnetic_field[0], expected_magnetic_field[1], expected_magnetic_field[2]];
            },
            None => return Col::<f64>::zeros(measurements_vec_size),
        } 
    }

    //Generation of sigma points to model the theorice region were the state could be in
    fn generate_sigma_points(current_state: &Col<f64>, scale: f64, state_covariance: &Mat<f64>) -> Vec<Col<f64>>{
        //current_state= The center of the distribution
        //  The sigma points is representing the distribution of the expected system state
        let mut sigma_points= vec![current_state.clone()];
        //Performing the cholesky decomposition to retrieve the sigma points from it
        ///!\ The state covariance is 3x3 because it model the uncertainty in a 3D frame
        //      Quaternion is a 4D vector that model an orientation in 3D space (Quaternion have 3 DOFs) 
        if let Ok(cholesky_decomp) = (scale*state_covariance).llt(faer::Side::Lower) && 
                let Some(current_quat)= Q64::new(*current_state.get(0), *current_state.get(1), *current_state.get(2), *current_state.get(3)).normalize() {
            let datas= cholesky_decomp.L();
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
                    //Add the normalized quaternion sigma point to the list of sigma point
                    if let Some(sigma_point_dist)= delta_quat.normalize() {
                        let binding= (&sigma_point_dist * &current_quat);
                        let sigma_point= binding.as_quaternion();
                        //println!("Sigma point: Delta quat normalized * Current State(mean state)= {:?}", sigma_point);
                        sigma_points.push(col![sigma_point.w, sigma_point.x, sigma_point.y, sigma_point.z]);
                    }
                }
            }
        }
        return sigma_points;
    }

    //Update of the state covariance, basd on data from the predict step
    fn update_state_covariance_from_predict(&self, mean_point: &Col<f64>, sigma_points: &Vec<Col<f64>>, state_covariance: &Mat<f64>, w0: f64) -> Mat::<f64> {
        let state_process_noise= self.get_state_process_noise();
        //Compute the state covariance (uncertainty in the state variables)
        let mut state_cov= Mat::<f64>::zeros(state_covariance.nrows(), state_covariance.ncols());
        let mean_quat= Q64::new(mean_point[0], mean_point[1], mean_point[2], mean_point[3]);
        for i in 0..sigma_points.len() {
            let sigma_quat= Q64::new(sigma_points[i][0], sigma_points[i][1], sigma_points[i][2], sigma_points[i][3]);
            let weight= if i ==0 {
                w0
            } else {
                (1.0-w0)/(sigma_points.len() as f64 - 1.0)
            };
            //Compute the "distance" between the sigma points and the center of the distribution
            if let Some(delta_q)= (sigma_quat*mean_quat.conj()).normalize() {
                let diff_quat= delta_q.as_quaternion();
                let vec_part= col![diff_quat.x, diff_quat.y, diff_quat.z];
                let norm= vec_part.norm_l2();
                let error_vec= if norm < 1e-8 {
                    col![0.0, 0.0, 0.0]
                } else {
                    let angle = 2.0 * f64::atan2(norm, diff_quat.w);
                    vec_part * (angle / norm)
                };
                state_cov += weight * &error_vec*&error_vec.transpose();
            }
        }
        state_cov += state_process_noise;
        return state_cov;
    }

    //Compute the mean point of a distribution
    //  The method are not the same for points in the state space
    //  And points in the ref measurements state
    fn compute_mean_points(&self, sigma_points: Vec<Col<f64>>, w0: f64) -> Col<f64> {
        if sigma_points[0].nrows() == 4 {
            return self.compute_mean_quaternion(sigma_points, w0);
        } else {
            return self.compute_mean_sensor_measurements(sigma_points, w0);
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
    
    //Specific implementation of the innovation and cross-covariances
    //  The usage of quaternions require particular computations
    fn compute_cross_innov_covariances(&self, quat_sigma_points: &Vec<Col<f64>>, theoric_sigma_points: &Vec<Col<f64>>, predicted_state: &Col<f64>, measured_state: &Col<f64>, state_covariance: &Mat<f64>, w0: f64) -> (Mat<f64>, Mat<f64>) {
        let measurements_noise= self.get_measurements_noise();
        //Cross Covariance:
        //  Global uncertainties (in the state space and in the ref sensor measurements state)
        let mut cross_covariance= Mat::<f64>::zeros(state_covariance.nrows(), measured_state.nrows());
        //Innovation Covariance:
        //  The uncertainty of the theoric ref sensor measurements
        let mut innovation_covariance= Mat::<f64>::zeros(measured_state.nrows(), measured_state.nrows());
        let binding= Q64::new(predicted_state[0], predicted_state[1], predicted_state[2], predicted_state[3]);
        if let Some(mean_quat)= binding.normalize() {
            //Computation of the covariances
            for i in 0..quat_sigma_points.len() {
                let weight = if i == 0 { 
                    w0 
                } else { 
                    (1.0 - w0) / (quat_sigma_points.len() as f64 - 1.0) 
                };
                innovation_covariance += weight * (&theoric_sigma_points[i] - measured_state)*(&theoric_sigma_points[i] - measured_state).transpose();
                let sigma_quat= Q64::new(quat_sigma_points[i][0], quat_sigma_points[i][1], quat_sigma_points[i][2], quat_sigma_points[i][3]);
                if let Some(delta_q)= (sigma_quat*mean_quat.conj()).normalize() {
                    let diff_quat= delta_q.as_quaternion();
                    let vec_part= col![diff_quat.x, diff_quat.y, diff_quat.z];
                    let norm= vec_part.norm_l2();
                    let error_vec= if norm < 1e-8 {
                        col![0.0, 0.0, 0.0]
                    } else {
                        let angle = 2.0 * f64::atan2(norm, diff_quat.w);
                        vec_part * (angle / norm)
                    };
                    cross_covariance += weight * error_vec *(&theoric_sigma_points[i] - measured_state).transpose()
                }
            }
            innovation_covariance += &measurements_noise;
            return (innovation_covariance, cross_covariance);
        } else {
            return (mat![[0.0]], mat![[0.0]]);
        }
    }

    //Computation of the estimated state (final state of the UKF loop)
    fn compute_final_state(&self, predicted_state: &Col<f64>, kalman_gain: &Mat<f64>, innovation: &Col<f64>) -> Col<f64> {
        let delta_vec= kalman_gain * innovation;
        let angle = delta_vec.norm_l2();
        let delta_quat = if angle < 1e-8 {
            Q64::new(1.0, delta_vec[0] / 2.0, delta_vec[1] / 2.0, delta_vec[2] / 2.0)
        } else {
            let half_angle = angle / 2.0;
            let s = f64::sin(half_angle) / angle;
            Q64::new(f64::cos(half_angle), delta_vec[0] * s, delta_vec[1] * s, delta_vec[2] * s)
        };
        if let Some(dq_norm) = delta_quat.normalize() {
            let predicted_quat = Q64::new(predicted_state[0], predicted_state[1], predicted_state[2], predicted_state[3]);
            
            // Updated state = dq * q_pred (Post-multiply or pre-multiply depends on your frame convention)
            //The update of the quaternion occur in the World Frame
            //  Apply change in the World Frame=    q ⊗ delta_q
            //  Apply change in the Body Frame=     delta_q ⊗ q
            let updated_quat = dq_norm*predicted_quat;
            
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
}