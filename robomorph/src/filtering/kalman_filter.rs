use std::sync::{Arc, Mutex};

use faer::{Col, Mat, prelude::Solve};

use crate::{core::event_management::{Event, Observer}, lidar_management::measurements};

pub const KAPPA: f64= 0.0;

pub trait NLKalmanFilter {
    fn predict(&mut self, current_state: Col<f64>, input_measurements: Col<f64>, delta_time: f64, state_covariance: Mat<f64>) -> (Col<f64>, Mat<f64>);

    fn update_prediction(&mut self, predicted_state: Col<f64>, observed_measurements: Col<f64>) -> (Col<f64>, Mat<f64>);

    
}

/*
  Unscented Kalman Filter (UKF)

  This filter is a nonlinear extension of the Kalman filter that avoids
  traditional linearization (no Jacobians or analytic derivatives needed). 
  Instead of approximating the system model itself, the UKF represents the 
  uncertainty of the state as a small set of carefully chosen sample points 
  and pushes those points through the actual nonlinear dynamics and measurement 
  functions. The transformed samples are then recombined to form an updated 
  estimate of the state and its uncertainty.

  In other words, UKF focuses on capturing how the distribution of possible
  states evolves through nonlinear behavior, which often results in more
  accurate results than linearized approaches — especially when the system 
  is highly nonlinear — while keeping implementation intuitive and without 
  requiring complex derivative calculations.
*/
pub trait UnscentedKalmanFilter {
    fn predict(&self, current_state: &Col<f64>, input_measurements: &Col<f64>, delta_time: f64, state_covariance: &Mat<f64>) -> (Col<f64>, Mat<f64>) {
        println!("===== PREDIC STEP =====");
        let nrows= state_covariance.nrows() as f64;
        let spread_factor= self.get_spread_factor();
        let prior_knowledge= self.get_prior_knowledge();
        let state_process_noise= self.get_state_process_noise();
        //The scale factor determine how far from the mean point the sigma point is placed
        //  IF the spread factor is small, the sigma points will be close to the mean point(center point)
        let scale_factor= spread_factor.powf(2.0)*(nrows + KAPPA) - nrows;
        let scale= nrows + scale_factor;
        //Get the sigma points by potentially a custom way, depending on the case 
        ///!\ The Self means calling the generate_sigma_points and compute_mean_points use by the same instance
        let mut sigma_points= Self::generate_sigma_points(&current_state, scale, &state_covariance);
        println!("Based sigma points= {:?}", sigma_points);
        //Apply the transition function to all the sigma points of the distribution to be able to retrieve the predicted state (by weighted mean computation)
        for i in 0..sigma_points.len() {
            sigma_points[i]= Self::apply_transition_function(&sigma_points[i], &input_measurements, delta_time)
        }
        println!("Transformed sigma points= {:?}", sigma_points);
        let mut predicted_state= self.compute_mean_points(sigma_points.clone(), 0.55);
        println!("Mean point= {:?}", predicted_state);
        let state_cov= self.update_state_covariance_from_predict(&predicted_state, &sigma_points, state_covariance, 0.55);
        
        return (predicted_state, state_cov);
    }

    fn update_state_covariance_from_predict(&self, mean_point: &Col<f64>, sigma_points: &Vec<Col<f64>>, state_covariance: &Mat<f64>, w0: f64) -> Mat::<f64> {
        let nrows= state_covariance.nrows() as f64;
        let spread_factor= self.get_spread_factor();
        let prior_knowledge= self.get_prior_knowledge();
        let state_process_noise= self.get_state_process_noise();
        //The scale factor determine how far from the mean point the sigma point is placed
        //  IF the spread factor is small, the sigma points will be close to the mean point(center point)
        let scale_factor= spread_factor.powf(2.0)*(nrows + KAPPA) - nrows;
        let scale= nrows + scale_factor;
        //Compute the state covariance (uncertainty in the state variables)
        let mut state_cov= Mat::<f64>::zeros(state_covariance.nrows(), state_covariance.ncols());
        for i in 1..sigma_points.len() {
            let weight= (1.0-w0)/(sigma_points.len() as f64 -1.0);
            //Compute the "distance" between the sigma points and the center of the distribution
            let diff= (&sigma_points[i] - mean_point);
            println!("Dist Sigma point Mean point= {:?}", diff);
            state_cov += weight * &diff.as_mat()*&diff.as_mat().transpose();
            println!("State covariance (state_cov += weight * &diff.as_mat()*&diff.as_mat().transpose())= {:?}", state_cov);
        }
        state_cov += state_process_noise;
        println!("Final State Covariance= {:?}", state_cov);
        return state_cov;
    }

    fn update_prediction(&self, predicted_state: &Col<f64>, observed_measurements: &Col<f64>, state_covariance: &Mat<f64>) -> (Col<f64>, Mat<f64>) {
        println!("===== UPDATE STEP =====");
        //Creation of another set of sigma points, based on the predicted state
        //  This set of sigma points will be compared with the ref sensor measurements
        let nrows= state_covariance.nrows() as f64;
        let spread_factor= self.get_spread_factor();
        let prior_knowledge= self.get_prior_knowledge();
        let measurements_noise= self.get_measurements_noise();
        let scale_factor= spread_factor.powf(2.0)*(nrows + KAPPA) - nrows;
        let scale= nrows + scale_factor;
        let mut base_sigma_points= Self::generate_sigma_points(&predicted_state, scale, &state_covariance);
        println!("Based sigma points= {:?}", base_sigma_points);
        let mut transformed_sigma_points= Vec::new();
        for i in 0..base_sigma_points.len() {
            //Conversion of the sigma points state space to the ref measurements space to be able to compare them
            //  The transformed sigma point represent the theoric ref sensor measurements at the given sigma point state
            let transformed_sigma_point= Self::apply_ref_measurements_function(base_sigma_points[i].clone(), observed_measurements.nrows());
            transformed_sigma_points.push(transformed_sigma_point);
        }
        println!("Transformed sigma points= {:?}", transformed_sigma_points);
        //Compute the mean points of the theoric sensor measurements at time T
        let measured_state= self.compute_mean_points(transformed_sigma_points.clone(), 0.55);
        println!("Mean point= {:?}", measured_state);
        let (innovation_covariance, cross_covariance)= self.compute_cross_innov_covariances(&base_sigma_points, &transformed_sigma_points, &predicted_state, &measured_state, &state_covariance, 0.55);
        //Computation of the Kalman Gain:
        //  Model the confidence tradeoff between the predict state and the ref sensor measurements 
        let binding = innovation_covariance.clone().qr().solve(cross_covariance.clone().transpose());
        let kalman_gain= binding.transpose().to_owned();
        println!("Kalman Gain= {:?}", kalman_gain);
        //The error between the ref sensor measurements and the theoric ref sensor measurement at the center of the sigma points distribution
        let innovation= observed_measurements  - &measured_state;
        println!("Innovation= {:?}", innovation);
        //Computation of the "true" state (estimated state)
        let final_state= self.compute_final_state(&predicted_state, &kalman_gain, &innovation);
        println!("Updated State= {:?}", final_state);
        //Update of the state covariance
        let updated_cov = state_covariance - &kalman_gain * &innovation_covariance * kalman_gain.transpose();
        println!("Updated State Covariance= {:?}", updated_cov);
        //Force symetry for the state covariance (not affect the filter performance, but avoid matrix computation error)
        let state_cov= 0.5*(&updated_cov + &updated_cov.transpose());
        println!("Final State Covariance= {:?}", state_cov);
        return (final_state, state_cov);
    }

    fn compute_cross_innov_covariances(&self, base_sigma_points: &Vec<Col<f64>>, transformed_sigma_points: &Vec<Col<f64>>, predicted_state: &Col<f64>, measured_state: &Col<f64>, state_covariance: &Mat<f64>, w0: f64) -> (Mat<f64>, Mat<f64>) {
        let nrows= state_covariance.nrows() as f64;
        let spread_factor= self.get_spread_factor();
        let prior_knowledge= self.get_prior_knowledge();
        let measurements_noise= self.get_measurements_noise();
        let scale_factor= spread_factor.powf(2.0)*(nrows + KAPPA) - nrows;
        let scale= nrows + scale_factor;
        //Cross Covariance:
        //  Global uncertainties (in the state space and in the ref sensor measurements state)
        let mut cross_covariance= Mat::<f64>::zeros(state_covariance.nrows(), measured_state.nrows());
        //Innovation Covariance:
        //  The uncertainty of the theoric ref sensor measurements
        let mut innovation_covariance= Mat::<f64>::zeros(measured_state.nrows(), measured_state.nrows());
        //Computation of the covariances
        for i in 1..base_sigma_points.len() {
            let weight= (1.0-w0)/(base_sigma_points.len() as f64 -1.0);
            innovation_covariance += weight * (&transformed_sigma_points[i] - measured_state)*(&transformed_sigma_points[i] - measured_state).transpose();
            cross_covariance += weight * (&base_sigma_points[i] - predicted_state)*(&transformed_sigma_points[i] - measured_state).transpose()
        }
        innovation_covariance += &measurements_noise;
        println!("Cross Covariance= {:?}", cross_covariance);
        println!("Innovation covariance= {:?}", innovation_covariance);
        return (innovation_covariance, cross_covariance);
    }

    fn compute_final_state(&self, predicted_state: &Col<f64>, kalman_gain: &Mat<f64>, innovation: &Col<f64>) -> Col<f64> {
        return predicted_state + kalman_gain * innovation;
    }

    fn update_state_covariance(&mut self) -> Mat::<f64>;

    fn generate_sigma_points(current_state: &Col<f64>, scale: f64, state_covariance: &Mat<f64>) -> Vec<Col<f64>>{
        //The center of the distribution
        let mean_point= current_state.clone();
        //The sigma points representing the distribution of the predicted state
        let mut sigma_points= vec![mean_point.clone()];
        //Performing the cholesky decomposition to retrieve the sigma points from it
        if let Ok(cholesky_decomp) = (scale*state_covariance).llt(faer::Side::Lower) {
            let datas= cholesky_decomp.L();
            //Adding sigma points from each columns of the decomposition
            for i in 0..mean_point.nrows(){
                sigma_points.push(&mean_point + datas.col(i));
                sigma_points.push(&mean_point - datas.col(i));
            }
        }
        return sigma_points;
    }

    fn compute_mean_points(&self, sigma_points: Vec<Col<f64>>, w0: f64) -> Col<f64> {
        let nrows= sigma_points[0].nrows() as f64;
        let mut mean_point= w0 * &sigma_points[0];
        //Compute the mean point / state
        for i in 1..sigma_points.len() {
            //Computation of the mean point / state with a weighted mean computation
            mean_point+= (1.0-w0)/(sigma_points.len() as f64-1.0) * &sigma_points[i];
        }
        return mean_point;
    }

    //The cinematic function of the system
    //  The function used to model how the system evolution, over time, given the input data applied to this system
    //  (state, sensor input measurements, delta time) -> next theoric state
    fn apply_transition_function(state: &Col<f64>, input_measurements: &Col<f64>, delta_time: f64) -> Col<f64>;

    //The sensor measurement estimation
    //  The goal of this function is to model the theoric ref sensor measurement for a given state
    //  (state, size of measurements vector) -> expected ref sensor measurements
    fn apply_ref_measurements_function(state: Col<f64>, measurements_vec_size: usize) -> Col<f64>;

    fn get_spread_factor(&self) -> f64;

    fn get_prior_knowledge(&self) -> f64;

    fn get_state_process_noise(&self) -> Mat<f64>;

    fn get_measurements_noise(&self) -> Mat<f64>;
}

#[derive(Clone)]
pub struct KalmanMeasurements {
    pub input_sensor_measurements: Col<f64>,
    pub ref_sensor_measurements: Col<f64>,
    pub delta_time: f64
}



