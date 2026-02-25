
use faer::{Col, Mat, prelude::Solve};


pub const KAPPA: f64= 0.0;

pub trait NLKalmanFilter {
    fn predict(&mut self, current_state: Col<f64>, input_measurements: Col<f64>, delta_time: f64, state_covariance: Mat<f64>) -> (Col<f64>, Mat<f64>);

    fn update_prediction(&mut self, predicted_state: Col<f64>, observed_measurements: Col<f64>) -> (Col<f64>, Mat<f64>);

    
}

/*
  Unscented Kalman Filter (UKF)
  =============================
  This filter is a nonlinear extension of the Kalman filter that avoids
  traditional linearization (no Jacobians or analytic derivatives needed). 
  Instead of approximating the system model itself, the UKF represents the 
  uncertainty of the state as a small set of carefully chosen sample points 
  (sigma points) and pushes those points through the actual nonlinear dynamics 
  and measurement functions. The transformed samples are then recombined to 
  form an updated estimate of the state and its uncertainty.

  In other words, UKF focuses on capturing how the distribution of possible
  states evolves through nonlinear behavior, which often results in more
  accurate results than linearized approaches — especially when the system 
  is highly nonlinear — while keeping implementation intuitive and without 
  requiring complex derivative calculations.

  Key Components:
    - State vector (x): The quantities we want to estimate
    - State covariance (P): Uncertainty in our state estimate
    - Process noise covariance (Q): Uncertainty in the process model
        It means the whole uncertainty during the predict step
            + Model imperfections= the uncertainty that is not model by the transition function
            + Input sensor measurements noise
        * Low Q= less confidence in the model / input sensor. 
        * High Q= more confidence in the model / input sensor 
    - Measurement noise covariance (R): Uncertainty in sensor measurements
    - Sigma points (χ): Deterministically chosen sample points that capture
      the mean and covariance of the state distribution
    - Weights (W): Used to recombine transformed sigma points into statistics

  UKF Pipeline to estimate state
  ==============================
    1. Predict Step (Dead Reckoning Step):
    _____________________________________
        Goal= Predict the new state at time T based on the state at T-1, 
            the control inputs, and the theoretical dynamics model of the system.
            This is the dead reckoning step.

        1.1. Sigma points generations:
                Instead of linearizing the system around the state at T-1, sample 
                states are generated to build the state distribution around x̂(T-1).
                This models the region where the system could be.
                Computation:
                    - χ₀ = x̂(T-1)                                       [mean point]
                    - χᵢ = x̂(T-1) + (√((n+λ)P(T-1)))ᵢ    for i=1..n     [positive spread]
                    - χᵢ = x̂(T-1) - (√((n+λ)P(T-1)))ᵢ₋ₙ  for i=n+1..2n  [negative spread]
                Where:
                    - x̂(T-1)= estimated point at T-1 and distribution center
                    - n = dimension of state vector
                    - λ = α²(n+κ) - n is a scaling parameter
                    - α controls the distance of the sigma point from the center points (typically 0.001 to 1)
                    - κ is a secondary scaling parameter (usually 0 or 3-n)
                    - √((n+λ)P) is computed via Cholesky decomposition: L such that LLᵀ = (n+λ)P
                Result: 2n+1 sigma points that capture mean and covariance of x̂(T-1)

        1.2. Process Model Propagation (Applying transition function the state distribution):
                Because the UKF is a nonlinear Kalman filter, the process/transition 
                function f(·) is applied directly to each sigma point to transform 
                the state distribution from time T-1 to time T.
                For each sigma point: χᵢ⁻(T) = f(χᵢ(T-1), u(T-1))
                Where:
                    - f(·) is the nonlinear process model
                    - u(T-1) is the control input
                    - χᵢ⁻(T) denotes predicted (a priori) sigma points
                Note: f(·) can be integration of differential equations, lookup tables,
                    or any nonlinear transformation. No linearization required!

        1.3. Predicted State Estimation (Computing Distribution Mean):
                Because the UKF is a nonlinear filter, we must compute the weighted
                mean of the transformed sigma points to get the predicted state x̂⁻(T).
                This step is MANDATORY because:
                    f(x̂(T-1)) ≠ mean(f(χᵢ(T-1)))  for nonlinear f
                Formula: x̂⁻(T) = Σᵢ₌₀²ⁿ Wᵢᵐ · χᵢ⁻(T)
                Where weights Wᵢᵐ (mean weights) are:
                    - W₀ᵐ = λ/(n+λ)
                    - Wᵢᵐ = 1/(2(n+λ))  for i=1..2n
                Quick example illustrating why direct propagation fails:
                    Nonlinear function: f(x) = x²,  spread factor λ = 2
                    Base point: x̂ = 0,  Sigma points: χ = {-2, 0, 2}
                    After propagation: χ⁻ = {4, 0, 4}
                    Weighted mean: x̂⁻ = mean([4, 0, 4]) = 2.667
                    Direct propagation: f(x̂) = f(0) = 0
                    Result: 0 ≠ 2.667  ⟹  Must use weighted mean!

        1.4. Predicted Covariance Update:
                Update the state covariance matrix P⁻(T), which models the 
                uncertainty in our state estimate after prediction.
                Formula: P⁻(T) = Σᵢ₌₀²ⁿ Wᵢᶜ · (χᵢ⁻(T) - x̂⁻(T))(χᵢ⁻(T) - x̂⁻(T))ᵀ + Q
                Where:
                    - Wᵢᶜ (covariance weights) are:
                        W₀ᶜ = λ/(n+λ) + (1 - α² + β)
                        Wᵢᶜ = 1/(2(n+λ))  for i=1..2n
                    - β is a parameter to incorporate prior knowledge of distribution
                        (β=2 is optimal for Gaussian distributions)
                    - Q is the process noise covariance matrix (models uncertainty
                        in the process model and unmodeled disturbances)
                Note: The first weight W₀ᶜ can be negative, which is fine for 
                    covariance computation and helps capture kurtosis of the distribution.

    2. UPDATE STEP (Measurement Update)
    _____________________________________
        Goal= Use reference sensors to correct potential drift/error from the 
            prediction step (which relied only on the process model and control inputs).
            This fuses the predicted state with actual sensor measurements.
        2.1. Sigma Points Regeneration:
                Generate a new set of sigma points from the predicted state x̂⁻(T)
                and covariance P⁻(T). This creates a fresh state distribution in
                the state space representing our current uncertainty.
                Computation (same as 1.1, but using predicted state):
                    - χ₀ = x̂⁻(T)
                    - χᵢ = x̂⁻(T) ± (√((n+λ)P⁻(T)))ᵢ
                Result: 2n+1 sigma points centered on the predicted state

        2.2. Measurement Model Transformation (Applying measurements function on the state distribution):
                Apply the measurement function h(·) to transform each sigma point
                from state space into measurement space. This represents the expected
                (theoretical) sensor measurements for each possible state in our
                distribution.
                For each sigma point: γᵢ = h(χᵢ)
                Where:
                    - h(·) is the nonlinear measurement model (maps state to measurements)
                    - γᵢ are the predicted measurements (sigma points in measurement space)
                Then compute the mean predicted measurement:
                    ŷ(T) = Σᵢ₌₀²ⁿ Wᵢᵐ · γᵢ
                This is what we expect the sensors to read, given our predicted state.

        2.3. Computing Innovation Covariance and Cross-Correlation:
                a) Innovation Covariance (Pᵧᵧ):
                Represents uncertainty in the predicted measurements.
                Formula: Pᵧᵧ = Σᵢ₌₀²ⁿ Wᵢᶜ · (γᵢ - ŷ)(γᵢ - ŷ)ᵀ + R
                Where:
                    - R is the measurement noise covariance matrix (sensor uncertainty)
                    - This captures how measurement uncertainty propagates through h(·)
             
                b) Cross-Correlation (Pₓᵧ):
                    Represents the correlation between state errors and measurement errors.
                    This is crucial for determining how to map measurement corrections
                    back into state space corrections.
                    Formula: Pₓᵧ = Σᵢ₌₀²ⁿ Wᵢᶜ · (χᵢ - x̂⁻)(γᵢ - ŷ)ᵀ
                    This tells us: "When the measurement is off by Δy, the state is 
                    typically off by (something related to) Pₓᵧ · Δy"

        2.4. Computing the Kalman Gain:
                The Kalman gain K determines how much we trust the measurements
                versus the prediction. It's the optimal weighting that minimizes
                the posterior error covariance.
                Formula: 
                    K = Pₓᵧ · Pᵧᵧ⁻¹
                Interpretation:
                    - K ≈ 0: Trust the prediction (measurements are very noisy)
                    - K ≈ I: Trust the measurements (prediction is very uncertain)
                    - Generally, K is somewhere in between based on relative uncertainties

        2.5. Computing the Innovation:
                The innovation (or measurement residual) is the difference between
                the actual sensor measurement and our predicted measurement.
                Formula: 
                    ν = y_measured(T) - ŷ(T)
                Where:
                    - y_measured(T) is the actual sensor reading at time T
                    - ŷ(T) is what we expected to measure (from step 2.2)
                This represents "new information" from the sensors that wasn't
                captured by our prediction.

        2.6. State Update (A Posteriori Estimate):
                Correct the predicted state using the innovation, weighted by 
                the Kalman gain.
                Formula: 
                    x̂(T) = x̂⁻(T) + K · ν
                This is the final, corrected state estimate incorporating both
                the prediction and the measurement.

        2.7. Covariance Update (A Posteriori Covariance):
                Update the state covariance to reflect the reduced uncertainty
                after incorporating the measurement.
                Formula: 
                    P(T) = P⁻(T) - K · Pᵧᵧ · Kᵀ
                This reduces uncertainty along directions where measurements
                provide information. The covariance should shrink (become more confident) 
                after a good measurement update.
                Note: Some implementations use the Joseph form for better numerical
                stability:
                    P(T) = (I - KH)P⁻(T)(I - KH)ᵀ + KRKᵀ
                where H is the linearized measurement matrix (if available).

        After Step 2.7, we have our final state estimate x̂(T) and covariance P(T) for time T. 
        This becomes x̂(T-1) and P(T-1) for the next iteration.

  Important Notes:
    - The UKF assumes Gaussian distributions. For highly non-Gaussian cases,
        particle filters may be more appropriate.
    - Numerical stability: Watch for P becoming non-positive-definite due to
        rounding errors. May need regularization or Joseph form updates.
    - Tuning Q and R matrices is critical for good performance. Q models 
        process uncertainty, R models sensor noise.
    - For constrained states (e.g., quaternions), special handling is needed
        in sigma point generation and mean computation (see quaternion-specific UKF).
*/
pub trait UnscentedKalmanFilter {
    //Determine the theoric state of the system at time T
    //  Based on the previous estimated state: x̂(T-1)
    //  The input control measuerements: u(T-1)
    //  The known system dynamics: f(χᵢ(T-1), u(T-1))
    fn predict(&self, current_state: &Col<f64>, input_measurements: &Col<f64>, delta_time: f64, state_covariance: &Mat<f64>) -> (Col<f64>, Mat<f64>) {
        let nrows= state_covariance.nrows() as f64;
        let spread_factor= self.get_spread_factor();
        //The scale factor determine how far from the mean point the sigma point is placed
        //  IF the spread factor is small, the sigma points will be close to the mean point(center point)
        let scale_factor= spread_factor.powf(2.0)*(nrows + KAPPA) - nrows;
        let scale= nrows + scale_factor;
        //Get the sigma points by potentially a custom way, depending on the case 
        ///!\ The Self means calling the generate_sigma_points and compute_mean_points use by the same instance
        let mut sigma_points= Self::generate_sigma_points(&current_state, scale, &state_covariance);
        //Apply the transition function to all the sigma points of the distribution
        //  The results sigma points are the state transformed by the system dynamics
        for i in 0..sigma_points.len() {
            sigma_points[i]= Self::apply_transition_function(&sigma_points[i], &input_measurements, delta_time)
        }
        //Get the mean points of the transformed state distribution
        let predicted_state= self.compute_mean_points(sigma_points.clone(), 0.55);
        //Update the state covariance, based on the predicted state determine in the predict step
        let state_cov= self.update_state_covariance_from_predict(&predicted_state, &sigma_points, state_covariance, 0.55);
        
        return (predicted_state, state_cov);
    }

    //Update operation of the state covariance based on the predict step transformed distribution  
    //  W0 is provided by the user to be able to use different weight computation method according to the uses cases
    fn update_state_covariance_from_predict(&self, mean_point: &Col<f64>, sigma_points: &Vec<Col<f64>>, state_covariance: &Mat<f64>, w0: f64) -> Mat::<f64> {
        let state_process_noise= self.get_state_process_noise();
        //Compute the state covariance (uncertainty in the state variables)
        let mut state_cov= Mat::<f64>::zeros(state_covariance.nrows(), state_covariance.ncols());
        for i in 1..sigma_points.len() {
            let weight= (1.0-w0)/(sigma_points.len() as f64 -1.0);
            //Compute the "distance" between the sigma points and the center of the distribution
            let diff= &sigma_points[i] - mean_point;
            state_cov += weight * &diff.as_mat()*&diff.as_mat().transpose();
        }
        state_cov += state_process_noise;
        return state_cov;
    }

    fn update_prediction(&self, predicted_state: &Col<f64>, observed_measurements: &Col<f64>, state_covariance: &Mat<f64>) -> (Col<f64>, Mat<f64>) {
        //Creation of another set of sigma points, based on the predicted state
        //  This set of sigma points will be converted in the ref measurements space
        let nrows= state_covariance.nrows() as f64;
        let spread_factor= self.get_spread_factor();
        let scale_factor= spread_factor.powf(2.0)*(nrows + KAPPA) - nrows;
        let scale= nrows + scale_factor;
        let base_sigma_points= Self::generate_sigma_points(&predicted_state, scale, &state_covariance);
        let mut transformed_sigma_points= Vec::new();
        for i in 0..base_sigma_points.len() {
            //The transformed sigma point represent the theoric ref sensor measurements at the given sigma point state
            let transformed_sigma_point= Self::apply_ref_measurements_function(base_sigma_points[i].clone(), observed_measurements.nrows());
            transformed_sigma_points.push(transformed_sigma_point);
        }
        //Compute the mean points of the theoric sensor measurements at time T
        let measured_state= self.compute_mean_points(transformed_sigma_points.clone(), 0.55);
        //Compute the covariances
        //  Innovation Covariance= model the uncertainty in the ref measurements space
        //  Cross-Covariance= model the correlation between the errors in the state space and the error in the ref measurements space
        let (innovation_covariance, cross_covariance)= self.compute_cross_innov_covariances(&base_sigma_points, &transformed_sigma_points, &predicted_state, &measured_state, &state_covariance, 0.55);
        //Computation of the Kalman Gain:
        //  Model the confidence tradeoff between the predict state(result of the model dynamics) and the ref sensor measurements 
        let binding = innovation_covariance.clone().qr().solve(cross_covariance.clone().transpose());
        let kalman_gain= binding.transpose().to_owned();
        //The error between the ref sensor measurements and the theoric ref sensor measurement at the center of the sigma points distribution
        let innovation= observed_measurements  - &measured_state;
        //Computation of the "true" state (estimated state)
        let final_state= self.compute_final_state(&predicted_state, &kalman_gain, &innovation);
        //Update of the state covariance
        let updated_cov = state_covariance - &kalman_gain * &innovation_covariance * kalman_gain.transpose();
        //Force symetry for the state covariance (not affect the filter performance, but avoid matrix computation error)
        let state_cov= 0.5*(&updated_cov + &updated_cov.transpose());
        return (final_state, state_cov);
    }

    fn compute_cross_innov_covariances(&self, base_sigma_points: &Vec<Col<f64>>, transformed_sigma_points: &Vec<Col<f64>>, predicted_state: &Col<f64>, measured_state: &Col<f64>, state_covariance: &Mat<f64>, w0: f64) -> (Mat<f64>, Mat<f64>) {
        let measurements_noise= self.get_measurements_noise();
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
        return (innovation_covariance, cross_covariance);
    }

    //
    fn compute_final_state(&self, predicted_state: &Col<f64>, kalman_gain: &Mat<f64>, innovation: &Col<f64>) -> Col<f64> {
        return predicted_state + kalman_gain * innovation;
    }

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



