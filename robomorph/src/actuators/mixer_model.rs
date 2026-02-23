pub trait MixerModel {
    fn apply_command_law(&mut self, current_state: Vec<f64>, setpoint: Vec<f64>, dt: f64) -> Vec<f64>;
}