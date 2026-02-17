pub trait ActuatorControl {
    fn apply_command_law(&mut self, state: Vec<f64>, setpoint: Vec<f64>);
}