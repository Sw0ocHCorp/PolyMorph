use crate::actuators::actuator_control::ActuatorControl;

pub struct ServoControl {
    setpoint: f64
}

impl ServoControl {

}

impl ActuatorControl for ServoControl {
    fn apply_command_law(&mut self, state: Vec<f64>, setpoint: Vec<f64>) {
        todo!()
    }
}