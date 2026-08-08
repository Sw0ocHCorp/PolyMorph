use std::time::Duration;

use robomorph::{communications::interface::HardwareInterface, messages::registered_message::AnyMessage};

/// Trait that give the fundamental methods of all the vehicle controllers connected to Gazebo simulation environment (Simulation)
pub trait VehicleController {

    /// Connect the simulation node to all the required topics to be able to control the vehicle
    /// 
    /// Notes:
    /// 
    /// Controller must be to be mutable because the topics will update the state of the vehicle
    fn start_listening_topics(&mut self);

    /// Apply the setpoints values for all the actuators
    /// 
    /// Arguments:
    /// 
    /// setpoints: list of the setpoints to apply
    ///
    /// dt: time elapsed since the last call, forwarded to the motion controller to integrate rate-based corrections
    fn apply_actuator_setpoints(&mut self, setpoints: Vec<AnyMessage>, dt: Duration);

    /// Send telemetry data to the telemetry receiver linked to telemetry_sender
    /// 
    /// Most likely to an interface
    fn send_telemetry(&mut self);

    /// Connect the vehicle controller to the interface to be able to send / receive data from the given interface
    /// 
    /// Arguments:
    /// 
    /// interface: The hardware interface the process must be to be connected to
    fn connect_interface(&mut self, interface: &mut dyn HardwareInterface);
}