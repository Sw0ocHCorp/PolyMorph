//! Trait of the vehicle controllers: the bridge between the generic control stack and a concrete
//! vehicle, simulated (Gazebo) or real. See `docs/src/simulation/gazebo.md`.

use std::time::Duration;

use robomorph::messages::registered_message::AnyMessage;

/// Trait that give the fundamental methods of all the vehicle controllers connected to Gazebo simulation environment (Simulation)
/// The hardware / simulator boundary of the chain. An implementor owns the vehicle-specific
/// knowledge that the generic stack must not contain: topic names, the discovery of the motor tree,
/// the effort law `T = k * w^n` and its inverse (efforts <-> actuator units), and the ordering of
/// the actuator message. Everything upstream (mixer, attitude) works in efforts (N, rad) only.
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
    //fn apply_actuator_setpoints(&mut self, setpoints: Vec<AnyMessage>, dt: Duration);

    /// Send telemetry data to the telemetry receiver linked to telemetry_sender
    /// 
    /// Most likely to an interface
    fn send_telemetry(&mut self);
}