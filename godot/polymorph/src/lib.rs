use godot::prelude::*;
pub mod flight_controller;
pub mod events_management;
pub mod com_channels;
pub mod process;

pub struct PolyMorph;

#[gdextension]
unsafe impl ExtensionLibrary for PolyMorph {}
