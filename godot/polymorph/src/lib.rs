use godot::prelude::*;

pub mod autonomy_node;
pub mod computer_vision;
pub mod flight_controller;
pub mod simcompanion;

pub struct PolyMorph;

#[gdextension]
unsafe impl ExtensionLibrary for PolyMorph {}
