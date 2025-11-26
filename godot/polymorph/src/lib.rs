use godot::prelude::*;


pub mod flight_controller;

pub struct PolyMorph;

#[gdextension]
unsafe impl ExtensionLibrary for PolyMorph {}
