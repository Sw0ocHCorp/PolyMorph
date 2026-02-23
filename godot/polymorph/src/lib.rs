use godot::prelude::*;

pub mod autonomy_node;
pub mod control_command;

pub struct PolyMorph;

#[gdextension]
unsafe impl ExtensionLibrary for PolyMorph {}
