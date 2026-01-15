use godot::prelude::*;

pub mod autonomy_node;

pub struct PolyMorph;

#[gdextension]
unsafe impl ExtensionLibrary for PolyMorph {}
