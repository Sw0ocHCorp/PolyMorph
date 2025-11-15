use godot::init::*;  // or godot::prelude::* depending on your version

struct MyExtension;

#[gdextension]
unsafe impl ExtensionLibrary for MyExtension {
    // optionally override methods like on_level_init, etc.
}
