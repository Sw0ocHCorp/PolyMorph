# PolyMorph documentation

An [mdBook](https://rust-lang.github.io/mdBook/) covering the PolyMorph control stack: the framework, the mathematics it uses, and each robotics pipeline built on top of it.

## Read it

```bash
cargo install mdbook      # once
cd docs && mdbook serve --open
```

`mdbook build` writes a static site into `docs/book/` (git-ignored). The diagrams are inline SVG and follow the book's light/dark theme; they do **not** render in GitHub's Markdown viewer, so read the built book rather than the raw files.

## Contents

| Part | Content |
|---|---|
| **Framework** | architecture, scheduler and `Process`, message catalogue, communications, and the conventions every component must respect |
| **Mathematical foundations** | frames and rotations, rigid-body efforts, cascaded control, constrained least squares |
| **Pipeline: motor control** | overview, motor model, mixer, attitude controller, roadmap, validation ladder, lessons learned |
| **Simulation** | the Gazebo bridge and the vehicle model |
| **Glossary** | every symbol with its unit and the code field that carries it |

Source comments are in English and point to these pages for the theory; `cargo doc --open` in `robomorph/` builds the matching API documentation.

New pipelines get their own part — see [Extending this documentation](src/contributing.md) for the conventions (structure, writing rules, how to draw a diagram).
