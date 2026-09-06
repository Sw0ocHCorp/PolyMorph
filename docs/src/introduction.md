# Introduction

PolyMorph is an open-source robotics stack written in Rust. This book documents the **software framework** and the **control pipelines** built on top of it: what each component does, which mathematics it implements, and why it is built the way it is.

## What the stack is for

The goal is a control stack that is **vehicle-agnostic**. The same code must be able to drive a multirotor, a tilt-rotor bicopter, a wheeled rover or any vehicle that can be described as a *tree of motors*. Specialisation to a given vehicle is **data** — mass, inertia, geometry, and the family of each motor — never code: there is no `UAV` subclass and no `UGV` subclass.

The validation vehicle is the **OSPRAI**, a tilt-rotor bicopter (two rotors on two tilting arms) simulated in Gazebo Harmonic. It is a test bench, not a target.

## The first pipeline: motor control

<figure>
<svg class="diagram" viewBox="0 0 700 210" xmlns="http://www.w3.org/2000/svg" role="img" aria-label="Closed control loop: attitude controller, mixer, vehicle, and the pose feedback path">
  <defs>
    <marker id="i-arw" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">
      <path d="M1 1 L9 5 L1 9" fill="none" stroke="currentColor" stroke-width="1.6" stroke-linecap="round" stroke-linejoin="round"/>
    </marker>
  </defs>
  <rect x="96" y="34" width="150" height="62" rx="6" fill="currentColor" fill-opacity="0.06" stroke="#4a90d9" stroke-width="1.2"/>
  <text x="171" y="60" text-anchor="middle" font-size="13" font-weight="600">Attitude</text>
  <text x="171" y="78" text-anchor="middle" font-size="13" font-weight="600">controller</text>
  <rect x="296" y="34" width="120" height="62" rx="6" fill="currentColor" fill-opacity="0.06" stroke="#2aa198" stroke-width="1.2"/>
  <text x="356" y="70" text-anchor="middle" font-size="13" font-weight="600">Mixer</text>
  <rect x="466" y="34" width="150" height="62" rx="6" fill="currentColor" fill-opacity="0.06" stroke="#cb8b1e" stroke-width="1.2"/>
  <text x="541" y="60" text-anchor="middle" font-size="13" font-weight="600">Motors and</text>
  <text x="541" y="78" text-anchor="middle" font-size="13" font-weight="600">vehicle</text>
  <line x1="18" y1="65" x2="90" y2="65" stroke="currentColor" stroke-width="1.3" marker-end="url(#i-arw)"/>
  <text x="54" y="52" text-anchor="middle" font-size="11" opacity="0.75">setpoint</text>
  <line x1="246" y1="65" x2="290" y2="65" stroke="currentColor" stroke-width="1.3" marker-end="url(#i-arw)"/>
  <text x="268" y="28" text-anchor="middle" font-size="11" opacity="0.75">wrench</text>
  <line x1="416" y1="65" x2="460" y2="65" stroke="currentColor" stroke-width="1.3" marker-end="url(#i-arw)"/>
  <text x="438" y="28" text-anchor="middle" font-size="11" opacity="0.75">efforts</text>
  <path d="M541 96 L541 162 L171 162 L171 102" fill="none" stroke="currentColor" stroke-width="1.3" stroke-dasharray="5 4" marker-end="url(#i-arw)"/>
  <text x="356" y="180" text-anchor="middle" font-size="11" opacity="0.75">measured attitude and angular rate</text>
</svg>
<figcaption>The motor-control loop in one picture. Every stage produces exactly the input of the stage below it.</figcaption>
</figure>

A single 6-dimensional quantity — the **wrench** (three forces and three moments) — is the pivot of the whole chain. Above it, control laws decide what effort the vehicle should apply; below it, an allocator decides how the motors produce that effort. Neither side needs to know anything about the other, which is what makes the stack generic.

## How to read this book

| Part | Content |
|---|---|
| **Framework** | the general code stack: scheduler, message catalogue, communications, and the invariants every component must respect |
| **Mathematical foundations** | the mathematics the pipelines rely on: frames and rotations, rigid-body efforts, cascaded control, constrained least squares |
| **Pipeline: motor control** | the first robotics pipeline, stage by stage, with its own mathematics, its validation ladder and the bugs it produced |
| **Simulation** | the Gazebo bridge and the vehicle model |
| **Glossary** | every symbol with its unit and the code field that carries it |

New pipelines (perception, navigation, planning) will each get their own part, built the same way: overview, components, mathematics, validation. [Extending this documentation](contributing.md) describes that convention.

## Notation conventions

- Symbols follow the [glossary](glossary.md). Two letters recur and must not be confused: `m` is **mass**, `M` is a **moment** (a torque).
- Frames are always named: *body* (attached to the vehicle) or *world* (fixed). A quantity with no explicit frame is in the body frame.
- Code is cited by file path. Source comments are in English, like this book; `cargo doc --open` in `robomorph/` produces the matching API documentation.
- Diagrams are inline SVG and follow the book's light/dark theme. They do not render on GitHub's Markdown viewer — read the book with `mdbook serve`.
