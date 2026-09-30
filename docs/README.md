# PolyMorph documentation

A [Docusaurus](https://docusaurus.io/) site covering the PolyMorph control stack: the framework, the mathematics it uses, and each robotics pipeline built on top of it.

## Read it

Requires Node.js ≥ 20.

```bash
cd docs
npm install        # once
npm run start      # http://localhost:3000, live reload
```

`npm run build` writes the static site into `docs/build/` (git-ignored) and fails on any broken link or anchor. The diagrams are SVG files rendered inline so that they follow the light/dark theme, and some figures are interactive: read the site rather than the raw `.mdx` files.

## Contents

| Part | Content |
|---|---|
| **Framework** | architecture, the paradigm (rules P1–P10 and the review checklist), scheduler and processes pipelines, message catalogue, communications, conventions |
| **Mathematical foundations** | frames and rotations, rigid-body efforts, cascaded control (with an interactive step response), constrained least squares |
| **Robotics pipeline: motor control** | overview, motor model, mixer, attitude controller, force → attitude resolver, roadmap, validation ladder, lessons learned |
| **Simulation** | the Gazebo bridge and the vehicle model |
| **Glossary** | every symbol with its unit and the code field that carries it |
| **Changelog** | every commit, generated from the repository's `CHANGELOG.md` |

Source comments are in English and point to these pages for the theory; `cargo doc --open` in `robomorph/` builds the matching API documentation.

New pipelines get their own part — see [Extending this documentation](docs/contributing.mdx) for the conventions (structure, writing rules, diagrams, MDX pitfalls).
