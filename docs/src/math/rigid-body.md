# Rigid-body efforts: wrench, mass and inertia

## The wrench: a 6D effort

A **wrench** is a six-component vector that completely describes the effort applied to a rigid body:

```
w = [ fx  fy  fz  |  mx  my  mz ]
      ─── force ──   ── moment ──
          (N)           (N·m)
```

The first three are a **force**, the last three a **moment** (a torque). Throughout the stack a wrench is expressed in the **body** frame and its moments are taken about the **reduction point** — the centre of mass (CoM). The Rust type is `WorkVec`.

The wrench is the **pivot of the chain**: the attitude controller publishes "the wrench to produce", the mixer finds the motor commands that produce it. Neither needs to know what the other is.

## Why a reduction point

A moment depends on the point it is computed about: the same rotor force creates a different moment measured at the CoM or at the model's origin. Every lever arm in the stack is measured from the CoM (`com_relative_location` offsets the root motors), so that the sum of all motor wrenches is directly the effort that rotates the vehicle about its centre of mass.

<figure>
<svg class="diagram" viewBox="0 0 700 220" xmlns="http://www.w3.org/2000/svg" role="img" aria-label="A force applied at a rotor is equivalent to the same force at the centre of mass plus a moment equal to p cross f">
  <defs>
    <marker id="w-arw" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">
      <path d="M1 1 L9 5 L1 9" fill="none" stroke="#2aa198" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round"/>
    </marker>
    <marker id="w-arw-c" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">
      <path d="M1 1 L9 5 L1 9" fill="none" stroke="#d4694a" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round"/>
    </marker>
  </defs>
  <rect x="100" y="132" width="170" height="30" rx="10" fill="currentColor" fill-opacity="0.10" stroke="currentColor" stroke-opacity="0.45"/>
  <circle cx="150" cy="147" r="4.5" fill="currentColor"/>
  <text x="150" y="185" text-anchor="middle" font-size="10.5" opacity="0.8">CoM</text>
  <line x1="150" y1="147" x2="250" y2="147" stroke="currentColor" stroke-opacity="0.55" stroke-width="1.2" stroke-dasharray="4 3"/>
  <text x="200" y="164" text-anchor="middle" font-size="11" opacity="0.85">p</text>
  <circle cx="250" cy="147" r="3" fill="currentColor" fill-opacity="0.6"/>
  <line x1="250" y1="147" x2="250" y2="66" stroke="#2aa198" stroke-width="2.2" marker-end="url(#w-arw)"/>
  <text x="258" y="76" font-size="12" font-weight="600" fill="#2aa198">f</text>
  <text x="185" y="40" text-anchor="middle" font-size="11.5" opacity="0.8">force applied at the rotor</text>
  <text x="350" y="154" text-anchor="middle" font-size="22" opacity="0.6">=</text>
  <rect x="430" y="132" width="170" height="30" rx="10" fill="currentColor" fill-opacity="0.10" stroke="currentColor" stroke-opacity="0.45"/>
  <circle cx="480" cy="147" r="4.5" fill="currentColor"/>
  <text x="480" y="185" text-anchor="middle" font-size="10.5" opacity="0.8">CoM</text>
  <line x1="480" y1="147" x2="480" y2="66" stroke="#2aa198" stroke-width="2.2" marker-end="url(#w-arw)"/>
  <text x="488" y="76" font-size="12" font-weight="600" fill="#2aa198">f</text>
  <path d="M516 128 A 40 40 0 1 1 505 178" fill="none" stroke="#d4694a" stroke-width="2" marker-end="url(#w-arw-c)"/>
  <text x="556" y="120" font-size="12" font-weight="600" fill="#d4694a">M = p × f</text>
  <text x="515" y="40" text-anchor="middle" font-size="11.5" opacity="0.8">same force at the CoM, plus a moment</text>
</svg>
<figcaption>Reducing an effort to the centre of mass: this is what every motor column computes.</figcaption>
</figure>

### Two meanings of the word "moment"

Beware of the homonym:

- a **moment** as a *torque* — the `mx, my, mz` part of a wrench, in N·m;
- a **moment of inertia** — resistance to being spun up, in kg·m².

The `moments_matrix` field of `VehicleKinematicConfig` is the matrix of **moments of inertia** (the inertia matrix), not a wrench. In formulas, `m` is mass and `M` is a moment-torque.

## Mass and inertia: the "mass" of rotation

The linear / angular symmetry structures the whole chain:

| | Translation | Rotation |
|---|---|---|
| What resists | mass `m` (kg) | moment of inertia `I` (kg·m²) |
| Newton | `F = m·a` | `M = I·α` |
| In the config | `weight` | `moments_matrix` |

Mass says how hard it is to *accelerate* the vehicle in a straight line. The moment of inertia says how hard it is to *spin* it — and unlike mass it **depends on the axis**. The unit tells the story: kg·**m²** is mass weighted by the square of its distance to the axis. A kilogram at the tip of an arm resists rotation enormously; the same kilogram at the centre barely at all.

<figure>
<svg class="diagram" viewBox="0 0 700 180" xmlns="http://www.w3.org/2000/svg" role="img" aria-label="Same mass close to the rotation axis gives a small inertia, spread far from the axis gives a large inertia">
  <line x1="185" y1="30" x2="185" y2="150" stroke="currentColor" stroke-opacity="0.45" stroke-width="1.2" stroke-dasharray="5 4"/>
  <line x1="515" y1="30" x2="515" y2="150" stroke="currentColor" stroke-opacity="0.45" stroke-width="1.2" stroke-dasharray="5 4"/>
  <text x="185" y="24" text-anchor="middle" font-size="10.5" opacity="0.75">rotation axis</text>
  <text x="515" y="24" text-anchor="middle" font-size="10.5" opacity="0.75">rotation axis</text>
  <line x1="150" y1="90" x2="220" y2="90" stroke="currentColor" stroke-opacity="0.5" stroke-width="1.6"/>
  <circle cx="150" cy="90" r="13" fill="#4a90d9" fill-opacity="0.3" stroke="#4a90d9" stroke-width="1.3"/>
  <circle cx="220" cy="90" r="13" fill="#4a90d9" fill-opacity="0.3" stroke="#4a90d9" stroke-width="1.3"/>
  <line x1="420" y1="90" x2="610" y2="90" stroke="currentColor" stroke-opacity="0.5" stroke-width="1.6"/>
  <circle cx="420" cy="90" r="13" fill="#4a90d9" fill-opacity="0.3" stroke="#4a90d9" stroke-width="1.3"/>
  <circle cx="610" cy="90" r="13" fill="#4a90d9" fill-opacity="0.3" stroke="#4a90d9" stroke-width="1.3"/>
  <text x="185" y="140" text-anchor="middle" font-size="12" font-weight="600">small I</text>
  <text x="515" y="140" text-anchor="middle" font-size="12" font-weight="600">large I</text>
  <text x="350" y="168" text-anchor="middle" font-size="10.5" opacity="0.75">same total mass — only the distance to the axis changed</text>
</svg>
<figcaption>The skater who pulls their arms in: same mass, smaller inertia, faster spin.</figcaption>
</figure>

Hence **one inertia per axis**. On the OSPRAI the yaw inertia is about twice the roll inertia, because the arms and rotors are spread out with respect to the vertical axis. To obtain the *same* angular acceleration on both axes you must ask for twice the torque in yaw — which is exactly the information `M = I·α` injects into the law.

### The 3×3 matrix, and why only the diagonal

The full form is a 3×3 tensor. The diagonal (`I_xx, I_yy, I_zz`) carries the inertias *about* each axis. The off-diagonal terms (products of inertia) encode the couplings of an asymmetric mass distribution: on a lopsided vehicle a pure torque about x also produces a parasitic acceleration about y. For a roughly symmetric vehicle — nearly every drone — those terms vanish, the matrix is diagonal, and `M = I·α` decouples into three scalar lines `M_k = I_k·α_k`. That is what licenses the "one loop per axis" structure of the attitude controller. On a strongly asymmetric vehicle you would keep the full matrix product: same formula, just not simplified.

### Where the numbers come from

These are **physical properties of the airframe**, like mass — not tunings. In Gazebo they are declared in the SDF (`<inertia>`: `ixx, iyy, izz`, …). The controller's config must carry the **total** inertia of the vehicle (base + arms + rotors, combined through the parallel-axis theorem), not just the base link's. On real hardware: CAD or a pendulum measurement.

A wrong inertia is not tuned away, it is corrected — and it is visible: if `I_config < I_real`, both gains of the attitude loop are too small by the same factor and the response becomes **both slower and less damped** (see [Validation](../pipelines/motor-control/validation.md)).

## Row normalisation in the allocator

The mixer minimises `‖A·dx − b‖²` where `b` mixes newtons and newton-metres. Adding N² and (N·m)² in one norm is dimensionally meaningless: the answer would change if lengths were expressed in millimetres. Every row is therefore divided so the residual is expressed as an **acceleration**:

```
force row       / m
moment row k    / sqrt(m · I_k)        (m/s² seen at the radius of gyration)
```

This is not a tuning but a correctness requirement: it decides, physically rather than by unit accident, how the solver arbitrates between a force error and a moment error. The same mass and inertia therefore appear twice in the chain: at the end of the control law (`M = I·α`, `f = m·a`) and in the mixer's normalisation.
