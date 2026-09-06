# Frames, vectors and rotations

All attitude control rests on one idea: **a physical vector and its coordinates are two different things**. This chapter builds that idea up to the formulas the controller implements.

## A vector and its coordinates

A physical vector is an arrow in space: "up", "the left rotor's thrust", "the vehicle's velocity". The arrow exists independently of any observer. To turn it into numbers — three of them — you must pick **three reference axes**. Those three numbers do not describe the arrow alone; they describe the arrow *seen through those axes*.

The stack uses two sets of axes:

| Frame | Definition | What is natural in it |
|---|---|---|
| **world** | fixed, planted in the ground (Gazebo: `z` up) | gravity, a GPS position, a heading |
| **body** | bolted to the airframe, tilts with the vehicle (`z` = thrust axis) | motors, the gyroscope, moments |

<figure>
<svg class="diagram" viewBox="0 0 700 300" xmlns="http://www.w3.org/2000/svg" role="img" aria-label="The same up vector on the same tilted vehicle, measured with world axes on the left and with body axes on the right">
  <defs>
    <marker id="f-arw" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">
      <path d="M1 1 L9 5 L1 9" fill="none" stroke="currentColor" stroke-width="1.6" stroke-linecap="round" stroke-linejoin="round"/>
    </marker>
    <marker id="f-arw-t" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="6.5" markerHeight="6.5" orient="auto">
      <path d="M1 1 L9 5 L1 9" fill="none" stroke="#2aa198" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"/>
    </marker>
  </defs>
  <line x1="350" y1="24" x2="350" y2="276" stroke="currentColor" stroke-opacity="0.18" stroke-width="1" stroke-dasharray="4 5"/>
  <text x="185" y="34" text-anchor="middle" font-size="12" font-weight="600">measured with the world axes</text>
  <text x="515" y="34" text-anchor="middle" font-size="12" font-weight="600">measured with the body axes</text>
  <g stroke="currentColor" stroke-opacity="0.55" stroke-width="1.3">
    <line x1="62" y1="252" x2="62" y2="192" marker-end="url(#f-arw)"/>
    <line x1="62" y1="252" x2="122" y2="252" marker-end="url(#f-arw)"/>
  </g>
  <text x="70" y="188" font-size="10.5" opacity="0.8">z world</text>
  <text x="126" y="256" font-size="10.5" opacity="0.8">y world</text>
  <g stroke="#8b7cd8" stroke-width="1.4">
    <line x1="392" y1="252" x2="362" y2="200" marker-end="url(#f-arw)"/>
    <line x1="392" y1="252" x2="444" y2="222" marker-end="url(#f-arw)"/>
  </g>
  <text x="330" y="196" font-size="10.5" fill="#8b7cd8">z body</text>
  <text x="448" y="220" font-size="10.5" fill="#8b7cd8">y body</text>
  <rect x="135" y="172" width="100" height="15" rx="5" fill="currentColor" fill-opacity="0.18" stroke="currentColor" stroke-opacity="0.5" transform="rotate(-30 185 180)"/>
  <rect x="465" y="172" width="100" height="15" rx="5" fill="currentColor" fill-opacity="0.18" stroke="currentColor" stroke-opacity="0.5" transform="rotate(-30 515 180)"/>
  <line x1="185" y1="180" x2="185" y2="96" stroke="#2aa198" stroke-width="2.4" marker-end="url(#f-arw-t)"/>
  <line x1="515" y1="180" x2="515" y2="96" stroke="#2aa198" stroke-width="2.4" marker-end="url(#f-arw-t)"/>
  <text x="193" y="104" font-size="11" fill="#2aa198" font-weight="600">up</text>
  <text x="523" y="104" font-size="11" fill="#2aa198" font-weight="600">up</text>
  <text x="185" y="286" text-anchor="middle" font-size="12.5" font-weight="600">up = (0, 0, 1)</text>
  <text x="515" y="286" text-anchor="middle" font-size="12.5" font-weight="600">up = (0, 0.50, 0.87)</text>
</svg>
<figcaption>Same vehicle tilted 30° in roll, same arrow, drawn identically. Only the measuring axes changed.</figcaption>
</figure>

## The attitude quaternion as a dictionary

The vehicle's attitude is a unit quaternion `q`. The most useful way to read it: **`q` is a body → world translation dictionary**. Give it the coordinates of an arrow measured with the body axes, it returns the coordinates of *the same arrow* measured with the world axes.

- `q · (0, 0, 1)` = where the thrust axis points, in world coordinates.
- Level flight: `q` is the identity `(w=1, x=0, y=0, z=0)` — the dictionary is trivial.

Saying "`q` represents the orientation" and saying "`q` translates body → world" is the same sentence: the orientation *is* the translation. There are not two objects.

### The inverse `q⁻¹`

`q⁻¹` is the dictionary the other way round: **world → body**. For a unit quaternion the inverse is the conjugate — negate the vector part:

```
q   = (w,  x,  y,  z)
q⁻¹ = (w, −x, −y, −z)          valid ONLY when ‖q‖ = 1
```

Geometrically `q⁻¹` is the same rotation the other way (angle `−θ` about the same axis). It shows up wherever a world-frame quantity must be handed to a body-frame component:

- **gravity feedforward**: the compensation force `(0, 0, m·g)` is known in the world; the mixer speaks body; hence `f_body = q⁻¹ · (0, 0, m·g)`.
- **attitude error**: see below.

Mnemonic: an arrow *bolted to the vehicle* has trivial coordinates in body (the thrust axis is always `(0,0,1)` there) — to know where it points in the world, use `q`. An arrow *planted in the world* has trivial coordinates in world (up is always `(0,0,1)`) — to know how the vehicle sees it, use `q⁻¹`.

> **Implementation pitfall.** `q⁻¹ = q*` holds **only for a unit quaternion**. The `UnitQuat` coming off the wire is four `double`s with no norm guarantee. `UnitQuaternion::from(UnitQuat)` renormalises: always convert **before** inverting, never negate the message fields by hand.

## Composition and the attitude error

A composition `a ⊗ b` reads right to left as a chain of translations. The controller's attitude error is:

```
q_err = q⁻¹ ⊗ q_d
```

*"Go from the desired body frame to the world (`q_d`), then from the world to the current body frame (`q⁻¹`)"* — the result is the rotation separating the current body frame from the desired one, **expressed in the current body frame**: the frame of the gyroscope, and the frame in which the mixer produces moments. The other order (`q_d ⊗ q⁻¹`) gives the same physical rotation expressed in the world, and the moments would land on the wrong axes as soon as the vehicle tilts. That bug is invisible while level (`q ≈ q_d ≈ identity` makes both conventions agree), which is why validation includes an [open-loop sign test with the vehicle tilted](../pipelines/motor-control/validation.md).

## Euler's theorem, θ and the axis e

However two orientations differ, you can always go from one to the other by **a single rotation, of a single angle θ, about a single axis e** (Euler's rotation theorem). The axis `e` is any unit vector of space: in general it is aligned with none of the body's X, Y, Z axes — it lies *between* them.

<figure>
<svg class="diagram" viewBox="0 0 700 270" xmlns="http://www.w3.org/2000/svg" role="img" aria-label="Left: the single rotation of angle theta about the axis e. Right: where theta hides inside the quaternion components.">
  <defs>
    <marker id="q-arw" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">
      <path d="M1 1 L9 5 L1 9" fill="none" stroke="currentColor" stroke-width="1.6" stroke-linecap="round" stroke-linejoin="round"/>
    </marker>
    <marker id="q-arw-c" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">
      <path d="M1 1 L9 5 L1 9" fill="none" stroke="#d4694a" stroke-width="1.6" stroke-linecap="round" stroke-linejoin="round"/>
    </marker>
    <marker id="q-arw-p" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">
      <path d="M1 1 L9 5 L1 9" fill="none" stroke="#8b7cd8" stroke-width="1.6" stroke-linecap="round" stroke-linejoin="round"/>
    </marker>
  </defs>
  <line x1="350" y1="20" x2="350" y2="250" stroke="currentColor" stroke-opacity="0.18" stroke-width="1" stroke-dasharray="4 5"/>
  <text x="185" y="32" text-anchor="middle" font-size="12" font-weight="600">one rotation: angle θ about axis e</text>
  <text x="515" y="32" text-anchor="middle" font-size="12" font-weight="600">where θ hides in q_err</text>
  <line x1="190" y1="200" x2="190" y2="92" stroke="currentColor" stroke-opacity="0.5" stroke-width="1.4" stroke-dasharray="5 4" marker-end="url(#q-arw)"/>
  <text x="198" y="90" font-size="10.5" opacity="0.8">current</text>
  <line x1="190" y1="200" x2="126" y2="123" stroke="#8b7cd8" stroke-width="1.8" marker-end="url(#q-arw-p)"/>
  <text x="70" y="116" font-size="10.5" fill="#8b7cd8">desired</text>
  <path d="M190 110 A 90 90 0 0 0 133 133" fill="none" stroke="#d4694a" stroke-width="1.6" marker-end="url(#q-arw-c)"/>
  <text x="152" y="98" font-size="13" font-weight="600" fill="#d4694a">θ</text>
  <circle cx="190" cy="200" r="7" fill="none" stroke="currentColor" stroke-opacity="0.7" stroke-width="1.2"/>
  <circle cx="190" cy="200" r="2" fill="currentColor"/>
  <text x="190" y="232" text-anchor="middle" font-size="10.5" opacity="0.75">axis e — out of the page</text>
  <line x1="452" y1="200" x2="640" y2="200" stroke="currentColor" stroke-opacity="0.5" stroke-width="1.2" marker-end="url(#q-arw)"/>
  <line x1="452" y1="200" x2="452" y2="70" stroke="currentColor" stroke-opacity="0.5" stroke-width="1.2" marker-end="url(#q-arw)"/>
  <path d="M602 200 A 150 150 0 0 1 452 50" fill="none" stroke="currentColor" stroke-opacity="0.3" stroke-width="1"/>
  <line x1="452" y1="200" x2="593" y2="149" stroke="#8b7cd8" stroke-width="1.8"/>
  <circle cx="593" cy="149" r="4.5" fill="#8b7cd8"/>
  <text x="601" y="143" font-size="11.5" font-weight="600" fill="#8b7cd8">q_err</text>
  <line x1="593" y1="149" x2="593" y2="200" stroke="currentColor" stroke-opacity="0.4" stroke-width="1" stroke-dasharray="3 3"/>
  <line x1="593" y1="149" x2="452" y2="149" stroke="currentColor" stroke-opacity="0.4" stroke-width="1" stroke-dasharray="3 3"/>
  <text x="523" y="219" text-anchor="middle" font-size="11">w = cos(θ/2)</text>
  <text x="446" y="140" text-anchor="end" font-size="11">‖v‖ = sin(θ/2)</text>
  <path d="M492 200 A 40 40 0 0 0 489 186" fill="none" stroke="#d4694a" stroke-width="1.4"/>
  <text x="500" y="188" font-size="11" fill="#d4694a">θ/2</text>
</svg>
<figcaption>The quaternion stores the pair (θ, e) with a <em>half</em>-angle — which is what makes the shortest-path test work.</figcaption>
</figure>

The quaternion encodes that pair as:

```
q_err = ( cos(θ/2) ,  e · sin(θ/2) )
          └─ w ─┘     └──── v ────┘
```

Checks: `θ = 0` gives `(1, 0, 0, 0)`, the identity; `θ = 180°` gives `(0, e)`.

- **θ** answers *"by how much?"* — one scalar in radians, the angular distance between the two attitudes.
- **e** answers *"in which direction?"*.

### Recovering θ: the logarithm

`(w, ‖v‖)` is a point on the unit circle at polar angle `θ/2`. Recovering an angle from its cosine and sine is exactly what `atan2` does:

```
θ   = 2 · atan2(‖v‖, w)
e_R = (θ / ‖v‖) · v         if ‖v‖ > ε
e_R ≈ 2 · v                 otherwise (small angle: v = e·sin(θ/2) ≈ e·θ/2)
```

`e_R = θ·e` is the **attitude error vector**: its norm is θ, its direction is the correction axis. Its component `e_R[k]` on body axis k is *"the share of the corrective rotation that happens about axis k"* — that is what the proportional term of the attitude loop consumes, axis by axis.

Two subtleties, both the source of real bugs:

> **The small-angle branch is not optional.** At equilibrium `‖v‖ → 0` and `θ/‖v‖` is `0/0`. The guard must test `‖v‖` — the **denominator** — *before* dividing, never test the result afterwards, because a `NaN` is unordered (`NaN <= ε` is false) and the fallback branch would never run. Use `atan2`, not `acos(w)`: `acos` has an infinite slope near 1 and amplifies noise precisely at equilibrium.

> **Double cover.** `q` and `−q` are the same rotation, but one encodes θ and the other `360° − θ` — the long way round. Because of the half-angle, `w = cos(θ/2)` turns negative exactly when θ exceeds 180°. Rule: **if `w < 0`, negate all four components** before the logarithm. Without it a −1° yaw error is corrected by a 359° turn. After the test, `θ ∈ [0, π]` by construction; a `θ > π` in a log is the unmistakable signature of the missing negation.

### Why e_R rather than Euler angles

Euler angles (roll, pitch, yaw) are three *successive* rotations about axes that move between steps; they are singular at ±90° of pitch, and their axes are not the body axes on which the mixer produces moments. `e_R` is the decomposition of *one* rotation onto the *current* body axes — the gyroscope's axes and the mixer's. For small errors `e_R ≈ (roll error, pitch error, yaw error)`, so the "three angles" intuition holds near equilibrium, while the quaternion form stays exact up to 180°.

### An axis between the axes

Suppose the vehicle has 10° of roll error *and* 10° of pitch error. For small angles the rotation vectors add:

```
e_R ≈ (10°, 10°, 0)   →   θ = ‖e_R‖ ≈ 14.1°,   e = (0.707, 0.707, 0)
```

The axis is the **45° diagonal between X and Y**. No motor is mounted on that axis — and none is needed, because **a moment is a vector**: producing a moment about `e` means producing the vector whose components on X, Y, Z are the projections. Serving the three components simultaneously *is* rotating about the diagonal. The body axes are a decomposition basis, nothing more.

> For large rotations this addition no longer holds — rotations do not commute, so the net axis of a composition is not the sum of the individual axes. That is precisely why `e_R` is never computed by adding angles by hand: composing the quaternions (`q⁻¹ ⊗ q_d`) performs the exact composition at any amplitude, and the vector part of the result already points along `e`.

## Convention checklist

| Topic | Convention in the stack | Trap |
|---|---|---|
| Direction of `q` | body → world | verify on any new sensor: `q·(0,0,1)` must point where the thrust axis physically points |
| `UnitQuat` field order | `w, x, y, z` (**w first**) | nalgebra stores and prints `(x, y, z, w)` |
| `UnitQuat` default | identity | the derived all-zero default has zero norm → `NaN` when normalised |
| Gazebo IMU | orientation referenced to the **world** through `<orientation_reference_frame>` (`parent_frame="world"`) | by default Gazebo references orientation to the sensor's **spawn pose**, so a vehicle spawned tilted reads the identity |
