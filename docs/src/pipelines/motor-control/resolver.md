# The force → attitude resolver

The stage between the velocity loop and the attitude loop. It answers one question: **which attitude must the vehicle take so that the force it is asked for becomes producible?** — and it answers it for any vehicle, from the data that describes it.

## Contract

| | |
|---|---|
| **Inputs** | `f_d`: desired force, **world** frame, N (from the velocity loop; in stabilize, the constant `(0, 0, m·g)`) |
| | `q_pref`: attitude **preference**, from the external mission setpoint (`Pose.orientation`) |
| | `q`: current attitude — only for tie-breaking and hysteresis, never for the alignment itself |
| **Vehicle data** | the reachable force subspace `S` (body frame) and the authority axes `𝒜` — both derived from the motor tree at start-up, see below |
| **Output** | the attitude stage's setpoint: `q_d`, `ω_d` (zero in v1), and `f_d` passed through unchanged (world, N) |
| **State** | the last valid alignment direction (guard against `‖f_d‖ → 0`) and, for bidirectional thrusters, the last forward/reverse choice (hysteresis). Nothing else. |

`f_d` is **not** consumed here: it travels down to the attitude stage, which rotates it into the body frame with `q⁻¹` and hands it to the mixer inside the wrench. While the vehicle is not yet aligned, the mixer serves the reachable projection of that force and leaves the rest in its residual — the transient sag this causes is corrected by the attitude loop within `τ`, and by the velocity loop's integrator for whatever remains.

## The degree-of-freedom budget

A rotation has three degrees of freedom. The resolver consumes exactly as many as it needs to make `f_d` reachable, and hands the rest to the preference. How many it needs depends only on the dimension `k` of `S`:

| `k` | `S` is | The constraint says | DOF consumed | Free DOF |
|---|---|---|---|---|
| 3 | all of space | nothing | 0 | 3 — `q_d = q_pref` |
| 2 | a plane with unit normal `n` | `n ⟂ f_d` | 1 | 2 |
| 1 | a line along unit vector `t` | `t ∥ f_d` | 2 | 1 (rotation about `f_d`) |

A holonomic ground vehicle (`S` = the horizontal plane, authority about z only) consumes nothing: its heading is entirely free. A differential-drive rover (`S` = its x axis, one authority axis) consumes its only degree of freedom: its heading is dictated by the force. A multirotor keeps yaw. The OSPRAI, whose arms tilt, has `S` = its xz plane and keeps **two** free degrees of freedom — the concrete payoff of the tilting arms, and it falls out of the count without any vehicle-specific code.

## The alignment: one formula for k = 1 and k = 2

Both non-trivial cases are the same operation: *bring one body vector to a target angle with `f̂_d` by the smallest possible rotation*. Call `v` the **characteristic vector** of `S` — its direction `t` when `k = 1`, its normal `n` when `k = 2` — and `γ*` the target angle: `0` for `k = 1` (align), `π/2` for `k = 2` (make perpendicular).

<figure>
<svg class="diagram" viewBox="0 0 700 260" xmlns="http://www.w3.org/2000/svg" role="img" aria-label="The characteristic vector v of the preferred attitude is rotated about the axis u, perpendicular to both v and the desired force direction, until it reaches the target angle with that direction">
  <defs>
    <marker id="r-arw" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">
      <path d="M1 1 L9 5 L1 9" fill="none" stroke="currentColor" stroke-width="1.6" stroke-linecap="round" stroke-linejoin="round"/>
    </marker>
    <marker id="r-arw-b" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">
      <path d="M1 1 L9 5 L1 9" fill="none" stroke="#4a90d9" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round"/>
    </marker>
    <marker id="r-arw-p" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">
      <path d="M1 1 L9 5 L1 9" fill="none" stroke="#8b7cd8" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round"/>
    </marker>
    <marker id="r-arw-c" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">
      <path d="M1 1 L9 5 L1 9" fill="none" stroke="#d4694a" stroke-width="1.6" stroke-linecap="round" stroke-linejoin="round"/>
    </marker>
  </defs>
  <line x1="350" y1="210" x2="350" y2="60" stroke="#4a90d9" stroke-width="2.2" marker-end="url(#r-arw-b)"/>
  <text x="358" y="66" font-size="12" font-weight="600" fill="#4a90d9">f̂_d</text>
  <text x="358" y="82" font-size="10" fill="#4a90d9" opacity="0.85">desired force direction</text>
  <line x1="350" y1="210" x2="238" y2="110" stroke="#8b7cd8" stroke-width="2" marker-end="url(#r-arw-p)"/>
  <text x="150" y="104" font-size="12" font-weight="600" fill="#8b7cd8">v_w = q_pref · v</text>
  <text x="150" y="120" font-size="10" fill="#8b7cd8" opacity="0.85">characteristic vector, world</text>
  <path d="M350 140 A 70 70 0 0 0 296 161" fill="none" stroke="#d4694a" stroke-width="1.6" marker-end="url(#r-arw-c)"/>
  <text x="305" y="134" font-size="13" font-weight="600" fill="#d4694a">γ</text>
  <circle cx="350" cy="210" r="7" fill="none" stroke="currentColor" stroke-opacity="0.7" stroke-width="1.2"/>
  <circle cx="350" cy="210" r="2" fill="currentColor"/>
  <text x="350" y="240" text-anchor="middle" font-size="10.5" opacity="0.8">u = v_w × f̂_d — the rotation axis, out of the page</text>
  <line x1="470" y1="210" x2="600" y2="210" stroke="currentColor" stroke-opacity="0.35" stroke-width="1" stroke-dasharray="4 4"/>
  <text x="535" y="200" text-anchor="middle" font-size="10" opacity="0.7">k = 2: target γ* = 90°</text>
  <text x="535" y="228" text-anchor="middle" font-size="10" opacity="0.7">k = 1: target γ* = 0</text>
</svg>
<figcaption>The smallest rotation taking the characteristic vector to its target angle is about the axis perpendicular to both vectors.</figcaption>
</figure>

```
f̂_d    = f_d / ‖f_d‖                      unit desired direction, world
v_w    = q_pref · v                        characteristic vector of the PREFERRED attitude, world
c      = v_w · f̂_d
γ      = acos(c)                           current angle between them
u      = (v_w × f̂_d) / ‖v_w × f̂_d‖        rotation axis, world
q_align = Rot(u, γ − γ*)                   turn v_w TOWARD f̂_d by (γ − γ*); negative = away
q_d    = q_align ⊗ q_pref                  a world-frame rotation applied after the preference
```

Why this is right, and why it is generic:

- **It is the closest constrained attitude to the preference.** The smallest rotation that repairs a violated single-vector constraint yields the attitude nearest to `q_pref` (in rotation angle) that satisfies it. Everything the preference asked for and that the force does not forbid is kept — this is the "project the preference onto the free subspace" step, done implicitly.
- **It is computed from `q_pref`, never from `q`.** Aligning from the current attitude every cycle would let the free degrees of freedom random-walk (yaw drift on a multirotor). Aligning from the preference is stateless and drift-free.
- **The force outranks the preference by construction.** No conflict handling is needed: the rotation is applied whatever the preference said.
- **Yaw is preserved where it should be.** A rotation about a horizontal axis leaves the heading unchanged to first order; the classic multirotor construction "`z_d = f̂_d`, then the preferred heading projected onto the plane ⟂ `z_d`" (Lee, Leok & McClamroch, 2010) and the formula above agree to second order in tilt and are interchangeable for the attitude setpoint.

Sign check with a multirotor (`k = 1`, `t = +z` body, `q_pref` level): `f_d = (0, +f_y, +m·g)` gives `u = ẑ × f̂_d ∝ (−f_y, 0, 0)`, a rotation about `−x`: the vehicle rolls to push toward `+y`. The direction of that roll must be confirmed by the open-loop sign test, like every convention in this stack.

## Restricting the rotation to the authority axes

The formula above assumes the vehicle can rotate about `u`. When it cannot — a ground vehicle has moment authority about z only — the alignment must be found among the rotations it *can* perform. In general this is a small optimisation: choose the rotation about `span(𝒜)` that minimises the unreachable part `‖f_d − P_{q_d·S} f_d‖`. The two cases that exist in practice have closed forms:

- **full authority** (aerial vehicles): the formula above, unrestricted;
- **one authority axis `a`** (ground vehicles, `a = z`): only the components of `f_d` in the plane perpendicular to `a` can ever be served. Project: `f_h = f_d − (f_d·â)·â`. Then align the characteristic vector with `f_h` by a rotation about `â` alone — for a differential rover this is simply the heading `atan2(f_h.y, f_h.x)`. The vertical part of `f_d` (the gravity feedforward) is unreachable whatever the heading; it stays in the mixer residual, harmlessly, until wheel–ground contact is modelled as an effectiveness column.

## Bidirectional thrust: the forward / reverse choice

When the thrusters along `t` can push both ways (`min_value < 0` — wheels), `−t` is as good a candidate as `t`. Evaluate the alignment for both and keep the one requiring the smaller rotation **from the current attitude** `q` — this is the one place where `q` legitimately enters. Add **hysteresis**: do not switch from one candidate to the other unless the other is better by a margin (a few degrees, or `90° + margin` on the heading difference), otherwise a force pointing sideways makes the vehicle chatter between facing it and backing into it.

## Degeneracies and guards

| Situation | Symptom | Handling |
|---|---|---|
| `‖f_d‖ → 0` | `f̂_d` undefined | if `‖f_d‖ < ε·m·g`, keep the **last valid** `q_d` (the resolver's only state). Free fall is never a valid demand for an aerial vehicle; for a ground vehicle, a stopped vehicle simply keeps its heading |
| `v_w ∥ f̂_d`, `k = 1`, same direction | `u` undefined, `γ = 0` | already aligned: `q_align = identity` |
| `v_w ∥ f̂_d`, `k = 1`, opposite | `γ = π`, any axis works | pick the preference's x axis as `u` — only happens inverted |
| `v_w ∥ f̂_d`, `k = 2` | the plane's normal points along the force | a 90° rotation is needed about any axis ⟂ `f̂_d`; pick the one in the plane closest to the current attitude — practically unreachable |
| tilt beyond what is safe | `γ − γ*` too large | optional **tilt limit**, a vehicle-level scalar with a sane default (~35°): scale down the component of `f_d` perpendicular to the vertical until the required rotation fits. Never scale the vertical component first — keeping altitude outranks lateral acceleration |
| `‖f_d‖` beyond the thrusters' capability | the mixer saturates | the velocity loop must saturate its own output (every stage saturates to what the next can serve); `T_max` is computable from the thrusters' `max_value` along `S`. Saturation here must be reported upstream for conditional integration |

## Where `S` and `𝒜` come from

Never from a per-vehicle flag. Both are derived **once, at start-up, from the motor tree** — the same configuration channel the mixer subscribes to:

- **`S`, the reachable force subspace.** For each thruster, take its rest thrust direction `fhat`. If it has no joint ancestor, it contributes the line `span{fhat}`. If it hangs under a joint of axis `a`, it can be swept about that axis and contributes the plane `span{fhat, a × fhat}` (assuming a useful range — the OSPRAI arms cover ±90°). `S` is the span of all contributions, orthonormalised (Gram–Schmidt or an SVD); `k` is its rank, and its characteristic vector is its direction (`k = 1`) or its normal (`k = 2`).
- **`𝒜`, the authority axes.** The body axes on which some motor has a *structurally* non-zero moment. Evaluate the effectiveness columns at a **nominal** thrust (hover: `m·g / n_thrusters`), not at zero — joint columns vanish at zero thrust and would hide their authority — and take the support of the moment rows.

For the OSPRAI this yields `S` = the body xz plane (`k = 2`, `v = ŷ`) and full authority; for a quadrotor `S` = the z axis (`k = 1`, `v = ẑ`); for a differential rover `S` = the x axis with `𝒜 = {z}`. Same code, three vehicles.

## What the attitude stage receives

The resolver emits the **internal** setpoint of the attitude stage — a message distinct from the external mission `Pose`, carrying:

| Field | Content | Frame / unit |
|---|---|---|
| `orientation` | `q_d` | — |
| `angular_velocity` | `ω_d`, zero in v1 (a rate feedforward from the time derivative of `q_d` is a later refinement for fast-changing forces) | body, rad/s |
| `force` | `f_d`, unchanged | **world**, N |

The attitude stage rotates the force into the body frame and assembles the wrench `[q⁻¹·f_d ; M]`. In stabilize without a velocity loop, the resolver still runs: `f_d = (0, 0, m·g)` is reachable at the preferred attitude (level), `q_d = q_pref`, and the constant feedforward flows through the same path — no mode flag anywhere.

## Validation

The stage is almost stateless, so its ladder starts with pure unit tests (rung A), each with a value predicted by hand:

| Case (`k = 1`, `t = ẑ`) | Expected |
|---|---|
| `f_d = (0, 0, m·g)`, `q_pref` = identity | `q_d` = identity |
| same `f_d`, `q_pref` = 30° yaw | `q_d` = 30° yaw — yaw is free |
| `f_d = (m·g·tan 20°, 0, m·g)`, `q_pref` = identity | a 20° pitch about y, sign per the convention test |
| `f_d = (0, m·g·tan 20°, m·g)`, `q_pref` = 90° yaw | 20° of tilt, heading still 90° |
| `‖f_d‖ = 0` | previous `q_d` held, no `NaN` |

| Case (`k = 2`, `v = ŷ`, the OSPRAI) | Expected |
|---|---|
| `f_d = (f_x, 0, f_z)`, any `q_pref` level | `q_d = q_pref` — in-plane, the arms will serve `f_x` |
| `f_d = (0, f_y, f_z)` | a pure roll of `atan2(f_y, f_z)` |

| Case (rover, `S = x̂`, `𝒜 = {z}`) | Expected |
|---|---|
| `f_d = (1, 1, m·g)` | heading 45°, the vertical part ignored |
| `f_d = (−1, 0, m·g)`, current heading 0, bidirectional wheels | heading kept at 0, reverse chosen |

Rung B (open loop, Gazebo): command a lateral `f_d` by hand, log `q_d`, and check the tilt sign against the direction the vehicle actually moves once the loop is closed. Rung D (closed loop): a lateral velocity step — the vehicle tilts by `atan(a_d/g)`, accelerates, and the velocity loop's integrator settles the residual; `‖e_R‖(t)` of the attitude stage must still follow its own `(τ, ζ)`.
