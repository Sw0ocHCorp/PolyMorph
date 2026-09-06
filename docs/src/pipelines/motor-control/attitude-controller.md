# The attitude controller

File: `robomorph/src/control/motion/attitude_controller.rs`. Stage v1 of the cascade (*stabilize* mode): it holds a desired attitude and compensates gravity.

## Contract

| | |
|---|---|
| **Measurements** | `q` (attitude, body→world) and `ω` (angular rate, body) — read from the same `Pose`, hence coherent |
| **Setpoint** | `q_d`, carried by a `Pose`. `ω_d` is implicitly zero in the current implementation (stabilize); the setpoint's `a_velocity` is not read yet |
| **Output** | an **absolute** `AnyMessage::VehicleWrench` `[f_body ; M]` to the mixer (pipe), or `None` (abstention) |
| **Tunings** | `τ` (s); `ζ` = 1 |
| **Vehicle data** | `m` (`weight`), the diagonal of `I` (`moments_matrix`) |

## The algorithm, cycle by cycle

```
1.  q, q_d → UnitQuaternion            (the conversion renormalises the wire quaternion)
2.  q_err = q⁻¹ ⊗ q_d                  (error in the body frame)
3.  if q_err.w < 0 : q_err = −q_err    (shortest path)
4.  θ = 2·atan2(‖v‖, w)                (v = vector part of q_err)
    e_R = (θ/‖v‖)·v    if ‖v‖ > ε
    e_R = 2·v          otherwise
5.  α = (1/τ²)·e_R + (2ζ/τ)·(ω_d − ω)  (rad/s²)
6.  M_k = I_k · α_k                     (N·m; k = x, y, z; element-wise with the inertia diagonal)
7.  f_body = q⁻¹ · (0, 0, m·g)          (N; gravity feedforward)
8.  publish [f_body ; M]
```

Steps 1–4 are justified in [Frames and rotations](../../math/frames-and-rotations.md), steps 5–7 in [Cascaded control](../../math/cascaded-control.md).

**Dimensional check**: `rad × 1/s²` and `rad/s × 1/s` both give rad/s²; `kg·m² × rad/s²` gives N·m. An equation whose units do not balance is wrong before it runs — that check is what exposed a missing inertia factor (a moment of 885 "N·m" on an axis whose inertia is 0.017 kg·m²).

## Deliberately absent

- no integrator (hence no anti-windup) — see [Cascaded control](../../math/cascaded-control.md);
- no gyroscopic term `ω × (I·ω)`: quadratic in ω, negligible in stabilize for a roughly diagonal inertia; relevant in acrobatic flight;
- no saturation on `α`: with no integrator, the mixer's clamps suffice;
- no special treatment of yaw;
- **no internal state** apart from the held setpoint. Same inputs ⇒ same outputs.

## Implementation notes

- **No `PIDController`.** The law is vectorial, the gains are identical on the three axes (inertia enters only afterwards), and `kp`/`kd` derive from a single `τ`. One nalgebra line, no state, fewer than a hundred flops per cycle: one quaternion product, one `atan2`, two scale-and-adds, three multiplications, one vector rotation. Precompute `kp = 1/τ²` and `kd = 2/τ` once — not for speed, but so that the code shows the single place where `τ` generates both gains.
- **Setpoint held, measurement fresh.** The setpoint is stored and served until a new one arrives. The measurement is read every tick; if there is none, the controller returns **`None`** — never a zero wrench (see [Conventions](../../framework/conventions.md)). With the current wiring (the measurement is published in the same pass, just before) this only happens before the very first pose.
- **The ε guard is on the denominator** `‖v‖`, never on the result: a `NaN` is unordered and would escape any ordering test.
- **The feedforward rotates the compensation force** `(0, 0, +m·g)`, not the gravity vector `(0, 0, −m·g)`. On a tilted bicopter its `fy` component is unreachable for the mixer and stays in the residual: expected and correct — it is the rotation that restores the thrust direction, not this stage.
- **Notation**: `m` = mass, `M` = moment, consistent with the mixer's NOTATION block.

## Choosing τ

- Sampling floor: `τ ≥ ~10·T` → at 100 Hz, `τ ≥ 0.1 s`.
- Actuator floor: `τ ≥ 3–5 ×` the rotor time constant → the Gazebo model uses 12.5 ms (spin-up) and 25 ms (spin-down) → `τ ≥ ~0.1 s`.

In service: **τ = 0.2 s**, `ζ = 1`. `τ` is a parameter of the component (a constructor argument), not of the vehicle config; a derivable default is `10·T`.

> `VehicleKinematicConfig` stays purely physical. A dedicated cascade config will only be justified when the time-scale-separation constraint links several `τ` values — a shared config is worth creating when a **constraint relates several values**, not when values merely look alike.

## Genericity

Nothing in the law is aerial: `M = I·α` is Newton's law for rotation, valid for any rigid body. What differs between a drone and a rover is **data**: which axes the mixer has moment authority on (the columns of `A`), and how the setpoint is built (a rover constrains only heading; roll and pitch are imposed by the terrain). Its only real assumption is a rigid body with roughly constant inertia — a heavy unfolding manipulator or a half-full tanker would violate it.

## Unit tests

`robomorph/src/lib.rs` freezes two reference cases whose every decimal was verified by independent computation:

| Case | Expected |
|---|---|
| `q = q_d` = identity | `fz = m·g`, moments zero **and finite** (the ε branch) |
| `q` = ~81° roll, written with `w < 0` (double cover) | `fy ≈ −11.631`, `fz ≈ +1.816`, `mx ≈ +0.602` — shortest path, righting sign, inertia applied |

> A test must have been seen **red** at least once — by deliberately flipping a sign — before its green is worth anything. A test that stays green while a known bug is alive teaches you to ignore its colour.
