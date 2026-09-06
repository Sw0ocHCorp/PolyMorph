# Validation

Validating a stage is never a matter of judging that "it looks stable". Because the gains are expressed as `(τ, ζ)`, **the exact response the system must have is known in advance**; validating means overlaying the measurement on the commanded curve and reading the gap. Each rung of the ladder can fail for only one reason, which makes diagnosis local.

## The ladder

| Rung | Content | What it isolates | Status |
|---|---|---|---|
| **A** — unit tests | `q, q_d → e_R → wrench` on reference cases, `#[cfg(test)]` | the mathematics, with no physics | done |
| **B** — open-loop signs | vehicle tilted, motors disabled, wrench logged on 3 axes | frame conventions (direction of `q`, order of `q_err`, sensor reference) | done |
| **C** — static equilibrium | closed loop, identity setpoint | holding equilibrium, weight sharing, data flow | done |
| **D** — step response | setpoint stepped a few degrees, curve `θ(t) = ‖e_R‖(t)` | the dynamics: conformity to the commanded second order | to do |
| **E** — disturbance rejection | external wrench, re-convergence | rejection (already observed incidentally, see below) | to formalise |

## Numeric criteria

- **A** — `q = q_d` → zero **and finite** error; 30° roll → `e_R = (0.5236, 0, 0)`; shortest path: a setpoint at 350° → `e_R = (−0.1745, 0, 0)`; the invariant `‖e_R‖ ≤ π` always. Float comparisons with a tolerance, never `==`.
- **B** — positive roll → negative `mx` (righting), likewise on the other axes; `f_body` leans the right way; `q·(0,0,1)` computed from the sensor quaternion points where the thrust axis physically points; **IMU and odometry agree**.
- **C** — wrench `[0, 0, m·g, 0, 0, 0]`, rotors steady at `m·g/2` each, arms motionless, attitude held; **slow altitude drift is tolerated** (thrust-model surplus, ~0.1 % of weight).
- **D** — see the decay milestones below, and no overshoot at `ζ = 1`.

### What a correct step looks like

For a critically damped loop released from an initial error, `e(t) = (1 + t/τ)·e^(−t/τ)`:

| Elapsed | 1τ | 2τ | 3τ | 4τ | 5.8τ |
|---|---|---|---|---|---|
| Error recovered | 26 % | 59 % | 80 % | 91 % | 98 % |

> The "63 % in one time constant" rule belongs to **first-order** systems. Applying it to this second-order loop understates the settling time by about a factor of five and would make a perfectly healthy response look sluggish. The 2 % settling time is ≈ **5.8 τ** — 1.16 s for τ = 0.2 s.

For a **velocity impulse** `ω₀` (a kick), `e(t) = ω₀·t·e^(−t/τ)` peaks at `t = τ` with `e_max = ω₀·τ/e`. This is the form already observed: an impulse of about 1 rad/s with τ = 0.2 s predicts a 4.2° peak; 4.33° was measured, followed by a monotonic return — rung E passed incidentally.

## The test rig

A test **on the ground with thrust = weight is ill-posed**: the vehicle is weightless while in contact (zero normal force, hence no friction), it slides under the lateral component of thrust, and unmodelled contact torques produce a slow growing oscillation (~2 s period) unrelated to `(τ, ζ)`. Two correct rigs:

- **ball joint at the CoM**: a `ball` joint between `world` and `base_link` in the SDF — three rotational degrees of freedom, no translation, no ground. Ideal for C, D and E, and about ten lines of SDF.
- **in the air**: spawn at altitude, tolerate altitude drift, and expect lateral travel during a step (a roll does what a roll does).

Taking off from the ground is a case of its own: it is a contact transition that a spool-up sequence should smooth — supervisor work, not attitude-loop work.

## Diagnosing a non-conforming step

| Symptom on `θ(t)` | Likely cause | Where to look |
|---|---|---|
| oscillates / overshoots | effective damping < 1: actuator lag too close to `τ`, or under-estimated inertia | rotor time constant; `moments_matrix` |
| slow **and** bouncy | signature of `I_config < I_real` (both gains too small by the same factor) | `moments_matrix` |
| one axis never converges | no moment authority on that axis: the mixer leaves the demand in its residual | the mixer's residual on that axis |
| constant offset | an unmodelled parasitic moment — typically a wrong `com_relative_location` | centre of mass |
| slower than `τ` but clean shape | motors saturating during the transient | clamped commands in the mixer log |
| slower than `τ`, gyro held | stale measurement (late D term) | telemetry rate |

The stage's own gains are the **last** suspects: since the target is known, the gap is a symptom localisable *below* — mixer, actuator, or data flow.

## What to log

To be able to **bisect the chain** (raw inputs → intermediate step → output), everything **on one line per cycle**, emitted from the controller's `exec`:

1. raw sensor `q`; 2. odometry orientation (ground truth); 3. gyro `ω`; 4. `q_d`; 5. `e_R`; 6. published wrench — plus, in closed loop, the motor commands and the mixer's residual. Twenty cycles are enough for a static test. **Predict the values before running, explain every gap afterwards.**

Flow health counters: fresh messages per tick (expected: exactly 1), abstentions (expected: 0 after start-up), null setpoints at the mixer (expected: 0).

## Next

Once D and E pass: wire the odometry, then build the velocity loop — whose first bias to absorb is already measured.
