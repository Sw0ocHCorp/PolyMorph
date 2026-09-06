# The mixer: control allocation

File: `robomorph/src/control/motion/motors_mixer.rs`. The mathematics is in [Constrained least squares](../../math/least-squares.md); this page is the implementation and what validation taught us about it.

## The problem

The vehicle must produce **one** 6D wrench (`w_setpoint`, asked by the stage above) and it has `n` motors. Find the `n` commands that produce it.

## 1. Incremental formulation

The solver does not look for absolute commands but for the **increment** to apply on top of the measured motor state:

```
A · dx  ≈  b        with    b = w_setpoint − w_current
```

`w_current` is the sum of the **root** motors' work vectors, rebuilt from the feedbacks. Two reasons:

- a joint's column is a tangent, only valid near the current configuration — the problem has to be re-linearised every cycle anyway;
- the measured state *is* the linearisation point: a wrong feedback is a wrong plant model.

Consequence upstream: the setpoint received is the **absolute** desired wrench, never a delta — the mixer does the subtraction itself. Publishing a delta would subtract it twice.

Consequence for understanding start-up: **the same setpoint does not mean zero increments.** The setpoint is constant (`[0, 0, m·g, 0, 0, 0]` to hold level flight) but `w_current` changes every cycle with the feedbacks. At cycle 1, `w_current = 0` and the residual is the *whole* setpoint; it then shrinks until the current wrench equals the setpoint. Holding a constant pose in flight costs a constant, non-zero force: hovering is a permanent effort, not the absence of effort.

## 2. What a column means

See [the motor model](motor-model.md). Thruster: exact column, decision variable in N. Joint: tangent column, proportional to its children's thrust, decision variable in rad. At zero thrust a joint's column is zero and the allocation loses rank.

## 3. Row normalisation

Force rows divided by `m`, moment row `k` by `√(m·I_k)`: the residual is expressed as an acceleration, which makes the force/moment arbitration physical rather than an accident of units. Applied to the columns **and** to the right-hand side `b`.

## 4. The solver: projected coordinate descent

```
score_j = a_j · r                   what this motor can serve of what is missing (signed)
step_j  = 1 / ‖a_j‖²                exact step along column j
dx_j   += score_j · step_j
```

The step is **per column**, and the residual is refreshed **after every motor** (Gauss–Seidel, not Jacobi). Rationale in the [mathematics chapter](../../math/least-squares.md).

> **Step ≠ increment.** The step is only a geometric scale factor; the actual increment is `score × step`, and the score follows the residual. A finite step with a zero residual gives a zero increment.

### Zero and near-zero columns

An exactly zero column gives an infinite step; the guard sets the step to 0 and the motor sits out. But a **near**-zero column — an arm whose child rotor has just started — gives an **astronomically large** step (10¹¹⁷ observed): any 10⁻¹⁷ residual noise then saturates the increment at the trust region. That produced a ±5° "flick" of the arms at every take-off, a roll impulse of about 1 rad/s and a lateral drift. The guard is therefore **relative**: a column whose norm is below `10⁻⁶ ×` the largest column sits out, exactly like a zero one.

## 5. The bounds are part of the problem

After each step the increment is projected onto the feasible box (bounds apply to the *result* `current + dx`, hence the shift by the current value):

- **both families**: `min_value` / `max_value` of the motor's own effort;
- **joints only**: intersected with the **trust region** `max_rot_speed · dt` — what the joint can physically travel in one cycle, and above all the domain where the tangent still describes reality. A thruster needs none: its column is exact.

## 6. What the solver cannot do

`n` motors serve at most `n` wrench components. Any demand outside the column space is **unreachable** and stays in the residual forever — on a bicopter, the lateral force `fy`: no column has an `fy` entry. Yet the gravity feedforward rotated into the body frame has `fy = m·g·sin φ` as soon as the vehicle rolls. The residual norm is therefore **not** a convergence indicator: it converges to the unreachable part, not to zero.

### The stopping criterion

The current criterion stops the sweeps when the **relative** improvement of `‖r‖` falls below 10 %. It has a measured weakness: as soon as the vehicle tilts, the unreachable part dominates the norm and never moves, so the first sweep's relative improvement looks negligible and the solver exits **after a single sweep** (against 9–10 while level), leaving *reachable* moments under-served.

> The clean fix: the least-squares optimality condition is **`Aᵀ·r = 0`**, not `r = 0`, and the unreachable part lies in the null space of `Aᵀ`. Judging convergence on `‖Aᵀr‖` — the `motor_scores` vector, already computed at every sub-step — removes the unreachable part from the criterion automatically.

## The full cycle (`compute_command_law`)

1. Sort motors by depth in the tree.
2. **Step 1**: resolve the `Transform`s, root → leaves.
3. **Step 2**: leaves → root, compute `(column, work vector)` for each motor; normalise the column; sum the root work vectors into `w_current`.
4. **Step 3**: compute the steps (with the guards), the normalised right-hand side `b = w_setpoint − w_current`, then up to 10 sweeps of projected coordinate descent.
5. Emit `current_value + dx`, clamped, as a `MotorCommand`: `THRUST` (N) for a thruster, `ANGULARPOSITION` (rad) for a joint.

### Worked example: the first cycles after start-up

| Cycle | Rotors (exact column) | Arms (column ∝ rotor thrust) |
|---|---|---|
| 1 | finite step, residual = the whole weight → commanded to ~5.4 N | zero column → step 0 → motionless |
| 2 | the feedback reports real thrust; the residual shrinks | columns become non-zero → finite steps; increments ≈ 0, since no moment is demanded |
| steady state | 5.886 N each (the weight shared) — zero increments | motionless, but **available** for yaw and pitch |

## Inputs, outputs, state

- **Inputs**: `MotorController`s (configuration channel, once), `MotorFeedBack`s (feedback channel, every tick, published by the Gazebo `joint_state` callback), the wrench setpoint (pipe from the attitude stage).
- **State**: the motor table and **the last setpoint** — held when nothing new arrives.
- **Output**: `AnyMessage::MotorCommands` on the command channel.

## OSPRAI-specific validation oracle

An exact closed-form geometric inversion exists for this airframe: take the in-plane components of each rotor's force as the unknowns, then convert to polar form (thrust, angle). It is a useful cross-check for the numerical solver, but it does not generalise to arbitrary kinematic trees — which is the whole point of the generic allocator.
