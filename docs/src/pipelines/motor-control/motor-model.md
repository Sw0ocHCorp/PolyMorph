# The motor model

Files: `robomorph/src/control/motion/motor_controller.rs` (efforts) and `motors_mixer.rs` (transform resolution).

## The motor tree

A vehicle is described as a **tree**: *root* motors mounted on the airframe (`parent_id = 0`), and *child* motors mounted on other motors.

<figure>
<svg class="diagram" viewBox="0 0 700 290" xmlns="http://www.w3.org/2000/svg" role="img" aria-label="The OSPRAI motor tree: the airframe carries two tilting arms, each carrying one rotor">
  <rect x="290" y="24" width="120" height="42" rx="6" fill="currentColor" fill-opacity="0.06" stroke="currentColor" stroke-opacity="0.45"/>
  <text x="350" y="50" text-anchor="middle" font-size="12" font-weight="600">base_link</text>
  <text x="350" y="14" text-anchor="middle" font-size="10" opacity="0.7">airframe — body frame</text>
  <line x1="330" y1="66" x2="228" y2="112" stroke="currentColor" stroke-opacity="0.55" stroke-width="1.3"/>
  <line x1="370" y1="66" x2="472" y2="112" stroke="currentColor" stroke-opacity="0.55" stroke-width="1.3"/>
  <rect x="140" y="116" width="140" height="48" rx="6" fill="#8b7cd8" fill-opacity="0.12" stroke="#8b7cd8" stroke-width="1.3"/>
  <text x="210" y="138" text-anchor="middle" font-size="12" font-weight="600">arm_left</text>
  <text x="210" y="155" text-anchor="middle" font-size="10" opacity="0.8">joint, rotates about Y</text>
  <rect x="420" y="116" width="140" height="48" rx="6" fill="#8b7cd8" fill-opacity="0.12" stroke="#8b7cd8" stroke-width="1.3"/>
  <text x="490" y="138" text-anchor="middle" font-size="12" font-weight="600">arm_right</text>
  <text x="490" y="155" text-anchor="middle" font-size="10" opacity="0.8">joint, rotates about Y</text>
  <line x1="210" y1="164" x2="210" y2="206" stroke="currentColor" stroke-opacity="0.55" stroke-width="1.3"/>
  <line x1="490" y1="164" x2="490" y2="206" stroke="currentColor" stroke-opacity="0.55" stroke-width="1.3"/>
  <rect x="140" y="206" width="140" height="48" rx="6" fill="#2aa198" fill-opacity="0.12" stroke="#2aa198" stroke-width="1.3"/>
  <text x="210" y="228" text-anchor="middle" font-size="12" font-weight="600">rotor_left</text>
  <text x="210" y="245" text-anchor="middle" font-size="10" opacity="0.8">thruster, pushes along Z</text>
  <rect x="420" y="206" width="140" height="48" rx="6" fill="#2aa198" fill-opacity="0.12" stroke="#2aa198" stroke-width="1.3"/>
  <text x="490" y="228" text-anchor="middle" font-size="12" font-weight="600">rotor_right</text>
  <text x="490" y="245" text-anchor="middle" font-size="10" opacity="0.8">thruster, pushes along Z</text>
  <text x="612" y="142" font-size="10" fill="#8b7cd8">reorients</text>
  <text x="612" y="156" font-size="10" fill="#8b7cd8">its subtree</text>
  <text x="612" y="232" font-size="10" fill="#2aa198">produces</text>
  <text x="612" y="246" font-size="10" fill="#2aa198">force</text>
  <text x="350" y="280" text-anchor="middle" font-size="10.5" opacity="0.7">a steered rover would be: steering column (joint) → left wheel, right wheel (thrusters)</text>
</svg>
<figcaption>The OSPRAI tree. Only the data changes from one vehicle to another; the traversal does not.</figcaption>
</figure>

## Step 1 — resolve every motor's pose in the body frame

Each motor is described **at rest, in its parent's frame** (`relative_location`, `relative_orientation`). Its absolute body-frame pose is built root → leaves:

```
orientation :  R_j = R_parent · R_j^rel · Rot(e_j, θ_j)
position    :  p_j = p_parent + R_parent · p_j^rel
root        :  R_j = R_j^rel · Rot(e_j, θ_j)        p_j = p_j^rel + p_com
```

Two things that are easy to get wrong:

- a joint's angle `θ_j` is composed **on the right** of `R_j^rel`, because it is a rotation about the joint's *own* axis, expressed in its own frame;
- `p_j^rel` is expressed in the **parent's** frame: it is rotated by `R_parent`, never by `R_j`.

`Rot(e_j, θ_j)` leaves `e_j` unchanged, so a joint's axis seen from the body does not depend on its own angle. The `p_com` offset on root motors is what makes every lever arm measured from the centre of mass.

## Step 2 — effectiveness column and work vector

This is the central concept of the mixer, and the source of most questions. `compute_motor_efforts` returns **two** objects per motor, of different natures:

| | Effectiveness column `a_j` | Work vector `w_j` |
|---|---|---|
| Question answered | *"if I add **one unit** of command on this motor, what wrench does the vehicle gain?"* | *"what wrench does this motor produce **right now**?"* |
| Nature | a **slope** (sensitivity, derivative) | a **value** |
| Unit | wrench per N (thruster) or per rad (joint) | N and N·m |
| Used for | a column of the solver's matrix `A` | summed over the roots → the vehicle's current wrench |

<figure>
<svg class="diagram" viewBox="0 0 700 260" xmlns="http://www.w3.org/2000/svg" role="img" aria-label="Graph of the wrench produced by a thruster against its thrust: a straight line through the origin whose slope is the effectiveness column">
  <defs>
    <marker id="m-arw" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">
      <path d="M1 1 L9 5 L1 9" fill="none" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
    </marker>
  </defs>
  <line x1="100" y1="210" x2="630" y2="210" stroke="currentColor" stroke-opacity="0.5" stroke-width="1.1" marker-end="url(#m-arw)"/>
  <line x1="100" y1="210" x2="100" y2="40" stroke="currentColor" stroke-opacity="0.5" stroke-width="1.1" marker-end="url(#m-arw)"/>
  <text x="632" y="228" text-anchor="end" font-size="11" opacity="0.85">thrust T  (N)</text>
  <text x="104" y="36" font-size="11" opacity="0.85">wrench produced ‖w‖</text>
  <line x1="100" y1="210" x2="560" y2="62" stroke="#2aa198" stroke-width="2.4"/>
  <text x="470" y="72" font-size="11.5" fill="#2aa198" font-weight="600">w = T · a</text>
  <line x1="300" y1="145" x2="400" y2="145" stroke="currentColor" stroke-opacity="0.6" stroke-width="1.2" stroke-dasharray="4 3"/>
  <line x1="400" y1="145" x2="400" y2="113" stroke="currentColor" stroke-opacity="0.6" stroke-width="1.2" stroke-dasharray="4 3"/>
  <text x="350" y="161" text-anchor="middle" font-size="10.5" opacity="0.85">ΔT</text>
  <text x="408" y="132" font-size="10.5" opacity="0.85">‖a‖ · ΔT</text>
  <circle cx="100" cy="210" r="6" fill="none" stroke="#d4694a" stroke-width="2"/>
  <text x="116" y="196" font-size="11" fill="#d4694a" font-weight="600">rotor stopped</text>
  <text x="116" y="212" font-size="10.5" fill="#d4694a">w = 0, but the slope a is unchanged</text>
</svg>
<figcaption>A lever keeps its lever arm when nobody pushes on it. That is why a thruster can start from zero.</figcaption>
</figure>

### The thruster

Notation: `fhat = R_j · e` is the thrust direction in the body frame (unit), `p` the motor's position from the CoM, `k_m` the reaction torque per newton (signed by the propeller's spin direction), and **`T` the thrust the rotor produces right now, in newtons** — `feedback.current_value` for a thruster, reconstructed by the vehicle bridge from the measured rotor speed (`T = k·ω²`).

```
column       a = [ fhat ; p × fhat + k_m · fhat ]      pure geometry
work vector  w = T · a                                  linear in T
```

- `p × fhat` is the lever arm of the thrust about the CoM;
- `k_m · fhat` is the propeller's reaction torque on the airframe, carried along the thrust axis. Two counter-rotating rotors must have **opposite** `k_m`, otherwise the model sees a yaw torque the vehicle does not produce.

The essential point: **the column exists even with the rotor stopped.** `T = 0` gives `w = 0 · a = 0`, while `a` is intact. A non-zero slope at a point of zero value is unremarkable — `f(T) = T·a` is zero at `T = 0` and still has slope `a` there. Because `w` is strictly linear in `T`, the column is **exact for any increment**, which is why a thruster's decision variable may jump straight to its final value.

### The angular joint

A joint **produces no wrench of its own**: it reorients what its children produce. Notation: `ahat = R_j · e` the rotation axis in body, `q = p_j` the pivot, and for each child `ci`: `p_ci` its position, `f_ci` its current force (read from its work vector), `m_ci = k_m,ci · f_ci` its intrinsic torque.

```
work vector  w = Σ_ci w_ci                              (the joint adds nothing)
column       a = dw/dθ                                  (derivative of the subtree wrench)

  force part  :  dF/dθ = ahat × f_ci

  moment part, child is a thruster:
      dM/dθ = (ahat × (p_ci − q)) × f_ci      the lever arm changes: the child swings along an arc
            + p_ci × (ahat × f_ci)            the force direction turns, at the same lever arm
            + ahat × m_ci                     the intrinsic torque turns too

  moment part, child is another joint: its wrench already aggregates its own subtree, so
      m_ci  = M_ci − q × f_ci                 the subtree moment moved to the pivot
      dM/dθ = ahat × m_ci + q × (ahat × f_ci)
```

Both writings are the same formula (check it with the identity `ahat × (u × f) = (ahat × u) × f + u × (ahat × f)`).

**Every term is linear in `f_ci`.** That is the algebraic reason for the behaviour that surprises everyone on the first run:

> **Why the arms have a zero effort vector at the first cycle while the rotors do not.**
> A rotor has an *intrinsic slope*: its column is pure geometry, available before it even spins. An arm only has the slope its child lends it: both its column **and** its work vector equal `(child thrust) × (geometry)`. Rotors stopped ⇒ `f_ci = 0` ⇒ zero column, zero work vector ⇒ the arm has **no authority at all** and sits out. On the next cycle the feedback reports real thrust, the arms' columns become non-zero and they can take part — but they only move if the residual contains something they can serve (yaw, pitch); while level, their increments stay ≈ 0.

A joint's column is a **tangent**: it only describes reality for a small angle. Keeping the increment inside that domain is the solver's job (trust region — see [the mixer](mixer.md)).

### Design consequences

- **`min_value > 0` on rotors** (0.1 N on the OSPRAI): since a joint borrows its authority from its child's thrust, letting a rotor stop in flight would silently remove the arm from the allocation.
- **The mixer works in efforts, never in rotor speeds.** `current_value` is already a thrust; applying `k·ωⁿ` a second time would square a force. The effort law and its inverse live at the hardware boundary.
- **A wrong feedback is a wrong plant model**: the current work vector is rebuilt from the feedbacks, and it is the solver's linearisation point.

## Traversal order

Step 1 walks the tree **root → leaves** (a child's pose needs its parent's). Step 2 walks **leaves → root** (a joint's column needs its children's work vectors). The mixer sorts motors by depth so both orders hold whatever the `HashMap` iteration order is.

## Utilities

`working_axis_to_vec3` / `_i32_` give a motor's own unit axis (`e_j`) from its `WorkingAxis`. `quaternion_to_euler` is for display only — never used by a control law.
