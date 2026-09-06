# Roadmap: the remaining stages

Build order, validated in design: attitude (done) → velocity loop → force→attitude resolver → position loop. Each stage introduces only two or three new symbols; the skeleton (error → desired acceleration → conversion by `m` or `I` at the end of the law) never changes.

## Velocity loop

```
a_d = kp_v·(v_d − v) + ki·∫(v_d − v)dt        (m/s², world frame)
f_d = m·a_d + (0, 0, m·g)                     (N, world frame)
```

- Same structure as attitude, one level up, in the **world** frame — where velocity means something, and where Gazebo's odometry provides it. `kp_v = 1/τ_v`, with `τ_v ≈ 5–10 × τ_attitude`.
- **The cascade's only integrator lives here**: this is the stage that sees the constant biases (wind, mis-estimated mass, thrust that does not match its model). Its first job is already quantified: the roughly 0.1 % of weight discrepancy between the `k·ω²` model and Gazebo's real thrust, visible in stabilize as a slow constant-acceleration climb.
- **Anti-windup by conditional integration**: accumulate only when the mixer reports the demand is serviceable. Its residual and its clamping already carry that information inside `compute_command_law`; it is worth surfacing in the return message.
- The gravity feedforward moves here; the attitude stage then publishes moments only.
- Saturate the incoming `v_d` to the vehicle's maximum speed.

## Force → attitude resolver

The velocity loop outputs a desired force `f_d` in the world frame. Two cases, decided **by data** (the force columns of `A`), never by subclasses:

- **fully actuated in force** (omnidirectional vehicle, rover): `f_d` passes through unchanged (rotated into the body), attitude stays independent — the resolver is the identity;
- **under-actuated** (multirotor, bicopter outside its tilt range): the vehicle must **rotate to align its thrust direction** with `f_d`. For the single-axis case (thrust along body z):

```
thrust = ‖f_d‖
z_d    = f_d / ‖f_d‖                  (desired body z, in the world)
q_d    = built from (z_d, yaw_d)      — yaw is a FREE degree of freedom
```

This is the heart of multirotor flight: translation is commanded through attitude, which is why the attitude loop must be the fast stage. Guard: `f_d → 0` leaves `z_d` undefined (hold the last attitude, or floor the thrust) — the same family of degeneracy as a joint column at zero thrust.

> A subtlety this stage fixes: projecting the feedforward `q⁻¹·(0,0,mg)` onto the reachable set by simply dropping `fy` yields `m·g·cos φ`, **not** `m·g/cos φ` — too little thrust to hold altitude while tilted. Computing thrust from `‖f_d‖` gets it right.

## Position loop

```
v_d = kp_p·(p_d − p)        (world frame; a plain P, kp_p = 1/τ_p)
```

No D (the velocity loop *is* the position loop's D term), no I. Saturate `v_d`.

## What belongs elsewhere

| Concern | Owner |
|---|---|
| Arming, spool-up, take-off sequencing (a hammer take-off from the ground once produced a roll impulse), failsafe, **data age** (a setpoint held for 20 ms is nominal; a pose 10 s old is flying blind) | supervisor |
| Path feasibility — non-holonomy: a wheeled rover cannot park sideways | planner |
| Wheel–ground contact (limited friction, slip): a wheel's effectiveness column must state what force it produces at the contact point | `MotorModel` level |
| Building `q_d` from the sticks (roll/pitch direct, yaw integrated to hold a heading; for a ground vehicle, current roll/pitch plus desired heading) | remote-control stage |

Making data age **observable** is cheap and worth doing now: store the reception instant next to every held value, and count ticks without fresh data. The failsafe policy built on top of it belongs to the supervisor.
