# Roadmap: the remaining stages

Build order, validated in design: attitude (done) → velocity loop → force→attitude resolver → position loop. Each stage introduces only two or three new symbols; the skeleton (error → desired acceleration → conversion by `m` or `I` at the end of the law) never changes.

## Velocity loop

```
a_d = kp_v·(v_d − v) + ki·∫(v_d − v)dt        (m/s², world frame)
f_d = m·a_d + (0, 0, m·g)                     (N, world frame)
```

| Symbol | What it is | Type | Unit / frame |
|---|---|---|---|
| `v` | measured linear velocity | 3-vector | m/s, world |
| `v_d` | desired linear velocity, from the position loop | 3-vector | m/s, world |
| **`a_d`** | **desired linear acceleration — the law's output, a decision, not a measurement** | 3-vector | m/s², world |
| `f_d` | desired force handed to the resolver | 3-vector | N, world |
| `kp_v = 1/τ_v`, `ki` | proportional and integral gains | scalars | 1/s, 1/s² |
| `m` | vehicle mass (`VehicleKinematicConfig::weight`) | scalar | kg |

`a_d` is the exact linear twin of `α` in the attitude loop — the same skeleton one level up:

| | Rotation | Translation |
|---|---|---|
| Error | `e_R` (rad) | `v_d − v` (m/s) |
| Law output | `α` (rad/s²) | `a_d` (m/s²) |
| Conversion to an effort | `M = I·α` | `f = m·a_d` |

> `a_d` is **not** the accelerometer reading. `Pose.imu_measurement.l_accel` is a measured specific force; `a_d` is a computed decision. They are never compared: there is no acceleration feedback loop anywhere in the cascade. The `+ (0, 0, m·g)` term is the gravity feedforward — the total force is the one that produces the wanted acceleration **plus** the one that cancels the weight.

- Same structure as attitude, one level up, in the **world** frame — where velocity means something, and where Gazebo's odometry provides it. `kp_v = 1/τ_v`, with `τ_v ≈ 5–10 × τ_attitude`.
- **The cascade's only integrator lives here**: this is the stage that sees the constant biases (wind, mis-estimated mass, thrust that does not match its model). Its first job is already quantified: the roughly 0.1 % of weight discrepancy between the `k·ω²` model and Gazebo's real thrust, visible in stabilize as a slow constant-acceleration climb.
- **Anti-windup by conditional integration**: accumulate only when the mixer reports the demand is serviceable. Its residual and its clamping already carry that information inside `compute_command_law`; it is worth surfacing in the return message.
- The gravity feedforward moves here; the attitude stage then publishes moments only.
- Saturate the incoming `v_d` to the vehicle's maximum speed.

## Force → attitude resolver

Promoted to its own page now that it is being built: [The force → attitude resolver](resolver.md). In one line: it consumes exactly as many rotational degrees of freedom as the force constraint requires — none for a holonomic vehicle, one for the OSPRAI, two for a multirotor, the only one a differential rover has — and hands the rest to the external attitude preference.

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
