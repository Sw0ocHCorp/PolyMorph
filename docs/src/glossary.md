# Glossary

## Measurements

| Symbol | Name | Unit | Source in the code |
|---|---|---|---|
| `q` | current attitude, unit quaternion body→world | — | `Pose.orientation` (IMU) |
| `ω` | measured angular rate, body frame | rad/s | `Pose.imu_measurement.a_velocity` |
| `T` | a thruster's current thrust | N | `MotorFeedBack.current_value` (thruster) |
| `θ_j` | a joint's current angle | rad | `MotorFeedBack.current_value` (joint) |
| `dt`, `T` (period) | time step / loop period | s | `Process::get_period` |

## Setpoints

| Symbol | Name | Unit / frame |
|---|---|---|
| `q_d` | desired attitude | — |
| `ω_d` | desired angular rate (implicitly 0 in stabilize) | rad/s, body |
| `v_d` | desired linear velocity (velocity-loop input, position-loop output) | m/s, world |
| `p_d` | desired position | m, world |
| `a_d` | **desired linear acceleration** — output of the velocity law, the linear twin of `α`; a decision, never a measurement | m/s², world |
| `f_d` | desired force, `m·a_d + m·g·ẑ`, before the force→attitude resolver | N, world |
| `yaw_d` | desired heading — the degree of freedom the force does not constrain | rad, world |

## Attitude error

| Symbol | Definition | Unit |
|---|---|---|
| `q⁻¹` | inverse of `q` (world→body); the conjugate for a unit quaternion | — |
| `q_err` | `q⁻¹ ⊗ q_d`, the error in the body frame | — |
| `w`, `v` | scalar (`cos θ/2`) and vector (`e·sin θ/2`) parts of `q_err` | — |
| `θ` | angle of the single rotation, `2·atan2(‖v‖, w)`, in `[0, π]` | rad |
| `e_max` | peak error after a rate impulse `ω₀`: `ω₀·τ/e`, reached at `t = τ` | rad |
| `e` | unit axis of that rotation, body frame | — |
| `e_R` | `θ·e`, the attitude error vector; `e_R[k]` its component on axis k | rad |

## Tunings and control law

| Symbol | Definition | Unit |
|---|---|---|
| `τ` | closed-loop time constant; error envelope decays as `e^(−t/τ)` | s |
| `ζ` | damping ratio (1 = critical) | — |
| `kp = 1/τ²`, `kd = 2ζ/τ` | derived attitude gains, never stored separately | 1/s², 1/s |
| `kp_v = 1/τ_v`, `ki` | velocity-loop proportional and integral gains | 1/s, 1/s² |
| `kp_p = 1/τ_p` | position-loop proportional gain | 1/s |
| `z_d` | desired body z axis, `f_d/‖f_d‖`, expressed in the world | — |
| `q_pref` | attitude **preference** from the external mission setpoint; honoured only in the free DOF | — |
| `S`, `k` | reachable force subspace in the body frame, and its dimension (1 line, 2 plane, 3 all) | — |
| `v` | characteristic vector of `S`: its direction `t` (k = 1) or its normal `n` (k = 2), body | — |
| `𝒜` | authority axes: body axes with a structurally non-zero moment column | — |
| `γ`, `γ*` | angle between `q_pref·v` and `f̂_d`, and its target (0 for k = 1, π/2 for k = 2) | rad |
| `u` | alignment rotation axis, `(q_pref·v) × f̂_d` normalised, world | — |
| `q_align` | `Rot(u, γ − γ*)`; `q_d = q_align ⊗ q_pref` | — |
| `f_h` | `f_d` projected on the plane ⟂ the single authority axis (ground vehicles) | N, world |
| `α`, `α_k` | **desired** angular acceleration (the law's output) | rad/s² |
| `M`, `M_k` | desired moment (torque), `I_k·α_k` | N·m |
| `f_body` | desired force in the body frame, `q⁻¹·(0,0,m·g)` | N |
| wrench | `[fx fy fz mx my mz]`, body frame, moments about the CoM (`WorkVec`) | N, N·m |

## Vehicle

| Symbol | Name | Unit | Field |
|---|---|---|---|
| `m` | mass | kg | `VehicleKinematicConfig.weight` |
| `I`, `I_k` | inertia matrix; inertia about axis k (diagonal) | kg·m² | `moments_matrix` |
| `p_com` | position of the centre of mass in the vehicle frame | m | `com_relative_location` |
| `g` | gravity | m/s² | `GRAVITY` |

## Allocation

| Symbol | Definition |
|---|---|
| `e_j` | motor `j`'s own working axis, unit, in **its own** frame (`WorkingAxis`) |
| `R_j`, `p_j` | motor `j`'s resolved orientation / position in the body frame (`Transform`) |
| `R_parent`, `p_parent` | the same two quantities for `j`'s parent, already resolved |
| `θ_j` | joint `j`'s current angle (0 for a thruster) |
| `fhat`, `ahat` | a thruster's thrust direction / a joint's rotation axis, body frame, unit (`R_j·e_j`) |
| `p_ci` | child `ci`'s position in the body frame |
| `f_ci`, `M_ci` | child `ci`'s current force / moment, read from its work vector |
| `m_ci` | the same moment moved to the pivot: `M_ci − q × f_ci` |
| `w_ci` | child `ci`'s work vector |
| `x_k` | the motor state at cycle `k` — the operating point the columns are linearised about |
| `k_m` | reaction torque per newton along the thrust axis, signed (`moment_constant`) |
| `q` (pivot) | a joint's position, about which its subtree rotates |
| `a_j` | effectiveness column: wrench gained per unit of command (per N or per rad) |
| `w_j` | work vector: wrench produced right now; `T·a_j` for a thruster, the children's sum for a joint |
| `w_current` | the vehicle's current wrench = sum of the root motors' `w_j` |
| `A` | the 6×n matrix of normalised columns |
| `b` | normalised right-hand side, `w_setpoint − w_current` |
| `dx_j` | command increment for motor `j` (N or rad) |
| `r` | residual, `b − A·dx` |
| `score_j = a_j·r`, `step_j = 1/‖a_j‖²` | score and step of the coordinate descent (`Aᵀr` is the vector of scores) |
| trust region | `max_rot_speed·dt`, the per-cycle bound on a joint's increment |

## Naming traps

| Name | What it actually is |
|---|---|
| `moments_matrix` | the matrix of moments of **inertia**, not moment-torques |
| `weight` | a **mass** in kg, not a weight in newtons |
| `WorkVec` | a wrench, not "work" in the energy sense |
| `MotorFeedBack.current_value` | an **effort** (N or rad), never a rotor speed |
