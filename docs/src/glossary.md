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

| Symbol | Name | Unit |
|---|---|---|
| `q_d` | desired attitude | — |
| `ω_d` | desired angular rate (implicitly 0 in stabilize) | rad/s |
| `v_d`, `p_d`, `f_d`, `a_d`, `yaw_d` | desired velocity, position, force, acceleration, heading (future stages) | m/s, m, N, m/s², rad |

## Attitude error

| Symbol | Definition | Unit |
|---|---|---|
| `q⁻¹` | inverse of `q` (world→body); the conjugate for a unit quaternion | — |
| `q_err` | `q⁻¹ ⊗ q_d`, the error in the body frame | — |
| `w`, `v` | scalar (`cos θ/2`) and vector (`e·sin θ/2`) parts of `q_err` | — |
| `θ` | angle of the single rotation, `2·atan2(‖v‖, w)`, in `[0, π]` | rad |
| `e` | unit axis of that rotation, body frame | — |
| `e_R` | `θ·e`, the attitude error vector; `e_R[k]` its component on axis k | rad |

## Tunings and control law

| Symbol | Definition | Unit |
|---|---|---|
| `τ` | closed-loop time constant; error envelope decays as `e^(−t/τ)` | s |
| `ζ` | damping ratio (1 = critical) | — |
| `kp = 1/τ²`, `kd = 2ζ/τ` | derived gains, never stored separately | 1/s², 1/s |
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
| `fhat`, `ahat` | a thruster's thrust direction / a joint's rotation axis, body frame, unit (`R_j·e`) |
| `k_m` | reaction torque per newton along the thrust axis, signed (`moment_constant`) |
| `p_j`, `R_j` | motor `j`'s resolved position / orientation in the body frame (`Transform`) |
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
