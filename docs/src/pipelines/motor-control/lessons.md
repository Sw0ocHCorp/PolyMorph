# Lessons learned

Every entry links an observed symptom to the mechanism that explains it and the rule that follows. Recommended reading before modifying the chain.

## Attitude mathematics (rung A)

| Symptom | Cause | Rule |
|---|---|---|
| `fz = −m·g` while level | the feedforward rotated the gravity vector `(0,0,−mg)` instead of the compensation force `(0,0,+mg)` | the wrench is what the motors must **produce** |
| `mx = −3042` for a −81° roll written with `w < 0` | missing `w < 0` negation: `θ = 279°`, the long way round, inverted sign | negate `q_err` when `w < 0`; `‖e_R‖ ≤ π` always |
| `NaN` moments at identity despite an "ε branch" | the guard tested the **result** (already `NaN`) instead of the denominator; `NaN <= ε` is always false | guard the denominator before dividing; a `NaN` is prevented, not repaired |
| `mx = 885` instead of 15 | inertia never applied (factor left at 1.0); then a scalar factor instead of one per axis | `M_k = I_k·α_k`, element-wise; check dimensional homogeneity |
| response far too stiff | `τ = 0.04 s` = 2·T | `τ ≥ 10·T` |
| a green test with a live bug | no assertion on the identity case | a test must have been seen red |

## Sensor conventions (rung B)

| Symptom | Cause | Rule |
|---|---|---|
| a quaternion showing 17° of phantom pitch the accelerometer did not see; the identity while odometry said 0.3 rad | the Gazebo IMU references orientation to its **spawn pose** (default `CUSTOM` with an empty `parent_frame`) | set `<custom_rpy parent_frame="world">`; cross-check IMU / odometry / accelerometer |
| `l_accel ≈ 0` "on the ground" | the vehicle was in free fall | `‖accel‖` is a contact detector |

## Data flow (rung C)

| Symptom | Cause | Rule |
|---|---|---|
| rotors alternating 5.9 N / 0.1 N at loop rate | on ticks without fresh telemetry (250 Hz controller vs 100 Hz telemetry) the attitude stage published `WorkVec::default()` | missing data is never a zero: return `None`, the consumer holds |
| 98.6 % null setpoints at the mixer while the attitude stage computed correctly 32 % of the time | the pipe's `input_state` reset to `None` every pass: producer and consumer landed in different passes | the pipe is only reliable within one pass; make its state persistent |
| 30 empty / 30 double ticks out of 454 | beat between two threads at the same nominal frequency | one chain, one clock; producer registered before consumer |
| 23 ticks of zero thrust after a single abstention | a zero latched by the mixer's hold while the pipe was broken by a pass overrun (a very verbose mixer) | verbose behind a flag; count late passes |
| all chains freezing mid-run | a debugger breakpoint | (not a bug) |

## Physics and solver (rung C)

| Symptom | Cause | Rule |
|---|---|---|
| slow growing oscillation (~2 s) on the ground, ending in a flip | ground test with thrust = weight: weightless in contact, sliding, unmodelled contact torques | ball-joint rig, or fly |
| 1 solver sweep when tilted against 9–10 while level | stopping criterion on total `‖r‖`, dominated by the unreachable `fy` | judge convergence on `‖Aᵀr‖` |
| ±5° arm flick on take-off → ~1 rad/s roll impulse → 0.85 m lateral drift | near-zero columns (tiny thrust at start-up) → step of 10¹¹⁷ → increment saturated at the trust region | **relative** guard on column norms |
| slow constant-acceleration climb | ~0.1 % surplus between `k·ω²` and Gazebo's thrust | the bias for the velocity loop's integrator |
| lateral travel after a roll disturbance | structural behaviour of stabilize mode: attitude holds attitude, not position; only rotor drag brakes | the velocity loop |

## Communications

| Symptom | Cause | Rule |
|---|---|---|
| UDP TX thread dying silently | `while let Ok = blocking_recv()` exits on the first `Lagged` | `Lagged` means "carry on" |

## Method

- **Predict before running, explain every gap afterwards.** Every bug above was found by comparing against a value computed in advance, never by watching behaviour.
- **Bisect**: log raw inputs, the intermediate step and the output on one line.
- **One symbol, one shape**: when a formula references a config field, state its type (scalar, vector, matrix), the exact operation, and flag a misleading name (`moments_matrix` holds moments of *inertia*).

## Flagged in the code, not fixed

Annotating the code surfaced fragilities that have not (yet) shown up at run time. They are marked `// NOTE:` in the sources:

| File | Point |
|---|---|
| `core/scheduler.rs` | pipe reset every pass; an empty chain spins forever; a late process's deadline reset to *now* (phase scrambling); `dt` = nominal period, not elapsed time; non-contiguous side-chain ids create empty spinning chains |
| `messages/registered_message.rs` | `from_frame` panics on an empty frame and never checks the tag byte; `MotorModel` and `RemoteControl` encode but are never decoded |
| `messages/motor_messages.rs` | `WorkingAxis::Unknown` (0) falls into the "joint" family through the mixer's ordering tests |
| `communications/udp_interface.rs` | TX thread dies on `Lagged`; internal-only variants encode to an **empty** frame, which is also the RX thread's poison pill; RX and TX on the same channel echo received frames; `buffer_capacity` unused |
| `control/joystick/*.rs` | `button_pressed` set on a *release*; `stick_pressed` receives a trigger-axis id; `timestamp` never filled; an all-zero snapshot on an event-less tick |
| `control/pid_controller.rs` | the accumulator integrates even with `i = 0`; `dt = 0` divides by zero; the derivative after a `reset` is taken against 0 |
| `control/motion/motors_mixer.rs` | convergence criterion on the total residual norm (replace with `‖Aᵀr‖`) |
| `control/motion/attitude_controller.rs` | the division by `‖v‖` is evaluated before the guard (harmless *only* because the guard tests `‖v‖`, not the result) |
