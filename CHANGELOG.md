# Changelog

All notable changes to PolyMorph, one entry per commit, newest first.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), adapted to a project without releases:

- An entry is a **commit** (`hash — date — title`), not a version.
- Entries are grouped into **phases** (`##`) so the evolution of the architecture can be read top-down.
- Each entry lists what was **Added / Changed / Fixed / Removed**. When a commit ships something known broken or not validated, a **Known issues** line says so.
- Commits of the `Total_Rework` history come first. Commits living only on branches that were never merged into it are listed in [Unmerged branches](#unmerged-branches), at the end.
- Merge and auto-stash commits are listed for completeness, with one line saying whether they carry content of their own.

Entries up to `130df2d` were written on 2026-09-30, the history reconstructed from the diffs of each commit rather than from the titles alone. From then on, every commit adds its own entry.

## Timeline

| Period | Phase | Where the code lived |
|---|---|---|
| 2026-06 → 2026-09 | [Gazebo Harmonic, scheduler, control cascade](#2026-06--2026-09--gazebo-harmonic-scheduler-and-control-cascade) | `robomorph/`, `gazebo_simulation/`, `docs/` |
| 2026-02 → 2026-03 | [Hardware design (KiCad)](#2026-02--2026-03--hardware-design) | `Hardware/` |
| 2026-01 → 2026-02 | [State estimation: Mahony, UKF](#2026-01--2026-02--state-estimation-mahony-then-ukf) | `robomorph/src/filtering/`, `companion/` |
| 2025-12 | [Lidar perception](#2025-12--lidar-perception) | `robomorph/src/lidar_management/`, `godot/polymorph/` |
| 2025-12 | [Framework rework: workers, UDP, serialization](#2025-12--framework-rework-workers-udp-serialization) | `robomorph/` (rewritten) |
| 2025-11 → 2025-12 | [Bootstrap: Godot, companion, first robomorph](#2025-11--2025-12--bootstrap-godot-companion-first-robomorph) | `godot/`, `companion/`, `controller/`, `robomorph/` |
| 2026-02 → 2026-05 | [Unmerged: `MotionControl`, `embedded`](#unmerged-branches) | `godot/`, `polymorph_embedded/`, `firmwares/` |

> **The runtime paradigm changed radically on 2026-06-26 (`ba22f38`).** The stack went from an event-driven **observer pattern** (synchronous callbacks and asynchronous buffers) to a **scheduler running processes pipelines**. See [Paradigm change](#paradigm-change-observers--scheduler-and-processes-pipelines).

## [Unreleased]

---

## 2026-06 → 2026-09 — Gazebo Harmonic, scheduler and control cascade

### Paradigm change: observers → scheduler and processes pipelines

Introduced by `ba22f38` (2026-06-26) and completed by `2f81cd3` (2026-09-06). It is the most structural change in the history: it decides how every module runs and how data moves between modules.

**Before** (`6603e06` → `01c21b2`): an event-driven **observer pattern**.
- `core/event_management.rs` provided an `Observer` of two kinds:
  - **synchronous**: a callback `Arc<Mutex<dyn FnMut>>` that `Event::trig` runs *in the emitter's context*;
  - **asynchronous**: a buffer the consumer polls.
- `core/worker.rs` wrapped each `Module` in a `Worker` (a frequency, inline or on a dedicated thread). Workers were chained through observers (`set_next_worker`) and started by a `WorkerFactory`.
- **Who ran when** was decided by whoever emitted an event.
- Since observers had to call into workers, a `Worker` had to live in an `Arc` with every mutable field behind a `Mutex` (as `worker.rs` itself explains). The result was shared ownership and locks everywhere.

**After**: a **scheduler** running **processes pipelines**.
- `core/scheduler.rs` defines `Process::exec(input, dt) -> output`.
- A **processes pipeline** (`ProcessesChain` in the code at the time) runs its processes in registration order, on one clock, each at its own period. It hands each output to the next process: this is the **pipe**.
- The `Scheduler` owns a **main pipeline** (run on the caller's thread) and **side pipelines** (each on a dedicated thread).
- Everything that does not travel through the pipe flows through non-blocking tokio channels (broadcast, plus mpsc until `2f81cd3`).
- `Arc<Mutex>` only survives for the state written by the Gazebo transport callbacks.

**Why it matters:**
- Execution order and timing are explicit and decided in one place.
- The data flow of the robot can be read from the registration order in `main.rs`.
- Modules no longer share locks.
- The flip side: the pipe only holds *within one pass* of a pipeline. Hence the rule "one processes pipeline, one clock; producer registered before consumer" (see `docs`, framework → scheduler).

### `130df2d` — 2026-09-30 — WIP: generic PID, velocity-loop and resolver drafts (chain does not fly)

Work-in-progress baseline, committed as-is so that the next bricks start from clean diffs. It builds (0 errors) and the unit tests pass.

#### Added
- The `PIDValue` trait. `PIDController<T: PIDValue>` is now generic, and `Vec3` implements `PIDValue` component by component (integrator clamped per axis).
- `AnyMessage::ForceVec(Vec3)`. `UnitQuat * Vec3` rotates a vector.
- `SpeedController` draft: a velocity loop on `PIDController<Vec3>`.
- `ForceToAttitudeResolver` draft: not a `Process`, not wired.
- The Gazebo odometry callback fills `Pose.l_velocity`. The mission setpoint asks for `l_velocity = (0, 0, 1)`.

#### Changed
- `MotorController::new(model, feedback)`: a motor no longer owns a PID.
- `encode_frame` gets a wildcard arm: internal-only variants encode to an empty frame.
- `main.rs`: `SpeedController` registered between the remote and the attitude stage. The attitude setpoint subscription is commented out.

#### Removed
- The PID dead band is no longer applied (still stored).
- Dead commented-out code in `osprai_controller.rs`.

#### Known issues — the chain does not fly
- The odometry passes quaternion components as Euler angles, so `l_velocity` is wrong.
- `SpeedController` has three errors in its gravity handling and one bad gain:
  - it rotates gravity by `q`;
  - it subtracts an acceleration from a velocity;
  - it adds gravity instead of compensating it;
  - `ki = 1e4`.
- `ForceVec` lands in the pipe where the attitude stage ignores it, and the attitude stage has no setpoint. It abstains every tick, and the mixer holds its initial zero wrench.
- `PIDController::compute_output` prints P/I/D on every call.

### `2e02e7b` — 2026-09-30 — Docs: force->attitude resolver spec, velocity-loop symbols

#### Added
- `pipelines/motor-control/resolver.md`, the specification of the force → attitude resolver:
  - the degree-of-freedom budget;
  - one alignment formula for `k = 1` and `k = 2`;
  - authority axes and the forward / reverse choice;
  - degeneracies;
  - how `S` and `𝒜` are derived from the motor tree;
  - validation cases with predicted values.

#### Changed
- `roadmap.md`: the velocity loop gets a symbol table (`v`, `v_d`, `a_d`, `f_d`, `kp_v`, `ki`, `m`) and a table pairing each rotation quantity with its translation twin. `a_d` is a decision, never the accelerometer reading. The resolver section now points to its own page.
- `glossary.md`: setpoint symbols with their frames, resolver symbols, motor-tree symbols.
- `overview.md` and `SUMMARY.md`: status table (velocity loop written but not validated, resolver specified); resolver page in the reading order and the sidebar.

### `2f81cd3` — 2026-09-06 — Attitude controller + Doc OK

#### Added
- `AttitudeController` (`robomorph/src/control/motion/attitude_controller.rs`), stage v1 of the cascade ("stabilize"). A geometric PD law:
  - quaternion error `q_err = q⁻¹ ⊗ q_d`, negated when `w < 0` (shortest path);
  - log map to the rotation vector `e_R`, with a small-angle guard;
  - `α = e_R/τ² + (2ζ/τ)(ω_d − ω)`, then `M = I·α`;
  - gravity feedforward `f_body = q⁻¹·(0, 0, m·g)`.

  Defaults: `τ = 0.2 s`, `ζ = 1`. No integrator. The setpoint is held between updates. Output: `AnyMessage::VehicleWrench`.
- `XboxPadControl`, a gamepad reader on `gilrs` 0.11, implementing `JoyStickController`. Emits a `RemoteControl` message (buttons, sticks). Also added `MessageType::RemoteControlMessage = 6` and the variants `AnyMessage::VehicleWrench` and `AnyMessage::RemoteControl`.
- `prost_types::Timestamp` fields on `Pose`, `IMUMeasurements`, `GNSSMeasurement`, `LidarMeasurements`, `MotorFeedBack` and `MotorCommand` (protobuf tags shifted).
- Regression tests in `robomorph/src/lib.rs`: the identity case (`fz = 11.772 N`) and an 81° roll case written with `w < 0`.
- mdBook documentation in `docs/`: `book.toml` and `theme/custom.css`, linked from the root `README.md`. Pages:
  - `introduction`;
  - Framework: `overview`, `scheduler`, `messages`, `communications`, `conventions`;
  - Mathematical foundations: `frames-and-rotations`, `rigid-body`, `cascaded-control`, `least-squares`;
  - Pipeline "motor control": `overview`, `motor-model`, `mixer`, `attitude-controller`, `roadmap`, `validation`, `lessons`;
  - Simulation: `gazebo`;
  - `glossary`, `contributing`.

#### Changed
- `MotionController::compute_command_law` takes and returns `Option<AnyMessage>` and a `verbose` flag. `set_setpoint_receiver(Receiver<Pose>)` was added; `send_motor_command` and `add_or_update_motor` were removed from the trait.
- `VehicleKinematicConfig.moments_matrix` is now an nalgebra `Matrix3` (the inertia matrix). The `error_*_factor` fields are marked legacy.
- `MotorsMixer`:
  - the wrench-setpoint law moved out into `AttitudeController`; the mixer now takes the `VehicleWrench` from the pipe and holds the last one;
  - motor configurations and feedbacks now arrive on broadcast receivers (`set_motor_config_receiver`, `set_motor_feedback_receiver`).
- Scheduler and `Process` (completes the [paradigm change](#paradigm-change-observers--scheduler-and-processes-pipelines)):
  - `exec(&Option<AnyMessage>, dt)`;
  - `set_receiver` / `set_sender`, broadcast channels in both directions;
  - each process owns its period (`set_period_from_freq`, `get_period`, `get_name`), and the frequency argument disappears from `register_*`;
  - `run_once` rewritten as a per-process deadline loop.
- `HardwareInterface` loses `set_outbound_rx` and `connect_process`. `UdpInterface` implements `Process` (`exec` is `todo!()`).
- Gazebo `main.rs` wires the cascade with broadcast channels:
  - one scheduler chain at 100 Hz, in the order `osprai → remote → attitude → mixer`;
  - `OspraiController` sends the motor configs, the feedbacks and a hover setpoint (identity attitude), maps the mixer's `MotorCommands` onto one `Actuators` message, and publishes it only when complete;
  - `VehicleController` loses `apply_actuator_setpoints` and `connect_interface`.
- SDF: IMU `orientation_reference_frame` set to `CUSTOM` with `parent_frame="world"` (by default Gazebo references it to the spawn pose). Spawn height 0.06 → 0.1 m.
- `.gitignore` ignores `*.txt` and `.claude/`.

#### Fixed
- Mixer: relative guard on near-zero effectiveness columns (step forced to 0 when `‖a_j‖ < 1e-6·max‖a‖`). A tiny column used to give a step of about 1e117 and flicked the arms at take-off.

#### Removed
- `Motion_controller_Logs.txt`. Also the debug GUI's `requirements.txt`, a side effect of the new `*.txt` ignore rule.

#### Validation
- `validation.md` marks rungs A (unit), B (open-loop signs) and C (static equilibrium) done. D (step) is to do and E (disturbance) is to formalise.

### `b3f837f` — 2026-08-09 — MotorsMixer: generic control allocation, validated on OsprAi

#### Added
- `robomorph::control`:
  - `motion::{motion_controller, motor_controller, motors_mixer}`;
  - `pid_controller::PIDController` (P/I/D, clamped accumulator);
  - an empty `JoyStickController` trait stub.
- `MotionController: Process` trait (`compute_command_law`, `send_motor_command`, `add_or_update_motor`), `VehicleKinematicConfig` (mass, 3×3 inertia, CoM offset) and `GRAVITY`.
- `MotorController`, one node of the motor tree (`MotorModel` + `MotorFeedBack` + PID). `compute_motor_efforts`:
  - **thruster** column `[f̂ ; p × f̂ + k_m·f̂]`, scaled by the current thrust `T`;
  - **joint** column = derivative of its child subtree's wrench with respect to the joint angle, using moments moved to the pivot (`m_ci = M_ci − q × f_ci`).
- `MotorsMixer`, generic control allocation over an arbitrary motor tree (`parent_id` / `child_ids`), with a "THEORY OF THE MIXER" block in the source:
  - **STEP 1**, root to leaves: transforms `R_j = R_parent·R_rel·Rot(e, θ)` and `p_j = p_parent + R_parent·p_rel`, root motors offset by the CoM;
  - **STEP 2**, leaves to root: the 6×n effectiveness matrix `A`, and the current wrench as the sum of the root motors;
  - **row normalisation**: force rows divided by `m`, moment rows by `√(m·I_k)`, on both `A` and `b`;
  - **STEP 3**: incremental solve `A·dx ≈ w_setpoint − w_current` by projected coordinate descent (Gauss-Seidel):
    - per-column step `1/‖a_j‖²`;
    - residual refreshed after every motor;
    - at most 10 sweeps;
    - increments clipped to `[min_value, max_value]`, plus a trust region `max_rot_speed·dt` for joints.
  - Output: `AnyMessage::MotorCommands` (`THRUST` for rotors, `ANGULARPOSITION` for joints).
- A first attitude law inside the mixer (`compute_wrench_setpoint`). It was moved out in `2f81cd3`.
- Messages:
  - `WorkVec` (6D wrench with operators), `WorkingAxis`, `MotorModel`, `PIDConfig`, `Transform`;
  - `Vec3` / `UnitQuat` wrappers around nalgebra with hand-written prost codecs;
  - `AnyMessage::MotorCommands`.
  - `Pose` and `IMUMeasurements` now carry `Vec3` / `UnitQuat`, and `MessageType` was renumbered.

#### Changed
- `Process` gains `set_name`. `ProcessesChain::run_once` switches to deadline-based timing. `num-quaternion` is replaced by `nalgebra` 0.35.
- `OspraiController`:
  - builds the `MotorModel`s from the Gazebo scene and joint states;
  - rotors get a minimum thrust of 0.1 N so the tilt joints keep authority;
  - converts thrust to rotor speed with `ω = (T / (transmission·k))^(1/n)`;
  - publishes a single `Actuators` message on `/osprai/command/motor_setpoints` and drops any non-finite command;
  - subscribes to `/model/osprai/odometry`.
- `main.rs`: OsprAi config `m = 1.2 kg`, diagonal inertia `(0.017, 0.0239, 0.0357) kg·m²`.
- SDF:
  - arm and rotor rest poses set to 0 (tilt convention: −π/2 = thrust up, 0 = forward);
  - tilt limits ±1.57 rad, velocity limit 10;
  - rotors and tilt controllers share `/osprai/command/motor_setpoints`;
  - tilt gains p 20, d 0.13, output ±0.8 N·m;
  - `rotorVelocitySlowdownSim` 10 → 1;
  - `world_anchor` joint removed.

#### Validation
- From the commit message: moment error < 0.3 %, hover attitude error < 1e-5 rad, −5° roll step reaches 96 % of the setpoint.

### `3ac34dd` — 2026-06-26 — Auto stash before merge of "Total_Rework" and "main"
- IDE auto-stash commit. `.gitignore` only (log folders reordered). No source change.

### `5c1431a` — 2026-06-26 — Merge branch 'main' into Total_Rework
- Merge commit with no content of its own: the resulting tree is exactly `ba22f38`'s.

### `ba22f38` — 2026-06-26 — New Publisher / Observer architecture removing almost all `Arc<Mutex>`

#### Changed (architecture)
- **Paradigm change.** The observer / worker framework (`Observer`, `Event`, `Worker`, `WorkerFactory`) is replaced by a scheduler running processes pipelines. See [Paradigm change](#paradigm-change-observers--scheduler-and-processes-pipelines). Despite the commit title, the new design is not an observer pattern: processes do not call each other; the scheduler calls them in order.

#### Added
- `robomorph::core::scheduler`:
  - the `Process` trait (`exec(Option<AnyMessage>, dt)`, a broadcast inbound receiver, an mpsc outbound sender);
  - `ProcessesChain`, which runs its processes in sequence at their period and hands each output to the next process;
  - `Scheduler`, with a main chain, side chains on dedicated threads (stopped through an `AtomicBool`) and registered interfaces.
- `robomorph::communications`: the `HardwareInterface` trait, `encode_frame` / `decode_frame`, and `UdpInterface` (RX and TX threads, broadcast fan-out to the processes).
- `robomorph::messages`, prost/protobuf types:
  - `MessageType` 0–4, the `AnyMessage` enum (`ImuState`, `GnssState`, `MotorState`, `PoseState`, `LidarState`);
  - the `Translatable` trait (`[1-byte type][protobuf]` frames);
  - motor, IMU, GNSS, `Pose` and lidar message types.
- Gazebo crate: the `VehicleController` trait and `OspraiController` (a `Process`: subscribes to Gazebo topics, publishes tilt setpoints, sends telemetry). `main.rs` runs it at 50 Hz with a `UdpInterface` (127.0.0.1:8080 → 8090).
- `gazebo_simulation/simulation_debug_gui` ("OSPR-AI Monitor", PySide6 / pyqtgraph). It takes cameras and lidar straight from Gazebo, takes IMU, GNSS and motors over UDP from the Rust program, and shows an attitude indicator, plots and a motors table.

#### Changed
- Cameras renamed `camera_left/right` on `osprai/cameras/*`. New dependencies: prost, tokio, chrono.

#### Removed
- `godot/`: the Godot simulation, the `polymorph` GDExtension and the Osprey models.
- The whole `Hardware/` tree: KiCad projects, notes, datasheets.
- The companion UKF and its `main.rs`, and `Visualization.py`.
- The old robomorph modules:
  - `communication.rs`;
  - `core/{worker, event_management, file_logger, messages, utils}.rs`;
  - `control/pid.rs`;
  - `filtering/{kalman_filter, mahony_filter}.rs`;
  - `lidar_management/*`;
  - `positionning/pose.rs`.

#### Known issues
- `__pycache__` files committed with the debug GUI.

### `01c21b2` — 2026-06-08 — Asserv arm orientation OK

#### Added
- `quaternion_to_euler()` (roll, pitch, yaw, gimbal-lock safe). The IMU attitude is stored and printed every cycle.
- A test-only fixed `world_anchor` joint that pins `base_link` to the world, to tune the arm tilt in isolation.

#### Changed
- Loop period 300 ms → 20 ms. The arm sweep is replaced by a fixed 0 rad tilt setpoint; rotor commands are commented out.
- Tilt `JointPositionController` retuned: p 25 → 0.2, i 1 → 0, d 2 → 0.01, output ±20 → ±0.5 N·m. The SDF explains the inertia and step-size reasoning.
- Joint effort 5 → 1, spawn height 0.2 → 0.06 m.

#### Fixed
- Lidar field mapping: `min_horizontal_angle` and `horizontal_resolution` were read from the wrong message fields.

### `14ac918` — 2026-06-06 — Setup Gazebo environment OK

#### Added
- Gazebo Harmonic world `gazebo_simulation/environments/classic_env.sdf` (`my_environment`):
  - 1 ms physics step;
  - `spherical_coordinates` (WGS84, ENU) so NavSat works;
  - physics, user-commands, scene-broadcaster, sensors, imu and navsat systems;
  - a helipad, buildings, a wall and a tower.
- The OsprAi tilt-rotor bicopter model (`vehicles/osprai/model.sdf`):
  - links `base_link`, `arm_left/right`, `rotor_left/right`;
  - revolute `tilt_*` / `spin_*` joints;
  - sensors: IMU at 250 Hz (`osprai/imu`), stereo cameras, `gpu_lidar` (`osprai/lidar`), navsat (`osprai/gps`);
  - plugins: 2× `MulticopterMotorModel` (maxRotVelocity 1200, motorConstant 1.2e-5, momentConstant 0.016), 2× `JointPositionController`, `OdometryPublisher`, `JointStatePublisher` (`/osprai/joint_state`).
- Crate `gazebo_vehicles_controller` (`gz` 0.10, harmonic feature, and `robomorph`). It subscribes to IMU, GPS, lidar and joint states into an `Arc<Mutex<OsprAiState>>`, and a 300 ms test loop sweeps the arm tilts and rotor speeds.

---

## 2026-02 → 2026-03 — Hardware design

### `e6344ac` — 2026-03-06 — FOC Controller Board Schematic OK

#### Added
- KiCad project `MotorController`, a FOC BLDC controller:
  - STM32G474RETx, DRV8305NPHPR three-phase gate driver;
  - 6× BSC040N08NS5 MOSFETs, LMR33620 buck converter;
  - AS5048A magnetic encoder, 2× WS2812C LEDs;
  - Zener/TVS diodes, bulk capacitors, connectors.
- DRV8305 vendor symbol and footprint.
- On the flight-controller board: `network_region` (ESP32-C3-WROOM-02), `actuators_region`, a placeholder `foc_bldc_controller` sheet, and an IST8310 magnetometer in `positioning_region`.
- `Hardware/Note_FOC.txt`, a primer on FOC: gate driver, MOSFET parameters, shunt sizing, DC-link capacitors.
- Datasheets and KiCad backups.

#### Changed
- Hardware notes translated to English, with the component-choice rationale: STM32G474, DRV8305, 1 mΩ shunts with gain 20.

### `ab99711` — 2026-02-26 — Add Hardware board project

#### Added
- KiCad project `PolyMorphFC_V0`, the flight-controller board:
  - STM32F405RGTx;
  - a "Positioning Region" sheet with an ICM42688-P IMU and a BMP388 barometer.
- Custom symbol library `PolyMorph_Parts.kicad_sym`.
- `Hardware/Notes_Hardware_PolyMorph.txt`: decoupling rationale, and checks to do before ordering (BMP388 decoupling, I2C rise time).

---

## 2026-01 → 2026-02 — State estimation: Mahony, then UKF

### `1924254` — 2026-02-16 — Setup Osprey Bicopter asset

#### Added
- Osprey bicopter meshes in `godot/Drone_models/` (body and wing). In the test scene the box robot is replaced by the body mesh (scale 0.3), with a new box collider and relocated sensor nodes.

#### Changed
- The UKF `FileLogger` becomes an `Option`, set to `None` (logging off), and an `Instant` timer is added.

### `757a0ae` — 2026-02-15 — Comments in the UKF implementations

#### Added
- Detailed doc comments in `imu_ukf.rs` and `kalman_filter.rs`:
  - state `x = [qw, qx, qy, qz]` (Hamilton);
  - measurement `y = [ax, ay, az, mx, my, mz]`;
  - NED world frame, 3×3 error covariance;
  - the predict / update algorithm, step by step.

#### Changed
- `OrientationUKF`'s own `predict` / `update_prediction` overrides removed in favour of the trait defaults.

#### Fixed
- The IMU test in `lib.rs` now sets `elapsed_time`.

#### Known issues
- The trait defaults still use the weight `w0 = 0.55`. The `w0 = 0.0` tuning validated in `316eeef` may therefore no longer apply. This comes from reading the code; it was never run.

### `738707e` — 2026-02-15 — Merge branch 'KalmanPositionning'
- Merge of `9baa38f`, empty diff.

### `9baa38f` — 2026-02-14 — UKF for orientation estimation OK
- Same patch and same tree as `316eeef`. The message adds that it "works only for a system without translations (fixed location)".

### `ae7c1a8` — 2026-02-14 — Merge branch 'KalmanPositionning'
- Brings `ead4f3c..316eeef` into `main`. The merged tree equals `316eeef`.

### `316eeef` — 2026-02-14 — UKF OK

#### Changed
- `OrientationUKF`:
  - weight `w0 = 0.0`, centre point included in the covariances;
  - cross-covariance on the rotation-vector error (`2·atan2`);
  - the update applies an exact axis-angle delta quaternion (`dq * predicted`).
- Tuning: `P0 = Q = 0.05·I₃`, accelerometer noise R 0.05, magnetometer noise R 0.1.
- Godot: `process` → `physics_process`, constant `GRAVITY = (0, −9.81, 0)`, "send late" warning restored.

#### Validation
- Validated in the Godot simulation for a vehicle that does not translate.

### `9714094` — 2026-02-13 — Add visualization dashboard

#### Added
- `Visualization.py`, a matplotlib dashboard on UDP 9000:
  - parses IMU, lidar and lidar-map frames;
  - draws the 2D scan and map, a 3D attitude view, and true vs. estimated roll/pitch/yaw over time.
- `IMUData.elapsed_time` (chunk `IMU_ELAPSED_TIME` 0x000d; the GNSS ids move to 0x000e / 0x000f).

#### Fixed
- Frame-orientation errors in the UKF:
  - the transition is rewritten as `q̇ = ½ q ⊗ [0, ω]`, with a consistent `[w, x, y, z]` order and renormalisation;
  - reference magnetic field normalised and shared with Godot;
  - `REF_REST_ACCEL` back to `[0, 0, 1]`, `Q = 0.01·I₃`.
- Godot divides the acceleration by `|g|`.

### `aefaf3d` — 2026-02-12 — UKF estimation drifting if gyro measurements != 0

#### Added
- `FileLogger`: timestamped `logs_*.txt` files, `add_logs`, a `test_logs` test. `Logs/` folders are ignored by git.

#### Changed
- UKF weights fixed at `w0 = 0.55`, `(1 − w0)/(2n)` for the others. `P0` becomes a constructor argument (`0.01·I₃`); alpha back to 0.1.

#### Fixed
- Godot IMU generation: gravity sign, normalised acceleration. The magnetometer is serialised as f32.

#### Known issues
- The estimate drifts as soon as the gyro reads non-zero (commit title).

### `0775018` — 2026-02-10 — Building file logger

#### Added
- Skeleton `core/file_logger.rs` (`FileLogger`, constructor only) and the `chrono` dependency.

#### Changed
- UKF update step re-enabled; alpha 0.1 → 1.0.
- Godot: joypad X/B give rotation commands, axes give full ±1.0 steps, and a "Send Late" timing warning is added. Lidar sending is commented out.

#### Fixed
- Bounds checks when parsing `IMUData`.

### `ead4f3c` — 2026-02-09 — UKF not works, it needs to be tuned

#### Added
- `robomorph/src/filtering/kalman_filter.rs`:
  - a generic `UnscentedKalmanFilter` trait: Cholesky sigma points, scaled weights (κ = 0, α / β), Kalman gain solved by QR;
  - the `NLKalmanFilter` trait and `KalmanMeasurements`.

  `faer` 0.24 replaces `ndarray`.
- `companion/src/filtering/imu_ukf.rs`, `OrientationUKF`:
  - quaternion state with a 3×3 error covariance;
  - sigma points built as rotation-vector perturbations of the mean;
  - quaternion mean from an eigen-decomposition;
  - gyro as the input, `[accel ; mag]` as the measurement.

  Settings: α = 0.1, β = 2, `P0 = I₃`, `Q = 0.001·I₃`, `R = diag(0.01×3, 0.1×3)`.
- `Pose` gets `GPSData` and a quaternion orientation. `utils::local_to_global_frame` / `global_to_local_frame` added. Godot gets `gpsFrequency` metadata and a GPS origin.

#### Changed
- f32 → f64 throughout (PID, Mahony, lidar). `BenchmarkFiltering` renamed to `FiltersManager`, now running the UKF.

#### Known issues
- The update step is commented out: the filter only predicts (commit title: "not works").

### `385250e` — 2026-01-15 — POC Dummy Lidar Object Mapping OK

#### Added
- `LidarMap`, serialisable, one bounding box (x/y min/max) per object. `LidarMap::update` merges a new object into a known one when their boxes intersect or one contains the other, keeping the larger one. Objects out of view are kept.

#### Changed
- `LidarPerceptionManager` takes an async `Pose` observer. Points are placed in the world frame (`LidarPoint::new_from_pose`), and the closest point is computed.
- `Worker` scheduling adds a fixed period per run (`get_frequency` added). The companion UDP worker runs on every frame (frequency −1), Godot UDP at 60 Hz, Mahony `dt = 1/60`.

#### Removed
- Godot stub modules `computer_vision`, `flight_controller`, `simcompanion`, `simsensor`.

#### Known issues
- Object updates still need work before they can be validated (commit message).

### `ab0c1bf` — 2026-01-09 — Mahony filter Tuned

#### Added
- `DataChunk::DebugChunk` (0xfeef). A Godot debug UDP worker (9010 → 9000) is set up for the ground-truth attitude; the sending itself is commented out.

#### Changed
- `MahonyFilter::new` takes a `d` gain, so the correction is a full PID. Tuning: Kp 20.0, Ki 0.01, Kd 3.25, max integral 10.0, `dt = 1/50`, leak 1.0. Kd is non-zero because a large Kp made roll and pitch oscillate.

#### Fixed
- Companion import paths (the previous commit referenced modules that did not exist).

#### Known issues
- Tested without magnetometer or gyro noise.

### `617a06e` — 2026-01-07 — POC Mahony Filter OK And Code organized in packages

#### Added
- `MahonyFilter`, orientation only:
  - quaternion state;
  - error = cross product of measured vs. expected gravity, plus the same for the magnetometer after its horizontal component is removed;
  - a PI per axis corrects the gyro;
  - Euler integration.
- `PIDController` with clamp anti-windup and a leak factor. `IMUData` and `Pose` are serialisable.
- Godot simulates the IMU: acceleration from velocity differences, world magnetic field `[1, 0, 0]` rotated into the body frame, `dataFrequency` metadata.
- Dependencies `ndarray`, `num-quaternion`, `socket2`.

#### Changed
- `robomorph` reorganised into `core/` (event management, messages, utils, worker), `filtering/`, `positionning/` and `control/`. `DataChunk` variants renamed to CamelCase.
- The UDP OS receive buffer is set through `SockRef`. `Worker` schedules on absolute timestamps and sleeps between runs.

#### Removed
- `gilrs` dependency.

#### Known issues
- The companion imports `filtering::kalman_filters::GenericEKF`, which does not exist at this commit. Fixed in `ab0c1bf`.

---

## 2025-12 — Lidar perception

### `b8c4470` — 2025-12-31 — Lidar Obstacle Segmentation OK

#### Added
- `robomorph/src/lidar_management/`, replacing `lidar.rs`:
  - `measurements.rs`: `LidarPoint` (angle, distance, x/y, `cluster_id`), `LidarObject`, `LidarMeasurements`, and a `LIDAR_OBSTACLES` chunk (0x000c) carrying one bound index per object;
  - `segmentation_algorithms.rs`: the `SegmentationAlgorithm` trait and `ClassicSolver`, a DBSCAN-like clustering along the scan order (neighbours under a distance threshold, expansion from core points);
  - `lidar_perception_manager.rs`: `LidarPerceptionManager` (observer → `detect_objects` → processed-measurements event).
- `utils::euclidean_distance`.

#### Changed
- The companion `MailBox` routes UDP → MailBox → perception → MailBox → UDP. Solver settings: threshold 0.10, minimum 2 points, 2D Euclidean distance.

### `1e5c02a` — 2025-12-29 — Lidar angles express in relative frame

#### Added
- Gamepad control in `AutonomyNode::input` (deadzone 0.2): a `motion_command` array and `motionFactor` metadata drive the robot's angular velocity. Joypad events added to the Godot input map.

#### Changed
- Measured angles are now relative to the robot. The robot's global yaw plus `lidarAngleOffset` (renamed from `lidarRelativeMidAngle`) is applied only to the raycast direction.

### `38856d3` — 2025-12-28 — POC Lidar Scan OK

#### Added
- `gilrs` dependency and a captured test frame.

#### Changed
- `LidarMeasurements` becomes two parallel `Vec`s (angles, distances) plus an `angle_in_deg` flag, replacing the `HashMap<OrderedFloat, f32>`.
- Wire format: start angle plus angle step (`LIDAR_ANGLES_CONFIG` 0x000a) instead of first / last angle. Degrees are converted to radians on serialisation.
- UDP receive buffer 1024 → 4096 bytes. Godot UDP worker at 50 Hz, ray angles wrapped with `modulo_pi_f64`.

#### Removed
- `order_by_angle`.

### `8cfcb52` — 2025-12-23 — Lidar Data Visualization Pipeline OK

#### Added
- `downcast-rs`: `Translatable` and `Module` can be downcast, and `Worker::get_module()` added.
- `utils::modulo_2pi` / `modulo_pi` (f32 and f64), with tests including `parse_real_frame`.
- Companion `MailBox` module that loops frames back, wired through `WorkerFactory` (UDP 8090 → 9000).

#### Changed
- `Worker::try_run()` returns a `bool`. `UDPChannel::publish_message` fills the command observer's buffer instead of triggering `frame_event`.
- Godot `AutonomyNode` wraps its `UDPChannel` in a 1 Hz `Worker`, stores FOV and offset in radians, and publishes `LidarMeasurements` frames.
- The lidar frame no longer writes its size bytes.

---

## 2025-12 — Framework rework: workers, UDP, serialization

### `d608782` — 2025-12-22 — Merge branch 'Total_Rework' (on `main`)
- Brings the whole rework into `main`; the resulting tree equals `85620fc`. Not on the first-parent history of `Total_Rework`.

### `85620fc` — 2025-12-22 — Worker Factory V0 OK

#### Added
- `WorkerFactory`: register / link / remove / start / stop workers, one by one or all at once. Test `test_worker_factory`.
- A stub `LidarPerceptionManager` implementing `Module`.
- `companion/` again. It creates a UDP channel (8090 → 8080) and a `LidarPerceptionManager`, not yet connected.

#### Changed
- `Worker::new` takes an `is_async` flag and an `i64` frequency (≤ 0 means "run on every call" via `force_run`). Scheduling is based on `Instant`.

### `b136054` — 2025-12-21 — Serialization / Deserialization OK

#### Added
- `messages.rs`, the first binary frame protocol:
  - `SOF = 0xabcd`, then a u16 frame size, then chunks;
  - `DataChunk` ids: COMMAND 0x0001, TELEOPERATION 0x0002, GNSS_POS 0x0003, INTERNAL_PERCEPTION 0x0004, LIDAR_SCAN 0x0005;
  - the `Translatable` trait (`fill_from_bytes`, `to_bytes`), `convert_to_frame`, and `parse_frame` (a SOF / size / chunk state machine).
- `lidar.rs`: `LidarMeasurements` with a big-endian chunk (angle range, count, f32 distances). Round-trip test.

#### Changed
- `publish_cmd` → `publish_message`, `set_data_observer` → `add_data_observer`. Godot `AutonomyNode` publishes its lidar chunk every `process()`.

### `6603e06` — 2025-12-18 — Workers, communication between them and UDP interface OK

#### Added
- A new `robomorph/`, rewritten from scratch:
  - `event_management.rs`: `Observer` (sync callback behind `Arc<Mutex<dyn FnMut>>`, or async buffer) and `Event`;
  - `worker.rs`: the `Module` trait and `Worker` (a module at a set frequency, inline or on a dedicated thread, chainable with `set_next_worker`);
  - `communication.rs`: the `Channel: Module` trait and `UDPChannel` (queued outgoing bytes, received frames published on an event).
- Tests: chained dummy workers, and two UDP channels exchanging messages across threads.

### `ffca476` — 2025-12-15 — Project Rework

#### Removed
- `companion/`, `controller/` and the whole `robomorph/` crate. Only `README.md` and `godot/` remain; this is the start of the `Total_Rework` branch.

---

## 2025-11 → 2025-12 — Bootstrap: Godot, companion, first robomorph

### `4ec4ee3` — 2025-12-09 — POC Serialization / Deserialization System OK

#### Added
- A first `Translatable` trait (via `downcast-rs`), feature messages, and a `Translator` (SOF + table of feature properties). The protocol is described in the README: SOF `0xabcd`, feature ids 0x0001–0x0006, lidar sub-ids.
- Round-trip test.

#### Known issues
- Frame building in `send_message` is commented out, so an empty frame is sent. `AutonomyNode` and the controller reference names that do not exist in robomorph.

### `84a3d39` — 2025-12-05 — Com UDP AutonomyNode => Controller Slint Application OK

#### Added
- Async observers (`new_async`, buffered incoming data) and `UDPChannel::set_socket`.

#### Changed
- The controller GUI moves from iced to Slint 1.14.1: `ControllerWindow` with camera and lidar image slots, and plots drawn with plotters into a pixel buffer.
- `AutonomyNode` sends lidar measurements over UDP (8080 → 8090). The payload is still a placeholder.

### `f390be8` — 2025-11-30 — Controller GUI with Factory to run Worker threads

#### Added
- `controller/` crate, an iced GUI whose `WorkerFactory` starts the workers when the window opens and stops them when it closes (UDP 8090 → 8080 at 50 Hz).

#### Changed
- `LidarMeasurements` keyed by angle (`ordered-float`). `UDPChannel` gets a `frequency`.

#### Removed
- `remote_controller/` crate.

### `73448cd` — 2025-11-28 — Lidar measure sequence in right order

#### Added
- `utils::normalize_angle`, test `sim_lidar`.

#### Changed
- The lidar is configured from node metadata (`lidarPoints`, `lidarFov`, `lidarRelativeMidAngle`) and does one ordered sweep over the FOV.

### `ae6a349` — 2025-11-28 — Merge remote-tracking branch 'origin/main'
- Trivial merge: the tree equals `e21c2e5`.

### `e21c2e5` — 2025-11-28 — LiDAR implementation OK

#### Added
- `AutonomyNode` (Godot Node3D): a software lidar casting 200×2 rays of 50 m per frame through `intersect_ray`.
- `robomorph/src/messages.rs` with a `Message` enum including `LidarMeasurements`.

#### Changed
- In the scene, `StaticVehicle` becomes `Robot`, and the obstacles get static bodies.

#### Known issues
- The measurements only reach a local printing observer; UDP send of the lidar is `todo!()`.

### `8467f78` — 2025-11-28 — LiDAR implementation OK
- IDE (`.idea`) files only. Second parent of `ae6a349`.

### `a5a0131` — 2025-11-26 — Setup robomorph lib import for polymorph and companion projects

#### Changed
- `godot/polymorph` depends on `robomorph`.

#### Fixed
- The `companion → robomorph` path.

### `26c016a` — 2025-11-26 — Creation of a RoboMorph library

#### Added
- The top-level `robomorph/` library. Channels, events and processes move into it from `companion`, with a two-UDP-channel test.

#### Removed
- The duplicate modules in `godot/polymorph`.

### `644c938` — 2025-11-26 — Channel and Module / Worker code sanitation in Companion project

#### Added
- In the companion:
  - the `Process` trait;
  - `ModuleLinker` (data and next-module events, to chain modules);
  - `Worker`, which runs a process at a set frequency on its own thread;
  - `WorkerFactory`.
- Godot stereo and mono camera viewports.

#### Changed
- The Godot crate is renamed `flight_controller` → `godot/polymorph`.

### `ab6e290` — 2025-11-23 — Communication between Companion and Godot Flight Controller OK

#### Added
- First working link: plain UDP on localhost between the companion (8090) and the Godot flight controller (8080).

#### Changed
- `Worker` split into `start_routine` (threaded) and `start_task` (one step). The Godot side runs sequentially because of threading issues inside Godot.

### `3b3b865` — 2025-11-20 — Setup projet for RUST

#### Added
- UDP channel, event / observer and worker abstractions, and a `FlightController` Godot class starting a UDP channel.

#### Changed
- The Godot project moves to `godot/`. The `flight_controller` binary is replaced by a GDExtension crate.

#### Removed
- The Terrain3D addon and its demo.

### `2dbcabe` — 2025-11-15 — Setup project

#### Added
- Hello-world crates `companion/`, `flight_controller/`, `remote_controller/`.

#### Removed
- The first `Simulation/robomorph` GDExtension.

### `d87a318` — 2025-11-15 — SLAM & CAM scene Ready

#### Added
- Godot 4.5 project `Simulation/godot`: a lidar/camera test scene with a vehicle, two cameras and about 23 obstacles.
- An empty Rust GDExtension `Simulation/robomorph`.
- Vendored Terrain3D addon.

### `a329748` — 2025-11-15 — Initial commit
- One-line `README.md`.

---

## Unmerged branches

These commits exist only on their remote branch. None of their code is in `Total_Rework`.

### Branch `embedded`

Forked from `e6344ac` (2026-03-06). Two successive phases: Rust / Embassy firmware, then C / C++ STM32 firmware.

#### `8e6121b` — 2026-05-23 — Base firmware architecture OK

##### Added
- `firmwares/CoreProject`, an STM32CubeIDE + CMake project in C++ for the STM32L476RG (TIM1 PWM, TIM2 interrupt, UART5 at 9600 baud).
- `EventsManagement.h`: `Observer<T>` (callback + id, `BROADCAST_ID = 255`) and `Publisher<T, N>` (`trigger(data)` or `trigger(data, obsId)`).
- `Worker.h`: frequency-gated workers chained through a `Publisher<void>`.
- `StaticVector.h`, `UARTInterface` (interrupt-driven RX), sensor and filter interfaces (`NLKalman` with stub `ImuUKF` / `ImuEKF`).
- `GNSSMeasurements`, an NMEA GGA / RMC parser.
- `Motor`, an open-loop servo on TIM1 CH1 (270°, 0.5–2.5 ms pulse over a 20 ms frame).

##### Known issues
- `updateAngle` only ever adds to the angle. IMU and UKF calls are commented out. The `Debug/` build output is committed.

#### `be1e78b` — 2026-04-10 — Footprint placements and screens

##### Changed
- Motor phases split into a hierarchical `MotorPhase` sheet instantiated three times. Layout replicated with the ReplicateLayout plugin.

##### Added
- Routing screenshots.

#### `5b0f572` — 2026-04-09 — Schematic OK & Setup FOC Controller board firmware using STM IDE (C / C++)

##### Added
- `firmwares/FOCController`, a CubeMX project for the STM32G431 (inside the STSPIN32G4). The toolchain recorded in the `.ioc` is IAR EWARM. `main.c` is generated code only: clocks, TIM1 complementary PWM with break inputs, I2C3.

##### Changed
- The motor-controller board moves from STM32G474 + DRV8305 to the **STSPIN32G4**, with 6× BSC040N08NS5 MOSFETs and an AS5048A encoder. PCB populated.

##### Removed
- The whole `polymorph_embedded` Rust crate.

#### `3712e77` — 2026-03-20 — Rework of Event - Observer system according rust constraints

##### Added
- `Orchestrator<N_WORKERS, MAX_OBS>`: workers in a `heapless` map, an execution graph, and a callback queue. Events returned by a module are routed **by id** to `exec_callback` on the targets.
- `EventData`, `ModuleStatus`.

##### Removed
- The reference-based observers (`&mut dyn EmbeddedObserver`), which conflicted with the borrow checker.

#### `357cfdb` — 2026-03-12 — Event - Observer sync system OK

##### Added
- `no_std` library `embed_core`:
  - `PolyVec<T, N>`, a fixed-capacity vector;
  - `EmbeddedObserver` / `EmbeddedEventPublisher`, synchronous publish;
  - `EmbeddedWorker`, frequency-gated and chainable.
- An example toggling two GPIOs at 1000 Hz and 10 Hz through chained workers.

#### `f7766d8` — 2026-03-06 — Flashing Board and Embassy setup OK

##### Added
- Crate `polymorph_embedded`:
  - Embassy executor / time / stm32 for the STM32L476RG;
  - target `thumbv7em-none-eabihf`, runner `probe-rs`;
  - a defmt hello world.

### Branch `MotionControl`

Forked from `1924254` (2026-02-16). A Godot prototype of Osprey wing control, superseded by the Gazebo work (`01c21b2`, then `b3f837f`), which shares no code with it.

#### `58fa064` — 2026-02-26 — Balancing wings of the drone

##### Changed
- Scene only: the waypoint and the wings' x offset.

#### `616fb5a` — 2026-02-25 — Setup Bicopter control pipeline

##### Changed
- The wings become `RigidBody3D`s attached with hinge joints.

##### Added
- A thruster loop: two PIs on the distance to the waypoint, turned into clamped wing velocities (`maxThrust = 1.15`).
- `PIDController::get_integral_error()`.

#### `ac87928` — 2026-02-23 — Control the orientation of the wings OK

##### Added
- The `MixerModel` trait (`apply_command_law`) and `OspreyBicopterMixer`. Each wing follows `±(heading) ∓ 90°` with its own P-only PID (Kp 0.1, 1° deadband, rate limit 100°/s).
- Angle helpers `modulo_180` / `modulo_360`, and direction-vector helpers.

##### Changed
- `PIDController` returns 0 inside its deadband.

#### `999d8ac` — 2026-02-17 — Bicopter wing rotating OK

##### Added
- Separate left and right wing meshes, rotated ±90° with the joypad Y button (direct setpoint, no control law yet).
- A `robomorph::actuators` stub (`ActuatorControl`, `ServoControl` with `todo!()`).

##### Fixed
- The `gpsFrequency` metadata fallback, which set the wrong field.
