# The Gazebo bridge

File: `gazebo_simulation/gazebo_vehicles_controller/src/vehicle_controllers/osprai_controller.rs`. This is the only component that knows Gazebo: it lifts the measurements, discovers the motor tree from the scene, converts efforts into actuator units and publishes the commands.

## Topics

| Topic | Direction | Content |
|---|---|---|
| `osprai/imu` | in | orientation `q` (body→world), gyro `ω` (body), accelerometer — 250 Hz |
| `osprai/gps` | in | NavSat → `GNSSMeasurement` |
| `osprai/lidar` | in | LaserScan → `LidarMeasurements` |
| `osprai/cameras/{left,right}` | in | stereo images |
| `/model/osprai/odometry` | in | ground-truth pose (printed in verbose mode: the **judge** of every convention test) |
| `/osprai/joint_state` | in | joint positions / velocities → motor feedbacks |
| `/osprai/command/motor_setpoints` | out | `Actuators`: rotor speeds (rad/s) and arm positions (rad) |
| `/world/my_environment/scene/info` | request | discovery of the model's links |

`gz-transport` callbacks run on Gazebo's own threads: they cannot borrow `self` and work on `Arc<Mutex<…>>` (`telemetry_state`, `motors`) and cloned senders.

## Building the motor tree

On connection the scene is queried: every `arm_*` link becomes a joint (`RotatingAroundY`, limits ±1.57 rad, speed 10 rad/s), every `rotor_*` link a thruster (`LinearMotionAlongZ`, `k = 1.2e-5`, `n = 2`, `k_m = ±0.016` signed by side). Ids are derived from the link name (byte sum). On the first `joint_state` message, parenthood is established (`tilt_*` → `arm_*`, `spin_*` → `rotor_*`), relative positions are recomputed in the parent's frame, rotor bounds are set (`min 0.1 N`, max = `k · 1200²`), and the `MotorController`s are pushed to the mixer on the configuration channel.

## Feedbacks: the effort law at the boundary

On every `joint_state`, for a rotor: `T = transmission · k · ω^n` is computed from the measured velocity and becomes `current_value`, **in newtons**; for an arm: the position, in radians. Feedbacks go out on the feedback channel to the mixer.

## Commands: the inverse law

On every `exec`, the mixer's `MotorCommand`s are converted into the actuator's unit: `THRUST` (N) → `ω = (T / (transmission·k))^(1/n)` (rad/s); `ANGULARPOSITION` → position (rad) unchanged. The `Actuators` message must contain exactly one value per motor, in the order `[rotor_left, rotor_right, arm_left, arm_right]`.

> **Absolute rule: never send a non-finite command to Gazebo.** The `MulticopterMotorModel` keeps a `NaN` in its first-order filter for the rest of the run — the rotor never spins again and the whole chain reads zero thrust. The same holds for real hardware: an ESC fed garbage does not recover on its own.

## Telemetry and setpoint

`exec` publishes the latest known pose on the telemetry channel (updated by the IMU callback at 250 Hz, hence less than 4 ms old) and a fixed attitude setpoint (identity) on the setpoint channel. Only the orientation of that setpoint is read by the attitude controller.

## The SDF model

- total mass 1.2 kg (base 1.0 + two arms + two rotors at 0.05 kg); the controller config carries the **total** inertia (0.017 / 0.0239 / 0.0357 kg·m²), consistent with the parallel-axis theorem applied to the offset masses;
- rotors at `y = ±0.30 m`, `z = 0.10 m`; arms at `y = ±0.18 m`;
- `MulticopterMotorModel` plugin: `motorConstant 1.2e-5`, `momentConstant 0.016`, `timeConstantUp 0.0125 s`, `timeConstantDown 0.025 s`, `maxRotVelocity 1200 rad/s`, rotor drag `8.06e-5` (this is what slowly brakes a residual lateral velocity);
- IMU at 250 Hz with:

```xml
<orientation_reference_frame>
  <localization>CUSTOM</localization>
  <custom_rpy parent_frame="world">0 0 0</custom_rpy>
</orientation_reference_frame>
```

**This block is mandatory.** Gazebo's default (`CUSTOM` with an empty `parent_frame`) means "the IMU reports in its local frame on boot": a vehicle spawned tilted then reads the identity, and a closed loop would faithfully hold whatever attitude it spawned in.

## Operational notes

- `default()` enables `verbose`, which prints a great deal (IMU, feedbacks, odometry): expensive inside the period budget.
- `send_telemetry` prints the full pose on every tick.
- The process order in the main chain must stay `osprai → … → attitude → mixer` so that the pose consumed is the one from the current pass.
