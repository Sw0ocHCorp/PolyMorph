# Architecture overview

## Layers

<figure>
<svg class="diagram" viewBox="0 0 700 360" xmlns="http://www.w3.org/2000/svg" role="img" aria-label="Three layers: application binary on top, robomorph library in the middle with its four modules, vehicle or simulator at the bottom">
  <defs>
    <marker id="o-arw" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">
      <path d="M1 1 L9 5 L1 9" fill="none" stroke="currentColor" stroke-width="1.6" stroke-linecap="round" stroke-linejoin="round"/>
    </marker>
  </defs>
  <rect x="60" y="16" width="580" height="58" rx="6" fill="currentColor" fill-opacity="0.06" stroke="#8b7cd8" stroke-width="1.2"/>
  <text x="350" y="40" text-anchor="middle" font-size="13" font-weight="600">Application — gazebo_vehicles_controller (binary)</text>
  <text x="350" y="60" text-anchor="middle" font-size="11" opacity="0.75">wires the processes together, owns the vehicle-specific bridge</text>
  <rect x="60" y="112" width="580" height="150" rx="6" fill="none" stroke="#4a90d9" stroke-width="1.2"/>
  <text x="76" y="134" font-size="12" font-weight="600" fill="#4a90d9">robomorph — generic library</text>
  <rect x="80" y="150" width="126" height="94" rx="5" fill="currentColor" fill-opacity="0.06" stroke="currentColor" stroke-opacity="0.35"/>
  <text x="143" y="176" text-anchor="middle" font-size="12" font-weight="600">core</text>
  <text x="143" y="196" text-anchor="middle" font-size="10.5" opacity="0.75">Process</text>
  <text x="143" y="212" text-anchor="middle" font-size="10.5" opacity="0.75">ProcessesChain</text>
  <text x="143" y="228" text-anchor="middle" font-size="10.5" opacity="0.75">Scheduler</text>
  <rect x="218" y="150" width="126" height="94" rx="5" fill="currentColor" fill-opacity="0.06" stroke="currentColor" stroke-opacity="0.35"/>
  <text x="281" y="176" text-anchor="middle" font-size="12" font-weight="600">messages</text>
  <text x="281" y="196" text-anchor="middle" font-size="10.5" opacity="0.75">AnyMessage</text>
  <text x="281" y="212" text-anchor="middle" font-size="10.5" opacity="0.75">Pose, WorkVec</text>
  <text x="281" y="228" text-anchor="middle" font-size="10.5" opacity="0.75">MotorModel</text>
  <rect x="356" y="150" width="126" height="94" rx="5" fill="currentColor" fill-opacity="0.06" stroke="currentColor" stroke-opacity="0.35"/>
  <text x="419" y="176" text-anchor="middle" font-size="12" font-weight="600">communications</text>
  <text x="419" y="196" text-anchor="middle" font-size="10.5" opacity="0.75">HardwareInterface</text>
  <text x="419" y="212" text-anchor="middle" font-size="10.5" opacity="0.75">wire codec</text>
  <text x="419" y="228" text-anchor="middle" font-size="10.5" opacity="0.75">UdpInterface</text>
  <rect x="494" y="150" width="126" height="94" rx="5" fill="currentColor" fill-opacity="0.06" stroke="#2aa198" stroke-width="1.2"/>
  <text x="557" y="176" text-anchor="middle" font-size="12" font-weight="600">control</text>
  <text x="557" y="196" text-anchor="middle" font-size="10.5" opacity="0.75">AttitudeController</text>
  <text x="557" y="212" text-anchor="middle" font-size="10.5" opacity="0.75">MotorsMixer</text>
  <text x="557" y="228" text-anchor="middle" font-size="10.5" opacity="0.75">MotorController</text>
  <rect x="60" y="300" width="580" height="52" rx="6" fill="currentColor" fill-opacity="0.06" stroke="#cb8b1e" stroke-width="1.2"/>
  <text x="350" y="322" text-anchor="middle" font-size="13" font-weight="600">Vehicle — Gazebo Harmonic simulation, or real hardware</text>
  <text x="350" y="341" text-anchor="middle" font-size="11" opacity="0.75">sensors, actuators, physics</text>
  <line x1="350" y1="74" x2="350" y2="106" stroke="currentColor" stroke-width="1.3" marker-end="url(#o-arw)"/>
  <line x1="350" y1="262" x2="350" y2="294" stroke="currentColor" stroke-width="1.3" marker-end="url(#o-arw)"/>
  <line x1="330" y1="294" x2="330" y2="262" stroke="currentColor" stroke-width="1.3" stroke-dasharray="5 4" marker-end="url(#o-arw)"/>
  <text x="640" y="284" text-anchor="end" font-size="11" opacity="0.7">commands ↓   measurements ↑</text>
</svg>
<figcaption>The library knows nothing about Gazebo; the binary knows everything vehicle-specific.</figcaption>
</figure>

## Crates

| Crate | Role |
|---|---|
| `robomorph/` | The generic library: scheduler, messages, communications, control (motor model, mixer, attitude, PID, gamepad). No dependency on Gazebo. |
| `gazebo_simulation/gazebo_vehicles_controller/` | The simulation binary: instantiates the chain for the OSPRAI and bridges to Gazebo Harmonic through `gz-transport` (`OspraiController`). |
| `gazebo_simulation/vehicles/osprai/` | The vehicle's SDF model: geometry, inertia, sensors, rotor plugins. |
| `gazebo_simulation/environments/` | The Gazebo world. |
| `companion/` | Empty shell today (future companion software). |

## Library modules

```
robomorph/src
├── core/scheduler.rs              Process, ProcessesChain, Scheduler
├── messages/
│   ├── registered_message.rs      AnyMessage, MessageType, Vec3, UnitQuat, Translatable
│   ├── motor_messages.rs          WorkVec, MotorModel, MotorFeedBack, MotorCommand, enums
│   ├── pose_messages.rs           Pose, IMUMeasurements, GNSSMeasurement, Transform
│   └── lidar_messages.rs          LidarMeasurements, Ray
├── communications/
│   ├── interface.rs               HardwareInterface, encode_frame / decode_frame
│   └── udp_interface.rs           UdpInterface (RX / TX threads)
└── control/
    ├── pid_controller.rs          scalar PIDController
    ├── joystick/                  RemoteControl, XboxPadControl (gilrs)
    └── motion/
        ├── motion_controller.rs   MotionController trait, VehicleKinematicConfig, GRAVITY
        ├── motor_controller.rs    MotorController: effectiveness columns and work vectors
        ├── motors_mixer.rs        MotorsMixer: control allocation
        └── attitude_controller.rs AttitudeController: attitude law
```

## The execution model in one paragraph

Everything that runs periodically is a **`Process`**. Processes are grouped in a **`ProcessesChain`**, which owns one clock and executes them in registration order. A `Scheduler` runs one chain on the caller's thread and any number of others on dedicated threads. Two transport mechanisms connect processes: the **pipe** (the value returned by one process becomes the input of the next one in the same chain) and **broadcast channels** (asynchronous, non-blocking, used across chains and with the sensor callbacks). See [The scheduler](scheduler.md) and [Conventions and invariants](conventions.md).

## Where the boundary lies

| Generic (in `robomorph`) | Vehicle-specific (in the binary) |
|---|---|
| control laws, allocation, motor model, scheduler, messages | the motor tree, mass / inertia / centre of mass, the effort law that converts to actuator units, topic names |

The boundary is strict and load-bearing: **the mixer works in efforts (newtons, radians) and knows nothing about rotor speeds.** The law `T = k·ωⁿ` and its inverse live in the vehicle controller, at the hardware boundary. Publishing newtons under a `VELOCITY` label would make the message lie about its own unit.
