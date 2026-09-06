# Motor control: overview

The first robotics pipeline of the stack. It answers one question: **given a desired attitude, what command should each motor receive?**

## Data flow

<figure>
<svg class="diagram" viewBox="0 0 700 300" xmlns="http://www.w3.org/2000/svg" role="img" aria-label="Data flow: Gazebo sensors into the vehicle controller and the mixer, the chain of processes, and the command channel back to the actuators">
  <defs>
    <marker id="p-arw" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">
      <path d="M1 1 L9 5 L1 9" fill="none" stroke="currentColor" stroke-width="1.6" stroke-linecap="round" stroke-linejoin="round"/>
    </marker>
    <marker id="p-arw-b" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">
      <path d="M1 1 L9 5 L1 9" fill="none" stroke="#4a90d9" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round"/>
    </marker>
  </defs>
  <rect x="40" y="18" width="620" height="50" rx="6" fill="currentColor" fill-opacity="0.05" stroke="#cb8b1e" stroke-width="1.2"/>
  <text x="350" y="48" text-anchor="middle" font-size="12.5" font-weight="600">Gazebo Harmonic — sensors, actuators, physics</text>
  <line x1="100" y1="68" x2="100" y2="114" stroke="currentColor" stroke-width="1.3" marker-end="url(#p-arw)"/>
  <text x="92" y="96" text-anchor="end" font-size="10" opacity="0.8">sensors</text>
  <line x1="190" y1="116" x2="190" y2="72" stroke="currentColor" stroke-width="1.3" marker-end="url(#p-arw)"/>
  <text x="198" y="96" font-size="10" opacity="0.8">actuators</text>
  <line x1="575" y1="68" x2="575" y2="114" stroke="currentColor" stroke-width="1.3" marker-end="url(#p-arw)"/>
  <text x="585" y="96" font-size="10" opacity="0.8">motor feedback</text>
  <rect x="40" y="120" width="170" height="64" rx="6" fill="currentColor" fill-opacity="0.06" stroke="#cb8b1e" stroke-width="1.2"/>
  <text x="125" y="146" text-anchor="middle" font-size="12" font-weight="600">OspraiController</text>
  <text x="125" y="166" text-anchor="middle" font-size="10" opacity="0.75">bridge, effort law</text>
  <rect x="265" y="120" width="170" height="64" rx="6" fill="currentColor" fill-opacity="0.06" stroke="#4a90d9" stroke-width="1.2"/>
  <text x="350" y="146" text-anchor="middle" font-size="12" font-weight="600">AttitudeController</text>
  <text x="350" y="166" text-anchor="middle" font-size="10" opacity="0.75">geometric PD + feedforward</text>
  <rect x="490" y="120" width="170" height="64" rx="6" fill="currentColor" fill-opacity="0.06" stroke="#2aa198" stroke-width="1.2"/>
  <text x="575" y="146" text-anchor="middle" font-size="12" font-weight="600">MotorsMixer</text>
  <text x="575" y="166" text-anchor="middle" font-size="10" opacity="0.75">control allocation</text>
  <line x1="210" y1="146" x2="259" y2="146" stroke="currentColor" stroke-width="1.3" marker-end="url(#p-arw)"/>
  <text x="234" y="136" text-anchor="middle" font-size="9.5" opacity="0.8">Pose</text>
  <line x1="435" y1="146" x2="484" y2="146" stroke="#4a90d9" stroke-width="1.7" marker-end="url(#p-arw-b)"/>
  <text x="459" y="136" text-anchor="middle" font-size="9.5" fill="#4a90d9">wrench</text>
  <path d="M160 184 L160 214 L350 214 L350 190" fill="none" stroke="currentColor" stroke-width="1.2" stroke-dasharray="5 4" marker-end="url(#p-arw)"/>
  <text x="255" y="209" text-anchor="middle" font-size="9.5" opacity="0.75">attitude setpoint</text>
  <path d="M575 184 L575 254 L125 254 L125 190" fill="none" stroke="currentColor" stroke-width="1.2" stroke-dasharray="5 4" marker-end="url(#p-arw)"/>
  <text x="350" y="274" text-anchor="middle" font-size="10" opacity="0.75">command channel — motor efforts, applied on the next pass</text>
</svg>
<figcaption>Solid arrows: the scheduler pipe, within one pass. Dashed arrows: broadcast channels.</figcaption>
</figure>

All four processes run in the **same chain at 100 Hz**, registered in this order: `osprai → remote → attitude → mixer`. That guarantees one fresh measurement per tick and makes the pipe reliable — see [Conventions](../../framework/conventions.md).

## Stage contracts

| Stage | Measurement | Setpoint | Output | Frame |
|---|---|---|---|---|
| [Attitude controller](attitude-controller.md) | `q`, `ω` (IMU) | `q_d` | wrench `[f_body ; M]` | body |
| [Mixer](mixer.md) | motor feedbacks (efforts) | wrench | one command per motor | body → per-motor |
| [Vehicle controller](../../simulation/gazebo.md) | — | motor efforts | actuator units | hardware |

The **wrench** is the contract that makes each side replaceable: the mixer does not know what commands it, the attitude stage does not know which vehicle executes it.

## What exists and what does not

| | Status |
|---|---|
| Attitude loop (stabilize) + gravity feedforward | built, validated up to the static-equilibrium rung |
| Control allocation over an arbitrary motor tree | built, validated on the OSPRAI |
| Velocity loop, force→attitude resolver, position loop | [planned](roadmap.md) |
| Arming / spool-up sequencing, failsafe, data-age supervision | not started — belongs to a supervisor |

## The one-cycle delay

Commands computed in pass *k* are applied in pass *k+1*: the vehicle controller drains the command channel at the start of its `exec`. Every discrete controller pays a computation delay somewhere; putting it on actuation rather than measurement keeps the critical loop reading the freshest possible state, and it is covered by the `τ ≥ 10·T` margin.

## Reading order

1. [The motor model](motor-model.md) — the kinematic tree, and the distinction between an effectiveness column and a work vector. Read this before the mixer.
2. [The mixer](mixer.md) — the allocation itself.
3. [The attitude controller](attitude-controller.md) — the control law.
4. [Roadmap](roadmap.md), then [Validation](validation.md) and [Lessons learned](lessons.md).
