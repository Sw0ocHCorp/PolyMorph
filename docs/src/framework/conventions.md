# Conventions and invariants

These rules govern how data circulates between components. Each was learnt from a real bug — see [Lessons learned](../pipelines/motor-control/lessons.md).

## Setpoints and measurements are not handled the same way

Every controller has two kinds of inputs:

| Input | Comes from | Behaviour when nothing fresh arrives |
|---|---|---|
| **Setpoint** | the stage above, slower | **hold the last one** (zero-order hold) |
| **Measurement** | a sensor / the simulator | do not compute; keep it only to know its age |

<figure>
<svg class="diagram" viewBox="0 0 700 240" xmlns="http://www.w3.org/2000/svg" role="img" aria-label="A slow producer updates a setpoint that a fast consumer holds between updates, drawn as a staircase">
  <text x="46" y="34" font-size="11.5" opacity="0.8">setpoint producer — slow stage</text>
  <line x1="46" y1="52" x2="656" y2="52" stroke="currentColor" stroke-opacity="0.35" stroke-width="1"/>
  <circle cx="90" cy="52" r="5" fill="#4a90d9"/>
  <circle cx="270" cy="52" r="5" fill="#4a90d9"/>
  <circle cx="450" cy="52" r="5" fill="#4a90d9"/>
  <circle cx="630" cy="52" r="5" fill="#4a90d9"/>
  <text x="46" y="94" font-size="11.5" opacity="0.8">consumer — fast stage, one tick per mark</text>
  <line x1="46" y1="112" x2="656" y2="112" stroke="currentColor" stroke-opacity="0.35" stroke-width="1"/>
  <g stroke="#2aa198" stroke-width="1.8">
    <line x1="90" y1="105" x2="90" y2="119"/><line x1="150" y1="105" x2="150" y2="119"/>
    <line x1="210" y1="105" x2="210" y2="119"/><line x1="270" y1="105" x2="270" y2="119"/>
    <line x1="330" y1="105" x2="330" y2="119"/><line x1="390" y1="105" x2="390" y2="119"/>
    <line x1="450" y1="105" x2="450" y2="119"/><line x1="510" y1="105" x2="510" y2="119"/>
    <line x1="570" y1="105" x2="570" y2="119"/><line x1="630" y1="105" x2="630" y2="119"/>
  </g>
  <text x="46" y="156" font-size="11.5" opacity="0.8">value actually used by the consumer</text>
  <path d="M90 210 L270 210 L270 178 L450 178 L450 196 L630 196" fill="none" stroke="#cb8b1e" stroke-width="2"/>
  <g stroke="#4a90d9" stroke-width="1" stroke-dasharray="3 3" stroke-opacity="0.7">
    <line x1="90" y1="57" x2="90" y2="210"/><line x1="270" y1="57" x2="270" y2="178"/>
    <line x1="450" y1="57" x2="450" y2="196"/><line x1="630" y1="57" x2="630" y2="196"/>
  </g>
  <text x="350" y="232" text-anchor="middle" font-size="10.5" opacity="0.7">held between updates — never replaced by a default value</text>
</svg>
<figcaption>A cascade is multi-rate by construction; holding the setpoint is the correct semantics, not a workaround.</figcaption>
</figure>

**Setpoints are held.** Between two updates the downstream stage serves the last known setpoint at full rate — that is exactly what makes an inner loop look fast to the outer one. The hold adds at most one *upstream* period of delay, which is negligible because upstream is the slow one. Every controller therefore stores its last setpoint in its struct; this is legitimate state, since it only remembers and cannot wind up.

**Measurements are not held for computing.** A loop that computes on a stale measurement does not servo faster; it servos **at the measurement rate**. Worse, a derivative term fed a stale gyroscope reading is a fake derivative. For a critical loop the measurement must arrive **at least as fast as the controller runs**. On a tick with no fresh measurement — which must be exceptional — the controller **abstains** and the downstream stage holds.

## Missing data is never a zero

A controller that cannot compute must **never** publish a default value. A null wrench is not an abstention: it is the active order *"cut everything"*.

> Historical bug: on ticks with no fresh telemetry the attitude stage published `WorkVec::default()`. The mixer executed it faithfully and the rotors alternated at loop rate between "hold the vehicle up" and "idle".

The rule is `Option::None` for *nothing to say*, and the consumer holds. Behaviour before the *very first* setpoint must be **explicit and deliberate** (publish nothing until the first datum, or an initial setpoint written down in plain sight) — never a `Default::default()` nobody chose.

## Equal frequencies are not synchronised

Two threads at the same nominal frequency **never stay in phase**: their scheduling jitter makes them slide against each other, and on each slip of one period a tick receives 0 messages while another receives 2. This **beat** was measured at 30 empty / 30 double ticks out of 454.

Two ways to remove it:

1. **Same chain, producer registered before consumer.** Processes then share one clock and run in order: a value published and consumed **in the same pass, by construction**. This is the architecture in use.
2. **Strictly faster producer** (≥ 2×), for instance publishing from a sensor callback. Useful when producer and consumer must live on different threads.

From the control point of view, equal frequencies are healthy: the cascade's time-scale separation lives in the time constants, not in the tick rates. The only constraint is the floor of the fastest loop (`τ ≥ 10·T`).

## Two transports, two contracts

| Mechanism | Guarantee | Used for |
|---|---|---|
| **Pipe** (`Process::exec` return value) | delivered **only** within the same pass of the same chain | attitude → mixer |
| **broadcast channel** (tokio) | asynchronous, non-blocking, bounded capacity, `Lagged` when a receiver falls behind | telemetry, motor feedback, commands |

A `Lagged` error means *"messages were skipped, carry on"*. Treating it as a termination condition kills the consumer silently.

## Units and frames

- The control chain works in **efforts**: newtons for a thruster, radians for a joint. Motor-native units (rotor speed, PWM) exist only at the hardware boundary.
- A message that carries a unit must **declare** it (`MotorCommandType`). Emitting newtons under a `VELOCITY` label makes the message lie.
- Any quantity known in the world frame must pass through `q⁻¹` before being handed to a component that speaks the body frame.
- Check dimensional homogeneity on every equation written. An equation whose units do not balance is wrong before it runs — that check caught an inertia factor left at 1.

## Numerical safety

- **Guard the denominator, never the result.** A `NaN` is unordered: `NaN <= ε` is *false*, so a guard placed after the division never fires. `NaN`s are prevented, not repaired.
- **Never publish a non-finite command.** Gazebo's rotor model keeps a `NaN` in its first-order filter for the rest of the run; a real ESC does not recover on its own either.
- **Never compare floats with `==`** in tests; always use a tolerance.
- The norm of the accelerometer is a free state detector: ≈ 9.81 m/s² when supported, ≈ 0 in free fall.

## Period budget

Execution times add up inside the period (10 ms at 100 Hz). A `println!` is a blocking call; a very verbose stage can overrun a pass, and an overrun shifts process phases and breaks the pipe. Verbose output stays behind a flag, and a counter of late passes is the right health indicator.
