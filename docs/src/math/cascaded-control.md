# Cascaded control

## Why a cascade

Vehicle control is organised as **nested loops**, each servoing a quantity that is the derivative of the one above.

<figure>
<svg class="diagram" viewBox="0 0 700 450" xmlns="http://www.w3.org/2000/svg" role="img" aria-label="Vertical cascade: position loop, velocity loop, force to attitude resolver, attitude loop, mixer, motors">
  <defs>
    <marker id="k-arw" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">
      <path d="M1 1 L9 5 L1 9" fill="none" stroke="currentColor" stroke-width="1.6" stroke-linecap="round" stroke-linejoin="round"/>
    </marker>
  </defs>
  <rect x="210" y="24" width="280" height="48" rx="6" fill="currentColor" fill-opacity="0.04" stroke="currentColor" stroke-opacity="0.4" stroke-dasharray="6 4"/>
  <text x="350" y="53" text-anchor="middle" font-size="12.5" font-weight="600">Position loop</text>
  <text x="500" y="53" font-size="10.5" opacity="0.7">τ ≈ 3 s</text>
  <text x="200" y="53" text-anchor="end" font-size="10.5" opacity="0.7">p</text>
  <rect x="210" y="106" width="280" height="48" rx="6" fill="currentColor" fill-opacity="0.04" stroke="currentColor" stroke-opacity="0.4" stroke-dasharray="6 4"/>
  <text x="350" y="129" text-anchor="middle" font-size="12.5" font-weight="600">Velocity loop</text>
  <text x="350" y="146" text-anchor="middle" font-size="10" opacity="0.7">the cascade's only integrator</text>
  <text x="500" y="135" font-size="10.5" opacity="0.7">τ ≈ 1 s</text>
  <text x="200" y="135" text-anchor="end" font-size="10.5" opacity="0.7">v</text>
  <rect x="210" y="188" width="280" height="48" rx="6" fill="currentColor" fill-opacity="0.04" stroke="currentColor" stroke-opacity="0.4" stroke-dasharray="6 4"/>
  <text x="350" y="217" text-anchor="middle" font-size="12.5" font-weight="600">Force → attitude resolver</text>
  <rect x="210" y="270" width="280" height="48" rx="6" fill="currentColor" fill-opacity="0.07" stroke="#4a90d9" stroke-width="1.4"/>
  <text x="350" y="299" text-anchor="middle" font-size="12.5" font-weight="600">Attitude loop</text>
  <text x="500" y="299" font-size="10.5" opacity="0.7">τ = 0.2 s</text>
  <text x="200" y="299" text-anchor="end" font-size="10.5" opacity="0.7">q, ω</text>
  <rect x="210" y="352" width="280" height="48" rx="6" fill="currentColor" fill-opacity="0.07" stroke="#2aa198" stroke-width="1.4"/>
  <text x="350" y="381" text-anchor="middle" font-size="12.5" font-weight="600">Mixer — control allocation</text>
  <text x="200" y="381" text-anchor="end" font-size="10.5" opacity="0.7">feedbacks</text>
  <line x1="350" y1="72" x2="350" y2="100" stroke="currentColor" stroke-width="1.3" marker-end="url(#k-arw)"/>
  <text x="358" y="92" font-size="10.5" opacity="0.8">v_d</text>
  <line x1="350" y1="154" x2="350" y2="182" stroke="currentColor" stroke-width="1.3" marker-end="url(#k-arw)"/>
  <text x="358" y="174" font-size="10.5" opacity="0.8">f_d (world)</text>
  <line x1="350" y1="236" x2="350" y2="264" stroke="currentColor" stroke-width="1.3" marker-end="url(#k-arw)"/>
  <text x="358" y="256" font-size="10.5" opacity="0.8">q_d, thrust</text>
  <line x1="350" y1="318" x2="350" y2="346" stroke="#4a90d9" stroke-width="1.6" marker-end="url(#k-arw)"/>
  <text x="358" y="338" font-size="10.5" fill="#4a90d9">wrench [f ; M] (body)</text>
  <line x1="350" y1="400" x2="350" y2="428" stroke="currentColor" stroke-width="1.3" marker-end="url(#k-arw)"/>
  <text x="358" y="420" font-size="10.5" opacity="0.8">motor commands</text>
  <text x="76" y="24" font-size="10.5" opacity="0.6">dashed = planned</text>
  <text x="76" y="40" font-size="10.5" opacity="0.6">solid = built</text>
</svg>
<figcaption>Each stage produces the setpoint of the stage below. Only the two solid stages exist today.</figcaption>
</figure>

The physics itself is a cascade: a moment rotates the vehicle (fast dynamics), the rotation reorients the thrust, the thrust accelerates the mass (slow dynamics), and integrating acceleration gives velocity then position.

### Time-scale separation

If the inner loop is **5 to 10× faster** than the loop enclosing it, the outer loop may treat it as perfect: by the time position has moved, velocity has already reached its setpoint. The consequences:

- each loop is **tunable independently, from the inside out** — tune attitude, validate it, then never touch its gains again;
- each loop, taken alone, sees an almost pure **double integrator** (`moment → angle`, `force → position`), for which a PD suffices — provably, not empirically;
- all the non-linearity (rotation, under-actuation) is confined to one stage: the force → attitude resolver.

> Time-scale separation lives in the **time constants τ**, not in the execution rates. Every loop may run on the same tick — see [Conventions](../framework/conventions.md).

## Gains expressed as accelerations: (τ, ζ)

Take one axis of the attitude loop. If the law output a moment directly, `M = kp·e + kd·ė`, the closed loop would be `I·ë + kd·ė + kp·e = 0`: its behaviour would depend on the inertia `I`, so the gains would have to be retuned for every vehicle.

If instead the law computes a **desired angular acceleration** `α = kp·e + kd·ė` and converts at the very end with `M = I·α`, the closed loop becomes `ë + kd·ė + kp·e = 0` — **the inertia has vanished**. Identifying with the canonical second-order form `ë + 2ζωₙ·ė + ωₙ²·e = 0`:

```
kp = 1/τ²         τ : the closed-loop time constant (s)
kd = 2ζ/τ         ζ : the damping ratio — 1 = critical, the fastest without overshoot
```

A loop therefore has exactly two tunings, `τ` and `ζ`, both **vehicle-independent**: mass and inertia are absorbed by `m` and `I` at the end of the law. The same values suit a 500 g drone and a 50 kg rover.

## What (τ, ζ) actually looks like

<figure>
<svg class="diagram" viewBox="0 0 700 250" xmlns="http://www.w3.org/2000/svg" role="img" aria-label="Decay of the attitude error for a critically damped loop and an under-damped loop, with milestones at one, two and three time constants">
  <line x1="80" y1="200" x2="640" y2="200" stroke="currentColor" stroke-opacity="0.45" stroke-width="1"/>
  <line x1="80" y1="235" x2="80" y2="40" stroke="currentColor" stroke-opacity="0.45" stroke-width="1"/>
  <line x1="170" y1="200" x2="170" y2="205" stroke="currentColor" stroke-opacity="0.45" stroke-width="1"/><text x="170" y="220" text-anchor="middle" font-size="10.5" opacity="0.75">1τ</text><line x1="260" y1="200" x2="260" y2="205" stroke="currentColor" stroke-opacity="0.45" stroke-width="1"/><text x="260" y="220" text-anchor="middle" font-size="10.5" opacity="0.75">2τ</text><line x1="350" y1="200" x2="350" y2="205" stroke="currentColor" stroke-opacity="0.45" stroke-width="1"/><text x="350" y="220" text-anchor="middle" font-size="10.5" opacity="0.75">3τ</text><line x1="440" y1="200" x2="440" y2="205" stroke="currentColor" stroke-opacity="0.45" stroke-width="1"/><text x="440" y="220" text-anchor="middle" font-size="10.5" opacity="0.75">4τ</text><line x1="530" y1="200" x2="530" y2="205" stroke="currentColor" stroke-opacity="0.45" stroke-width="1"/><text x="530" y="220" text-anchor="middle" font-size="10.5" opacity="0.75">5τ</text><line x1="620" y1="200" x2="620" y2="205" stroke="currentColor" stroke-opacity="0.45" stroke-width="1"/><text x="620" y="220" text-anchor="middle" font-size="10.5" opacity="0.75">6τ</text>
  <line x1="80" y1="50" x2="640" y2="50" stroke="currentColor" stroke-opacity="0.18" stroke-width="1" stroke-dasharray="4 4"/>
  <text x="74" y="54" text-anchor="end" font-size="10.5" opacity="0.75">100 %</text>
  <text x="74" y="129" text-anchor="end" font-size="10.5" opacity="0.75">50 %</text>
  <text x="74" y="204" text-anchor="end" font-size="10.5" opacity="0.75">0</text>
  <text x="20" y="120" font-size="11" opacity="0.8" transform="rotate(-90 20 120)">remaining error</text>
  <polyline points="80.0,50.0 82.2,50.0 84.5,50.2 86.8,50.4 89.0,50.7 91.2,51.1 93.5,51.6 95.8,52.2 98.0,52.8 100.2,53.5 102.5,54.3 104.8,55.2 107.0,56.1 109.2,57.1 111.5,58.1 113.8,59.2 116.0,60.4 118.2,61.6 120.5,62.9 122.8,64.3 125.0,65.7 127.2,67.1 129.5,68.6 131.8,70.1 134.0,71.7 136.2,73.3 138.5,74.9 140.8,76.6 143.0,78.4 145.2,80.1 147.5,81.9 149.8,83.7 152.0,85.6 154.2,87.4 156.5,89.3 158.8,91.2 161.0,93.1 163.2,95.1 165.5,97.1 167.8,99.0 170.0,101.0 172.2,103.0 174.5,105.1 176.8,107.1 179.0,109.1 181.2,111.2 183.5,113.2 185.8,115.2 188.0,117.3 190.2,119.3 192.5,121.4 194.8,123.4 197.0,125.5 199.2,127.5 201.5,129.6 203.8,131.6 206.0,133.6 208.2,135.6 210.5,137.6 212.8,139.6 215.0,141.6 217.2,143.5 219.5,145.5 221.8,147.4 224.0,149.3 226.2,151.2 228.5,153.1 230.8,155.0 233.0,156.9 235.2,158.7 237.5,160.5 239.8,162.3 242.0,164.1 244.2,165.8 246.5,167.5 248.8,169.2 251.0,170.9 253.2,172.6 255.5,174.2 257.8,175.8 260.0,177.4 262.2,179.0 264.5,180.5 266.8,182.0 269.0,183.5 271.2,184.9 273.5,186.4 275.8,187.8 278.0,189.2 280.2,190.5 282.5,191.8 284.8,193.1 287.0,194.4 289.2,195.6 291.5,196.8 293.8,198.0 296.0,199.2 298.2,200.3 300.5,201.4 302.8,202.5 305.0,203.5 307.2,204.5 309.5,205.5 311.8,206.5 314.0,207.4 316.2,208.3 318.5,209.2 320.8,210.0 323.0,210.8 325.2,211.6 327.5,212.4 329.8,213.1 332.0,213.9 334.2,214.5 336.5,215.2 338.8,215.8 341.0,216.5 343.2,217.0 345.5,217.6 347.8,218.1 350.0,218.6 352.2,219.1 354.5,219.6 356.8,220.0 359.0,220.5 361.2,220.9 363.5,221.2 365.8,221.6 368.0,221.9 370.2,222.2 372.5,222.5 374.8,222.8 377.0,223.0 379.2,223.2 381.5,223.4 383.8,223.6 386.0,223.8 388.2,223.9 390.5,224.0 392.8,224.2 395.0,224.2 397.2,224.3 399.5,224.4 401.8,224.4 404.0,224.4 406.2,224.5 408.5,224.4 410.8,224.4 413.0,224.4 415.2,224.3 417.5,224.3 419.8,224.2 422.0,224.1 424.2,224.0 426.5,223.9 428.8,223.8 431.0,223.6 433.2,223.5 435.5,223.3 437.8,223.1 440.0,223.0 442.3,222.8 444.5,222.6 446.8,222.4 449.0,222.2 451.2,221.9 453.5,221.7 455.8,221.5 458.0,221.2 460.2,220.9 462.5,220.7 464.8,220.4 467.0,220.1 469.2,219.9 471.5,219.6 473.8,219.3 476.0,219.0 478.2,218.7 480.5,218.4 482.7,218.1 485.0,217.8 487.3,217.5 489.5,217.1 491.8,216.8 494.0,216.5 496.2,216.2 498.5,215.8 500.8,215.5 503.0,215.2 505.2,214.9 507.5,214.5 509.8,214.2 512.0,213.9 514.2,213.5 516.5,213.2 518.8,212.9 521.0,212.5 523.2,212.2 525.5,211.9 527.8,211.5 530.0,211.2 532.2,210.9 534.5,210.5 536.8,210.2 539.0,209.9 541.2,209.6 543.5,209.2 545.8,208.9 548.0,208.6 550.2,208.3 552.5,208.0 554.8,207.7 557.0,207.4 559.2,207.1 561.5,206.8 563.8,206.5 566.0,206.2 568.2,205.9 570.5,205.6 572.8,205.3 575.0,205.0 577.2,204.8 579.5,204.5 581.8,204.2 584.0,204.0 586.2,203.7 588.5,203.5 590.8,203.2 593.0,203.0 595.2,202.7 597.5,202.5 599.8,202.2 602.0,202.0 604.2,201.8 606.5,201.6 608.8,201.4 611.0,201.1 613.2,200.9 615.5,200.7 617.8,200.5 620.0,200.3" fill="none" stroke="#d4694a" stroke-width="1.8" stroke-dasharray="6 4"/>
  <polyline points="80.0,50.0 82.2,50.0 84.5,50.2 86.8,50.4 89.0,50.7 91.2,51.1 93.5,51.5 95.8,52.0 98.0,52.6 100.2,53.3 102.5,54.0 104.8,54.7 107.0,55.5 109.2,56.4 111.5,57.3 113.8,58.2 116.0,59.2 118.2,60.3 120.5,61.3 122.8,62.4 125.0,63.5 127.2,64.7 129.5,65.9 131.8,67.1 134.0,68.3 136.2,69.5 138.5,70.8 140.8,72.1 143.0,73.4 145.2,74.7 147.5,76.0 149.8,77.3 152.0,78.7 154.2,80.0 156.5,81.4 158.8,82.8 161.0,84.1 163.2,85.5 165.5,86.9 167.8,88.3 170.0,89.6 172.2,91.0 174.5,92.4 176.8,93.8 179.0,95.1 181.2,96.5 183.5,97.9 185.8,99.2 188.0,100.6 190.2,102.0 192.5,103.3 194.8,104.6 197.0,106.0 199.2,107.3 201.5,108.6 203.8,109.9 206.0,111.2 208.2,112.5 210.5,113.8 212.8,115.1 215.0,116.3 217.2,117.6 219.5,118.8 221.8,120.0 224.0,121.3 226.2,122.5 228.5,123.7 230.8,124.8 233.0,126.0 235.2,127.2 237.5,128.3 239.8,129.5 242.0,130.6 244.2,131.7 246.5,132.8 248.8,133.9 251.0,134.9 253.2,136.0 255.5,137.0 257.8,138.1 260.0,139.1 262.2,140.1 264.5,141.1 266.8,142.1 269.0,143.1 271.2,144.0 273.5,145.0 275.8,145.9 278.0,146.8 280.2,147.7 282.5,148.6 284.8,149.5 287.0,150.4 289.2,151.2 291.5,152.1 293.8,152.9 296.0,153.7 298.2,154.5 300.5,155.3 302.8,156.1 305.0,156.9 307.2,157.7 309.5,158.4 311.8,159.2 314.0,159.9 316.2,160.6 318.5,161.3 320.8,162.0 323.0,162.7 325.2,163.4 327.5,164.0 329.8,164.7 332.0,165.3 334.2,166.0 336.5,166.6 338.8,167.2 341.0,167.8 343.2,168.4 345.5,169.0 347.8,169.6 350.0,170.1 352.2,170.7 354.5,171.2 356.8,171.8 359.0,172.3 361.2,172.8 363.5,173.3 365.8,173.8 368.0,174.3 370.2,174.8 372.5,175.3 374.8,175.7 377.0,176.2 379.2,176.7 381.5,177.1 383.8,177.5 386.0,178.0 388.2,178.4 390.5,178.8 392.8,179.2 395.0,179.6 397.2,180.0 399.5,180.4 401.8,180.8 404.0,181.1 406.2,181.5 408.5,181.9 410.8,182.2 413.0,182.6 415.2,182.9 417.5,183.2 419.8,183.6 422.0,183.9 424.2,184.2 426.5,184.5 428.8,184.8 431.0,185.1 433.2,185.4 435.5,185.7 437.8,186.0 440.0,186.3 442.3,186.5 444.5,186.8 446.8,187.1 449.0,187.3 451.2,187.6 453.5,187.8 455.8,188.1 458.0,188.3 460.2,188.5 462.5,188.8 464.8,189.0 467.0,189.2 469.2,189.4 471.5,189.6 473.8,189.9 476.0,190.1 478.2,190.3 480.5,190.5 482.7,190.6 485.0,190.8 487.3,191.0 489.5,191.2 491.8,191.4 494.0,191.6 496.2,191.7 498.5,191.9 500.8,192.1 503.0,192.2 505.2,192.4 507.5,192.5 509.8,192.7 512.0,192.8 514.2,193.0 516.5,193.1 518.8,193.3 521.0,193.4 523.2,193.5 525.5,193.7 527.8,193.8 530.0,193.9 532.2,194.1 534.5,194.2 536.8,194.3 539.0,194.4 541.2,194.5 543.5,194.7 545.8,194.8 548.0,194.9 550.2,195.0 552.5,195.1 554.8,195.2 557.0,195.3 559.2,195.4 561.5,195.5 563.8,195.6 566.0,195.7 568.2,195.8 570.5,195.8 572.8,195.9 575.0,196.0 577.2,196.1 579.5,196.2 581.8,196.3 584.0,196.3 586.2,196.4 588.5,196.5 590.8,196.6 593.0,196.6 595.2,196.7 597.5,196.8 599.8,196.8 602.0,196.9 604.2,197.0 606.5,197.0 608.8,197.1 611.0,197.2 613.2,197.2 615.5,197.3 617.8,197.3 620.0,197.4" fill="none" stroke="#4a90d9" stroke-width="2.2"/>
  <circle cx="170.0" cy="89.6" r="3.6" fill="#4a90d9"/><circle cx="260.0" cy="139.1" r="3.6" fill="#4a90d9"/><circle cx="350.0" cy="170.1" r="3.6" fill="#4a90d9"/>
  <text x="196" y="78" font-size="10.5" fill="#4a90d9">26 %</text>
  <text x="286" y="127" font-size="10.5" fill="#4a90d9">59 %</text>
  <text x="376" y="158" font-size="10.5" fill="#4a90d9">80 % recovered</text>
  <line x1="470" y1="70" x2="500" y2="70" stroke="#4a90d9" stroke-width="2.2"/>
  <text x="508" y="74" font-size="11">ζ = 1 (critical)</text>
  <line x1="470" y1="92" x2="500" y2="92" stroke="#d4694a" stroke-width="1.8" stroke-dasharray="6 4"/>
  <text x="508" y="96" font-size="11">ζ = 0.5 (overshoots)</text>
</svg>
<figcaption>Error decay from an initial offset, in units of τ. This curve is the acceptance criterion of the step test.</figcaption>
</figure>

For a critically damped loop released from an initial error with zero rate, the error follows

```
e(t) = (1 + t/τ) · e^(−t/τ)
```

so the milestones are: **26 %** of the error gone after `τ`, **59 %** after `2τ`, **80 %** after `3τ`, and a 2 % settling time of about **5.8 τ**. The envelope decays as `e^(−t/τ)`; τ is the time constant of the pole, not a "63 % point" — that rule of thumb belongs to first-order systems and understates a second-order loop's settling time by a factor of about five.

For a **velocity impulse** `ω₀` (the shape of a disturbance rejection, and of a bump on take-off) the error is `e(t) = ω₀·t·e^(−t/τ)`, which peaks at `t = τ` with

```
e_max = ω₀ · τ / e ≈ 0.368 · ω₀ · τ
```

then returns monotonically to zero. Both expressions are what a logged `θ(t)` must be compared against.

## τ is constrained, ζ is chosen

- **τ** is bounded from below by two measurable floors: **sampling** (`τ ≳ 10·T`, where `T` is the loop period — otherwise one cycle of delay eats the damping) and **actuator dynamics** (`τ ≳ 3–5 ×` the rotor time constant). Take the more constraining one.
- **ζ** is a free choice; no property of the vehicle imposes it. `ζ = 1` is the right default: an attitude overshoot is a thrust-direction overshoot, and actuator lag *eats* effective damping — asking for 1 and getting ~0.8 stays healthy. It would only vary by **mode** (acro ≈ 0.7), never by vehicle.

`kp` and `kd` are never stored as independent tunings: they derive from `τ` in one place. Nudging `kd` "by 20 %" would silently change ζ and forfeit the second-order guarantee. ζ stays in the formulas because it documents where the `2` in `2/τ` comes from, and because the *effective* ζ measured from a step's overshoot `D` is a diagnostic tool: `ζ = −ln(D)/√(π² + ln²D)`.

## State feedback: "P on the angle, P on the rate"

The attitude law reads:

```
α_k = (1/τ²) · e_R[k]  +  (2ζ/τ) · (ω_d[k] − ω[k])
```

These are **two proportional feedbacks**: one on the angle error, one on the angular-rate error. No derivative is computed anywhere: angular velocity *is* the derivative of attitude (`d(e_R)/dt ≈ ω_d − ω` to first order) and the gyroscope **measures** it — cleanly, without lag, without amplifying noise, and without the kick a setpoint step gives a numerical derivative. The rate term plays the functional role of a PID's D (it damps), hence the name, but it is not "the D of a PID on the angle".

This reading has a name: **state feedback**. The rotational state of a rigid body is the pair (attitude, angular rate) — two quantities, not three. Two states → two feedbacks → complete state feedback. There is **no** third "P on acceleration" term: `α` is the law's *output*, not a measurement compared to a setpoint, and `M = I·α` is a unit conversion, not a feedback.

The whole cascade follows this pattern: each stage is a pure P on its own error, and the derivative it needs for damping is supplied by the stage below, which servos the derived quantity ("the velocity loop *is* the position loop's D term"). Numerical differentiation — the classic PID's main noise source — does not exist in the chain.

## Where the integrator lives, and windup

An integral term corrects constant biases (wind, mis-estimated mass, thrust that does not match its model). Rule: **one integrator per physical quantity in the whole cascade**, and it lives in the **velocity loop** — the stage that sees those biases directly. Attitude does not need one (the gravity feedforward removes its main bias) and neither does position.

**Windup** is the pathology specific to accumulation: while the actuators saturate, `∫e·dt` keeps piling up an error nothing can absorb; when saturation clears, the accumulator discharges as a massive overshoot. The criterion:

> **A stateless term cannot wind up. Only a term that accumulates needs anti-windup.**

The attitude loop, being stateless, needs none. The velocity loop will, and the right form is **conditional integration**: accumulate only when the mixer reports the demand is serviceable — information its residual already carries.

## Feedforward: compensate the known, feed back the unknown

Feedback corrects what could not be predicted; what is known in advance is compensated by **feedforward**. In stabilize the force needed to hold flight is known exactly: `m·g` upward in the world, rotated into the body: `f_body = q⁻¹·(0, 0, m·g)`. Leaving that job to an integrator would cost a transient error on every take-off and windup on every saturation.

## Saturate every output to what the stage below can serve

A far-away position setpoint would demand an enormous velocity, then an enormous force, and everything would saturate arbitrarily. Every loop **saturates its own output** to what the next stage can physically deliver — the stage-by-stage equivalent of the mixer's projection onto the feasible box.
