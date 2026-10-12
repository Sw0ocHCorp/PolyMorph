#!/usr/bin/env python3
"""Reference model of the attitude loop on the ROLL axis of the OSPRAI, with its actuation delays.

Purpose: show, with numbers, what the actuation-delay budget does to the ideal (tau, zeta)
response - the figure and the table of the section "The actuation-delay budget" of
docs/docs/math/cascaded-control.mdx come from here. It shares no code with the Rust stack.

Why not the ideal formulas alone: the closed-form responses of cascaded-control.mdx assume
instantaneous actuators. The real chain has an actuation-delay budget:
  - one control period T = 10 ms: the command of tick k reaches the rotors at tick k+1;
  - the rotors: first order on their SPEED (T = k w^2), 12.5 ms when accelerating and 25 ms when
    decelerating (Gazebo's MulticopterMotorModel, same constants as the SDF).
This model keeps only that: rigid roll axis, the attitude law, the allocation of a roll moment to
a differential thrust, the delay and the rotor lags. No arms, no translation, no mixer solver.

Model:
    theta'' = M / Ixx,   M = y (T_left - T_right),  y = 0.30 m (true lever arm)
    at each tick:  alpha = (theta_d - theta) / tau^2 - (2 zeta / tau) w,   M_cmd = Ixx alpha
                   T_left/right = T0 +/- M_cmd / (2 y),   T0 = m g / 2
    rotors: w_i -> w_i_cmd, exact first-order update at h = 1 ms

Run (from the repository root):
      python3 docs/scripts/delay_budget_reference.py
      python3 docs/scripts/delay_budget_reference.py --svg docs/docs/math/img/delay-budget.svg
      (the second form also rewrites the documentation figure: ideal vs delay budget, rate impulse)
"""
import math
import sys

# Vehicle (OSPRAI, from the SDF / main.rs): roll inertia (kg.m^2), rotor lever arm (m), mass (kg),
# gravity (m/s^2), rotor effort constant (N / (rad/s)^2)
IXX, Y, MASS, G, K = 0.017, 0.30, 1.2, 9.81, 1.2e-5
# Attitude loop: time constant (s), damping, control period (s), integration sub-step (s)
TAU, ZETA, PERIOD, H = 0.2, 1.0, 0.01, 0.001
# Rotor speed lags (s): accelerating / decelerating
TAU_UP, TAU_DOWN = 0.0125, 0.025
# Hover thrust per rotor (N)
T0 = MASS * G / 2
# The two test cases: a 10 deg roll step, and a 1 rad/s roll-rate kick, both at t = 1 s
EVENT_TIME, ROLL_STEP, RATE_IMPULSE = 1.0, math.radians(10.0), 1.0


def run(theta_d, impulse_at=None, duration=3.0):
    """Simulate the roll axis. `theta_d(t)` is the roll setpoint. Return [(t, theta, w)] per tick."""
    # 1. initial state: level, at rest, both rotors at hover speed (commanded and actual)
    theta, w = 0.0, 0.0
    speed_l = speed_r = math.sqrt(T0 / K)
    cmd_l = cmd_r = speed_l
    pending = None
    out = []
    # 2. one iteration per control tick
    for i in range(int(round(duration / PERIOD)) + 1):
        t = i * PERIOD
        # 2a. the kick, if this is its tick
        if impulse_at is not None and abs(t - impulse_at) < 1e-9:
            w += RATE_IMPULSE
        # 2b. the command computed at the PREVIOUS tick reaches the rotors (one-tick delay)
        if pending is not None:
            cmd_l, cmd_r = pending
        # 2c. the attitude law on the measurements of this tick: P on the angle, D on the rate
        alpha = (theta_d(t) - theta) / TAU**2 - (2 * ZETA / TAU) * w
        # 2d. allocation: the roll moment Ixx*alpha becomes a differential thrust +/- dT on the
        #     two rotors (moment = y * (T_left - T_right) = 2 y dT), then rotor speeds w = sqrt(T/k)
        d_thrust = IXX * alpha / (2 * Y)
        pending = (math.sqrt((T0 + d_thrust) / K), math.sqrt((T0 - d_thrust) / K))
        # 2e. record the state the law saw
        out.append((t, theta, w))
        # 2f. physics until the next tick, at sub-step H
        for _ in range(int(round(PERIOD / H))):
            # rotors: exact first-order update towards their command, up or down time constant
            speed_l += (cmd_l - speed_l) * (1 - math.exp(-H / (TAU_UP if cmd_l > speed_l else TAU_DOWN)))
            speed_r += (cmd_r - speed_r) * (1 - math.exp(-H / (TAU_UP if cmd_r > speed_r else TAU_DOWN)))
            # roll axis: M = y (k wl^2 - k wr^2), then semi-implicit Euler (w first, then theta)
            w += Y * K * (speed_l**2 - speed_r**2) / IXX * H
            theta += w * H
    return out


def interpolate(series, t):
    """Linear interpolation of [(t, x)] at time t (the last value after the end)."""
    for (t0, x0), (t1, x1) in zip(series, series[1:]):
        if t0 <= t <= t1:
            return x0 + (x1 - x0) * (t - t0) / (t1 - t0)
    return series[-1][1]


def impulse_figure(path):
    """SVG of the roll angle after the rate kick: ideal response vs the delay-budget model.
    Follows docs/docs/contributing.mdx: class `diagram`, 700 wide, currentColor, accent colours."""
    # 1. the two curves, in degrees, over 0.8 s after the kick
    #    ideal: theta(t) = w0 t e^(-t/tau) (zeta = 1, instantaneous actuators)
    ideal = [(s, math.degrees(RATE_IMPULSE * s * math.exp(-s / TAU))) for s in (i * 0.002 for i in range(401))]
    #    delay budget: this model, sampled at the ticks
    model = [(t - EVENT_TIME, math.degrees(th)) for t, th, _ in run(lambda t: 0.0, impulse_at=EVENT_TIME)
             if EVENT_TIME <= t <= EVENT_TIME + 0.8 + 1e-9]
    # 2. geometry: canvas, margins, and the mapping (time, degrees) -> SVG coordinates
    w, h, l, r, top, b = 700, 300, 64, 24, 20, 48
    x = lambda s: l + s / 0.8 * (w - l - r)
    y = lambda deg: h - b - deg / 5.0 * (h - top - b)
    polyline = lambda pts: " ".join(("M" if i == 0 else "L") + "%.1f %.1f" % (x(s), y(d)) for i, (s, d) in enumerate(pts))
    # 3. the peak of each curve
    pk_i = max(ideal, key=lambda p: p[1])
    pk_m = max(model, key=lambda p: p[1])
    # 4. the SVG: axes, tick labels, the two curves, the peak dots, the legend
    parts = [
        '<svg class="diagram" viewBox="0 0 %d %d" xmlns="http://www.w3.org/2000/svg" role="img" '
        'aria-label="Roll angle after a 1 rad/s roll-rate impulse: the ideal second-order response peaks at 4.2 degrees '
        'after 0.20 s; with the actuation-delay budget it peaks higher, at 4.7 degrees, and earlier, after 0.16 s">' % (w, h),
        '<line x1="%d" y1="%d" x2="%d" y2="%d" stroke="currentColor" stroke-opacity="0.5"/>' % (l, h - b, w - r, h - b),
        '<line x1="%d" y1="%d" x2="%d" y2="%d" stroke="currentColor" stroke-opacity="0.5"/>' % (l, top, l, h - b),
    ]
    for deg in (0, 1, 2, 3, 4, 5):
        parts.append('<text x="%d" y="%.1f" text-anchor="end" font-size="11" opacity="0.75">%d°</text>' % (l - 8, y(deg) + 4, deg))
    for s in (0.0, 0.2, 0.4, 0.6, 0.8):
        parts.append('<text x="%.1f" y="%d" text-anchor="middle" font-size="11" opacity="0.75">%.1f s</text>' % (x(s), h - b + 18, s))
    parts.append('<text x="%.1f" y="%d" text-anchor="middle" font-size="11" opacity="0.75">time after the impulse</text>' % ((l + w - r) / 2, h - 8))
    parts.append('<path d="%s" fill="none" stroke="#4a90d9" stroke-width="2.2"/>' % polyline(ideal))
    parts.append('<path d="%s" fill="none" stroke="#d4694a" stroke-width="2.2"/>' % polyline(model))
    # peaks marked by a dot on each curve; their values go in the legend (no label crosses a curve)
    for (s, d), colour in ((pk_i, "#4a90d9"), (pk_m, "#d4694a")):
        parts.append('<circle cx="%.1f" cy="%.1f" r="3.5" fill="%s"/>' % (x(s), y(d), colour))
    parts.append('<text x="%d" y="%d" text-anchor="end" font-size="11.5" fill="#4a90d9">blue: ideal, instantaneous actuators - peak %.2f° at %.2f s</text>'
                 % (w - r, top + 14, pk_i[1], pk_i[0]))
    parts.append('<text x="%d" y="%d" text-anchor="end" font-size="11.5" fill="#d4694a">coral: one tick + rotor lag - peak %.2f° at %.2f s</text>'
                 % (w - r, top + 32, pk_m[1], pk_m[0]))
    parts.append("</svg>")
    # 5. write the file
    with open(path, "w") as f:
        f.write("\n".join(parts) + "\n")
    print("figure written to", path)


if __name__ == "__main__":
    # 1. optional: rewrite the documentation figure
    if len(sys.argv) == 3 and sys.argv[1] == "--svg":
        impulse_figure(sys.argv[2])
    # 2. the roll step: fraction of the step recovered at tau, 2 tau, 3 tau, and the overshoot
    step = [(t, th) for t, th, _ in run(lambda t: ROLL_STEP if t >= EVENT_TIME else 0.0)]
    fractions = [interpolate(step, EVENT_TIME + k * TAU) / ROLL_STEP for k in (1, 2, 3)]
    overshoot = max(0.0, max(th for t, th in step if t >= EVENT_TIME) / ROLL_STEP - 1)
    # 3. the rate kick: peak of |theta| and when it happens
    kick = run(lambda t: 0.0, impulse_at=EVENT_TIME)
    t_peak, peak = max(((t, abs(th)) for t, th, _ in kick if t >= EVENT_TIME), key=lambda p: p[1])
    # 4. ideal vs delay budget, side by side
    print("ideal (no delay):   step fractions [0.264, 0.594, 0.801], kick peak %.5f rad at %.2f s"
          % (RATE_IMPULSE * TAU / math.e, TAU))
    print("with delay budget:  step fractions [%s], overshoot %.4f, kick peak %.5f rad at %.2f s"
          % (", ".join("%.4f" % f for f in fractions), overshoot, peak, t_peak - EVENT_TIME))
