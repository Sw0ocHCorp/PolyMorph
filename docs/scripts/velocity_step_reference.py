#!/usr/bin/env python3
"""Reference model of the velocity loop, for the predictions of the velocity scenarios.

The loop of `robomorph/src/control/motion/velocity_controller.rs`, on one horizontal axis of the
world frame:
    a_d = kp (v_d - v) + ki int(v_d - v),   kp = 2 zeta / tau_v,   ki = 1 / tau_v^2
sampled every PERIOD (the processes pipeline), the force m a_d delivered:
    - one tick late (the motor commands of pass k are applied at pass k + 1);
    - through the attitude loop: the lateral force follows the roll, and the roll follows its
      setpoint as a critically damped second order of time constant TAU_ATTITUDE without a zero,
      i.e. the force is filtered by 1 / (TAU_ATTITUDE s + 1)^2;
    - against the lateral drag F = -b v identified in free flight (docs/scripts/identify_vehicle.py,
      b = 0.1122 N per m/s on runs A and B);
    - minus the roll the attitude loop fails to hold while the vehicle slides. The drag acts on the
      rotors, h above the CoM: a moment h b v that pushes the roll back. The attitude law has no
      integrator, so it settles with a roll error e = tau_att^2 h b v / I (docs/docs/math/
      cascaded-control.mdx, "a constant disturbing moment leaves a steady error"): 1.26 deg per m/s
      with the identified h, b and roll inertia I, 1.41 deg per m/s measured on the bench. That error
      tilts the thrust back, a lateral force m g e lost in proportion to v: a second drag, 2.3 times
      the aerodynamic one, which removes the overshoot of the PI. It goes through the attitude loop
      like the setpoint (same lag). `--drag-arm 0` removes it.
Not modelled: the rotor lag (tens of ms against tau_v = 2 s), the roll-yaw coupling through the
tilting arms, the vertical axis.

Prints, for each edge of `velocity_step_y`, the fraction of the step covered at tau_v, 2 tau_v,
3 tau_v and the overshoot, as the report measures them (from the velocity at the edge). The
`--ideal` option drops the delay, the attitude lag and both drags: the textbook second order
(100 % at tau, 113.5 % at 2 tau).

Usage: python3 docs/scripts/velocity_step_reference.py [--tau-v 2.0] [--drag-arm 0.0833] [--inertia 0.017] [--ideal]
"""

import argparse

MASS, PERIOD, TAU_ATTITUDE, DRAG_B, GRAVITY= 1.2, 0.01, 0.2, 0.1122, 9.81
# rotor height above the CoM (m, geometry) and roll inertia (kg m^2, identified: 0.0170 on runs A and B)
DRAG_ARM, ROLL_INERTIA= 0.0833, 0.017
H= 1e-3  # integration step of the physics (s)


def simulate(tau_v, zeta, ideal, edges, duration, drag_arm=DRAG_ARM, inertia=ROLL_INERTIA):
    """The velocity v(t) along the axis for the piecewise constant setpoint `edges` [(t, v_d)]."""
    kp, ki= 2.0 * zeta / tau_v, 1.0 / tau_v ** 2
    # lateral force lost to the attitude's steady roll error, per m/s of sliding (N per m/s)
    roll_loss= MASS * GRAVITY * TAU_ATTITUDE ** 2 * drag_arm * DRAG_B / inertia
    v, integral, lag1, lag2, t= 0.0, 0.0, 0.0, 0.0, 0.0
    pending, applied, next_tick= 0.0, 0.0, 0.0
    samples= []
    while t < duration:
        # 1. a pass of the processes pipeline: the PI, its command applied at the next pass
        if t >= next_tick - 1e-12:
            v_d= [vd for (te, vd) in edges if te <= t][-1]
            error= v_d - v
            if t > 0.0:
                integral += error * PERIOD
            applied= pending if not ideal else MASS * (kp * error + ki * integral)
            pending= MASS * (kp * error + ki * integral)
            next_tick += PERIOD
        # 2. the force actually delivered: through the attitude loop, minus the roll that loop fails
        #    to hold against the drag moment, unless ideal
        if ideal:
            force= applied
        else:
            lag1 += (applied - roll_loss * v - lag1) / TAU_ATTITUDE * H
            lag2 += (lag1 - lag2) / TAU_ATTITUDE * H
            force= lag2
        # 3. the physics: the drag opposes the motion
        drag= 0.0 if ideal else DRAG_B * v
        v += (force - drag) / MASS * H
        t += H
        samples.append((t, v))
    return samples


def measure(samples, t_edge, tau_v, target, window):
    """Fractions of the step at tau_v, 2 tau_v, 3 tau_v and the overshoot, from the value at the edge."""
    at= lambda time: min(samples, key=lambda s: abs(s[0] - time))[1]
    y0= at(t_edge)
    fractions= [(at(t_edge + k * tau_v) - y0) / (target - y0) for k in (1, 2, 3)]
    highest= max((y - y0) / (target - y0) for (t, y) in samples if t_edge <= t < t_edge + window)
    return fractions, max(0.0, highest - 1.0)


def main():
    parser= argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--tau-v", type=float, default=2.0, help="time constant of the velocity loop (s)")
    parser.add_argument("--zeta", type=float, default=1.0, help="damping of the velocity loop")
    parser.add_argument("--drag-arm", type=float, default=DRAG_ARM, help="rotor height above the CoM (m), geometry; 0 removes the attitude's roll error")
    parser.add_argument("--inertia", type=float, default=ROLL_INERTIA, help="roll inertia (kg m^2), identified")
    parser.add_argument("--ideal", action="store_true", help="no delay, no attitude lag, no drag")
    args= parser.parse_args()
    # velocity_step_y: v_d = 0, then 0.5 m/s at 3 s, back to 0 at 13 s
    edges= [(0.0, 0.0), (3.0, 0.5), (13.0, 0.0)]
    samples= simulate(args.tau_v, args.zeta, args.ideal, edges, 24.0, args.drag_arm, args.inertia)
    for n, (t_edge, target) in enumerate(edges[1:], start=1):
        fractions, overshoot= measure(samples, t_edge, args.tau_v, target, 10.0)
        print(f"edge {n} at {t_edge:.0f} s -> {target} m/s: fractions "
              f"{', '.join(f'{f:.3f}' for f in fractions)}, overshoot {overshoot:.3f}")


if __name__ == "__main__":
    main()
