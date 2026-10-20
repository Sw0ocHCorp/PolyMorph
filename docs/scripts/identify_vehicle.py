#!/usr/bin/env python3
"""Identify, from one flight recording, the vehicle parameters the reference model needs, and
check the vehicle description the controller uses against the flight.

Purpose: give docs/scripts/delay_budget_reference.py its free-flight parameters from MEASUREMENTS
only. Nothing here reads the simulator's parameters (no SDF aerodynamic or motor constant): the
same identification runs unchanged on a real robot, whose recording has the same columns.

Generic: the vehicle comes as arguments (mass, CoM, rotors with their spin, the joints that tilt
them, the inertia the controller uses). Nothing in this file is specific to one vehicle.

Data used, all available on a real robot:
  - accelerometer (specific force, body)    IMU                            columns ax ay az
  - gyro (body)                              IMU                            columns wx wy wz
  - attitude q (body -> world)               IMU / attitude filter          columns qw qx qy qz
  - attitude setpoint q_d                    mission                        columns qd_w qd_x qd_y qd_z
  - linear velocity, world frame             state estimator                columns vx vy vz
  - motor commands and feedbacks (efforts)   mixer / ESC and servo telemetry columns motor_cmds motor_fb
  - the vehicle description: mass, CoM, rotor positions and spin directions, joint axes (geometry)

1. Lateral drag b (N per m/s). When the vehicle slides, the air acts on its rotors with a force
   opposite to the sliding velocity and roughly proportional to it ("rotor drag"). The thrust is
   along the rotor axes, about body z, so what the accelerometer reads on body x and y is the drag
   divided by the mass:
       m * a_y = -b * v_y          (body frame; v_body = q^-1 * v_world)
   Fitted by least squares through the origin over the samples where the vehicle slides
   (|v| > 0.2 m/s), on y and on x (consistency check). R^2 says how well a line explains the data.

2. Lever arm h (m): the drag acts at the rotors, h above the CoM. Geometry, not a fit.

3. Rotor time constants. A rotor follows its commanded speed as a first-order lag, faster when it
   accelerates than when it slows down. Speed is proportional to s = sqrt(thrust) (T = k w^2), so
   the lag is fitted on s and the unknown k cancels out. The command of a pass is applied at the
   next pass, and reaches the rotor a transport delay d later: between two control updates j and
   j+1, the command of pass j-2 still acts for d, then the command of pass j-1. The three
   unknowns (tau_up, tau_down, d) minimise the one-step prediction error over EVERY interval: a
   10 deg roll step only changes the rotor speeds by about 0.6 %, so a fit that kept only the
   intervals with a large gap would see the spin-up and nothing else.

4. Rigid-body moments, axis by axis (Euler's equation; the cross terms w x I w are negligible at
   these rates). Over a window of a few control updates, the change of the rate is the integral of
   the moments divided by the inertia:
       I_k * delta(w_k) = int(M_thrust_k) - c * delta(H_k) + k_m * int(D_k) + b * int(R_k)
     M_thrust = sum (p_i - CoM) x T_i d_i      thrusts on their axes d_i (tilted by their joints)
     H        = sum spin_i sqrt(T_i) d_i       rotor angular momentum, per unit c (c = J / sqrt(k))
     D        = sum -spin_i T_i d_i            rotor drag torque, per unit k_m (opposite to the spin)
     R        = (0, 0, h) x (-v_body_xy)       moment of the lateral drag, per unit b
   -c * delta(H) is the GYROSCOPIC term: changing a spinning rotor's speed (reaction torque) or
   tilting it (precession) pushes back on the airframe. The unknowns 1/I, c/I, k_m/I, b/I come out
   of one least-squares fit per axis: roll uses (M, H, R), yaw uses (M, H, D). They are compared
   with the description (inertia), with step 1 (b), and with each other (c is found on both axes).

5. Moment ratio: the moment the motors were commanded to produce (commands of the pass, TRUE
   geometry, k_m from step 4) against the moment the attitude law demanded, recomputed offline:
       alpha = e_R / tau^2 - (2 zeta / tau) w,   M_demand = I alpha     (stabilize: w_d = 0)
   1 when the allocation is right. Bug #9 of the Gazebo bridge gave 0.625 on roll.

Run (from the repository root), e.g. for the OSPRAI:
      python3 docs/scripts/identify_vehicle.py <recording.csv> --mass 1.2 --com 0 0 0.016667 \\
          --rotor 64 0 0.30 0.10 ccw --rotor 179 0 -0.30 0.10 cw \\
          --joint 74 64 0 1 0 --joint 189 179 0 1 0 --inertia 0.017 0.0239 0.0357 --tau 0.2
The last line it prints runs the reference model with the identified parameters.
Requires numpy.
"""
import argparse
import csv
import math

import numpy as np

# samples below this sliding speed (m/s) are left out of the drag fit: the drag is lost in noise
MIN_SLIDING_SPEED = 0.2
# a window of the moment fits spans this many control updates (about 0.1 s at 100 Hz)
WINDOW = 10
# an axis is identified only if its rate changes by more than this over a window (rms, rad/s)
MIN_EXCITATION = 0.01
# a moment ratio is judged only if the demand is larger than this (rms, N.m): below, what the axis
# receives is mostly the side effect of the other axes, not its own law
MIN_DEMAND = 5e-3


def pairs(field):
    """'id=value|id=value' -> {id: value}; an empty field gives an empty dict."""
    return {int(k): float(v) for k, v in (p.split("=") for p in field.split("|") if p)}


def rotation_matrices(q):
    """(N, 4) quaternions (w, x, y, z), body -> world  ->  (N, 3, 3) rotation matrices."""
    w, x, y, z = q.T
    return np.stack([np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
                     np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
                     np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1)], -2)


def rotate(axis, angle, v):
    """Rodrigues: v (N, 3) rotated by `angle` (N,) about the unit `axis` (3,)."""
    c, s = np.cos(angle)[:, None], np.sin(angle)[:, None]
    return v * c + np.cross(axis, v) * s + axis * (v @ axis)[:, None] * (1 - c)


def fit(columns, y):
    """Least squares y = X a. Returns a and R^2 (about the mean)."""
    x = np.column_stack(columns)
    a = np.linalg.lstsq(x, y, rcond=None)[0]
    residual = ((y - x @ a) ** 2).sum()
    return a, 1 - residual / ((y - y.mean()) ** 2).sum()


def cumulative_integral(f, t):
    """Trapezoidal running integral of f (N, ...) over t (N,), 0 at the first sample."""
    steps = 0.5 * (f[1:] + f[:-1]) * np.diff(t).reshape(-1, *([1] * (f.ndim - 1)))
    return np.concatenate([np.zeros((1,) + f.shape[1:]), np.cumsum(steps, axis=0)])


def rotor_lag_fit(t, u, s):
    """Step 3. u, s: (N, R) sqrt of the command and of the feedback. Returns (tau_up, tau_down, d)."""
    # 1. one entry per interval j -> j+1 and per rotor: old command, new command, s before, s after
    j = np.arange(2, len(t) - 1)
    u_old, u_new = u[j - 2].ravel(), u[j - 1].ravel()
    s0, s1 = s[j].ravel(), s[j + 1].ravel()
    dt = np.repeat(t[j + 1] - t[j], u.shape[1])
    taus = np.arange(0.002, 0.0605, 0.0005)
    best = None
    # 2. exhaustive search: transport delay d, then tau_up, with every tau_down at once
    for d in np.arange(0.0, 0.0085, 0.001):
        d_eff = np.minimum(d, dt)
        for tau_up in taus:
            td = taus[:, None]
            # 2a. during d the old command acts: up or down depending on where it is
            tau1 = np.where(u_old > s0, tau_up, td)
            s_mid = u_old + (s0 - u_old) * np.exp(-d_eff / tau1)
            # 2b. then the new command, for the rest of the interval
            tau2 = np.where(u_new > s_mid, tau_up, td)
            pred = u_new + (s_mid - u_new) * np.exp(-(dt - d_eff) / tau2)
            err = ((pred - s1) ** 2).sum(axis=1)
            k = int(np.argmin(err))
            if best is None or err[k] < best[0]:
                best = (err[k], tau_up, taus[k], d)
    # 3. the share of the interval-to-interval change the model leaves unexplained
    unexplained = best[0] / ((s1 - s0) ** 2).sum()
    return best[1], best[2], best[3], unexplained


def main():
    # 1. the command line: the recording, and the vehicle description
    parser = argparse.ArgumentParser(description="Identify the vehicle's free-flight parameters from a recording")
    parser.add_argument("recording")
    parser.add_argument("--mass", type=float, required=True, help="vehicle mass (kg)")
    parser.add_argument("--com", type=float, nargs=3, required=True, metavar=("X", "Y", "Z"),
                        help="CoM in the vehicle frame (m)")
    parser.add_argument("--rotor", nargs=5, action="append", required=True, metavar=("ID", "X", "Y", "Z", "SPIN"),
                        help="a rotor: motor id, position in the vehicle frame (m), spin seen from above its "
                             "thrust axis (ccw or cw). Repeat for each rotor")
    parser.add_argument("--joint", nargs=5, action="append", default=[], metavar=("ID", "ROTOR", "AX", "AY", "AZ"),
                        help="a joint that tilts a rotor: motor id, id of the rotor it carries, rotation axis in "
                             "the vehicle frame. Repeat for each joint")
    parser.add_argument("--inertia", type=float, nargs=3, required=True, metavar=("IXX", "IYY", "IZZ"),
                        help="the inertia the controller uses (kg.m^2)")
    parser.add_argument("--tau", type=float, required=True, help="attitude law time constant (s)")
    parser.add_argument("--zeta", type=float, default=1.0, help="attitude law damping (default 1)")
    parser.add_argument("--skip", type=float, default=1.0,
                        help="simulated seconds left out at the start: the rotors spin up from their minimum (default 1)")
    args = parser.parse_args()
    rotor_ids = [int(r[0]) for r in args.rotor]
    positions = np.array([[float(c) for c in r[1:4]] for r in args.rotor]) - np.array(args.com)
    if any(r[4] not in ("ccw", "cw") for r in args.rotor):
        parser.error("the spin of a rotor is ccw or cw")
    spins = np.array([1.0 if r[4] == "ccw" else -1.0 for r in args.rotor])
    joints = [(int(j[0]), rotor_ids.index(int(j[1])), np.array([float(c) for c in j[2:5]])) for j in args.joint]
    joints = [(jid, i, axis / np.linalg.norm(axis)) for jid, i, axis in joints]

    # 2. read the recording: header line, column names, then one row per pass. Only the passes that
    #    received a new pose are kept (the others repeat the previous one while Gazebo is stalled),
    #    after the spin-up, and with every field this script needs
    with open(args.recording) as f:
        lines = list(csv.reader(f))
    names = lines[1]
    missing = [c for c in ("ax", "ay", "az", "motor_fb") if c not in names]
    if missing:
        print("this recording has no %s column(s): record a new run" % ", ".join(missing))
        return
    rows = [dict(zip(names, line)) for line in lines[2:]]
    t_first = float(rows[0]["t_sim_s"])
    numeric = ("t_sim_s", "qw", "qx", "qy", "qz", "wx", "wy", "wz", "ax", "ay", "az", "vx", "vy", "vz",
               "qd_w", "qd_x", "qd_y", "qd_z")
    motor_ids = rotor_ids + [jid for jid, _, _ in joints]
    kept, cmds, fbs = [], [], []
    for r in rows:
        if r["fresh"] != "1" or "" in (r[c] for c in numeric) or float(r["t_sim_s"]) < t_first + args.skip:
            continue
        cmd, fb = pairs(r["motor_cmds"]), pairs(r["motor_fb"])
        if any(i not in cmd or i not in fb for i in motor_ids):
            continue
        kept.append([float(r[c]) for c in numeric])
        cmds.append(cmd)
        fbs.append(fb)
    if len(kept) < 3 * WINDOW:
        print("only %d usable control updates: record a longer run" % len(kept))
        return
    data = np.array(kept)
    t, q, w, acc, v_world, q_d = data[:, 0], data[:, 1:5], data[:, 5:8], data[:, 8:11], data[:, 11:14], data[:, 14:18]
    thrust_fb = np.array([[fb[i] for i in rotor_ids] for fb in fbs])
    thrust_cmd = np.array([[c[i] for i in rotor_ids] for c in cmds])
    angle_fb = np.array([[fb[jid] for jid, _, _ in joints] for fb in fbs]).reshape(len(t), len(joints))
    angle_cmd = np.array([[c[jid] for jid, _, _ in joints] for c in cmds]).reshape(len(t), len(joints))
    v_body = np.einsum("nji,nj->ni", rotation_matrices(q), v_world)

    # 3. lateral drag: body-frame velocity against body-frame specific force, while sliding
    sliding = np.hypot(v_world[:, 0], v_world[:, 1]) >= MIN_SLIDING_SPEED
    if sliding.sum() < 10:
        print("not enough sliding samples (%d): the vehicle never slid faster than %.1f m/s" % (sliding.sum(), MIN_SLIDING_SPEED))
        return
    #    3a. m a = -b v, a line through the origin: slope = sum(x y) / sum(x^2), R^2 about 0
    def through_origin(x, y):
        slope = (x @ y) / (x @ x)
        return slope, 1 - ((y - slope * x) ** 2).sum() / (y @ y)
    slope_y, r2_y = through_origin(v_body[sliding, 1], args.mass * acc[sliding, 1])
    slope_x, r2_x = through_origin(v_body[sliding, 0], args.mass * acc[sliding, 0])
    b = -slope_y

    # 4. lever arm of the drag: geometry only (mean rotor height above the CoM)
    h = positions[:, 2].mean()

    # 5. rotor time constants and transport delay, on sqrt(thrust)
    tau_up, tau_down, delay, unexplained = rotor_lag_fit(t, np.sqrt(np.maximum(thrust_cmd, 0.0)),
                                                         np.sqrt(np.maximum(thrust_fb, 0.0)))

    # 6. rigid-body moments
    #    6a. rotor axes, tilted by the joints that carry them (measured angles), and the four terms
    def rotor_axes(angles):
        d = np.tile([0.0, 0.0, 1.0], (len(t), len(rotor_ids), 1))
        for k, (_, i, axis) in enumerate(joints):
            d[:, i] = rotate(axis, angles[:, k], d[:, i])
        return d
    d_fb = rotor_axes(angle_fb)
    m_thrust = np.cross(positions, thrust_fb[:, :, None] * d_fb).sum(axis=1)
    momentum = (spins[None, :, None] * np.sqrt(np.maximum(thrust_fb, 0.0))[:, :, None] * d_fb).sum(axis=1)
    drag_torque = -(spins[None, :, None] * thrust_fb[:, :, None] * d_fb).sum(axis=1)
    v_plane = v_body * np.array([1.0, 1.0, 0.0])
    drag_moment = np.cross(np.array([0.0, 0.0, h]), -v_plane)
    #    6b. one row per window: change of the rate, and the integrals (or changes) of the terms
    start = np.arange(len(t) - WINDOW)
    end = start + WINDOW
    def over_windows(f):
        c = cumulative_integral(f, t)
        return c[end] - c[start]
    dw = w[end] - w[start]
    int_thrust, int_drag_torque, int_drag_moment = over_windows(m_thrust), over_windows(drag_torque), over_windows(drag_moment)
    d_momentum = momentum[end] - momentum[start]
    #    6c. the fits: roll and pitch with the lateral drag, yaw with the rotor drag torque. The
    #        coefficient of the thrust moment is 1/I; the others, divided by it, are the parameters
    axes = {}
    for k, name, third in ((0, "roll ", int_drag_moment), (1, "pitch", int_drag_moment), (2, "yaw  ", int_drag_torque)):
        excitation = np.sqrt((dw[:, k] ** 2).mean())
        if excitation < MIN_EXCITATION:
            axes[k] = (name, excitation, None)
            continue
        a, r2 = fit([int_thrust[:, k], -d_momentum[:, k], third[:, k]], dw[:, k])
        _, r2_no_gyro = fit([int_thrust[:, k], third[:, k]], dw[:, k])
        axes[k] = (name, excitation, (1 / a[0], a[1] / a[0], a[2] / a[0], r2, r2_no_gyro))
    k_m = axes[2][2][2] if axes[2][2] else 0.0

    # 7. moment ratio: commanded (true geometry, identified k_m) against demanded (law, offline)
    #    7a. the demand: e_R from q^-1 q_d (shortest path, log map), then the law, then I alpha
    w0, v0 = q[:, 0], -q[:, 1:]
    w1, v1 = q_d[:, 0], q_d[:, 1:]
    err_w = w0 * w1 - (v0 * v1).sum(axis=1)
    err_v = w0[:, None] * v1 + w1[:, None] * v0 + np.cross(v0, v1)
    flip = np.where(err_w < 0, -1.0, 1.0)
    err_w, err_v = err_w * flip, err_v * flip[:, None]
    norm_v = np.linalg.norm(err_v, axis=1)
    angle = 2 * np.arctan2(norm_v, err_w)
    e_r = np.where(norm_v[:, None] > 1e-6, err_v * (angle / np.maximum(norm_v, 1e-12))[:, None], 2 * err_v)
    m_demand = np.array(args.inertia) * (e_r / args.tau ** 2 - (2 * args.zeta / args.tau) * w)
    #    7b. what the commands of the same pass produce, with the commanded joint angles
    d_cmd = rotor_axes(angle_cmd)
    m_commanded = (np.cross(positions, thrust_cmd[:, :, None] * d_cmd).sum(axis=1)
                   - k_m * (spins[None, :, None] * thrust_cmd[:, :, None] * d_cmd).sum(axis=1))

    # 8. the results
    print("control updates used: %d (from t = %.2f s, %.2f s skipped)" % (len(t), t[0], args.skip))
    print("1. lateral drag b = %.4f N per m/s   (body y, R^2 = %.3f, %d sliding samples)" % (b, r2_y, sliding.sum()))
    if np.abs(v_body[sliding, 0]).max() > MIN_SLIDING_SPEED:
        print("                    %.4f N per m/s   (body x, R^2 = %.3f, consistency check)" % (-slope_x, r2_x))
    else:
        print("                    body x: the vehicle did not slide along x, no consistency check")
    print("2. lever arm h = %.4f m   (mean rotor height above the CoM, geometry)" % h)
    print("3. rotor time constants: up %.1f ms, down %.1f ms, transport delay %.0f ms   (%.1f %% of the change unexplained)"
          % (1000 * tau_up, 1000 * tau_down, 1000 * delay, 100 * unexplained))
    print("4. rigid-body moments, windows of %d control updates:" % WINDOW)
    for k, (name, excitation, result) in axes.items():
        if result is None:
            print("   %s  not excited (rate change %.3f rad/s rms): not identified" % (name, excitation))
            continue
        inertia, c, third, r2, r2_no_gyro = result
        third_text = ("lateral drag b = %.4f N per m/s (step 1: %.4f)" % (third, b)) if k < 2 else ("rotor drag torque k_m = %.4f N.m per N" % third)
        print("   %s  I = %.4f kg.m^2 (controller: %.4f)   gyroscopic c = %.4f   %s   R^2 = %.4f (%.4f without the gyroscopic term)"
              % (name, inertia, args.inertia[k], c, third_text, r2, r2_no_gyro))
    print("   (rotor drag torque on the airframe: -k_m T for a ccw rotor, +k_m T for a cw one)")
    print("5. commanded / demanded moment (same pass, true geometry):")
    ratios = {}
    for k, name in ((0, "roll "), (1, "pitch"), (2, "yaw  ")):
        demand_rms = np.sqrt((m_demand[:, k] ** 2).mean())
        if demand_rms < MIN_DEMAND:
            print("   %s  demand %.1f mN.m rms: too small to judge" % (name, 1000 * demand_rms))
            continue
        #   the slope of the line through the origin, and how well that line explains the commands
        x, y = m_demand[:, k], m_commanded[:, k]
        ratio = (x @ y) / (x @ x)
        r2 = 1 - ((y - ratio * x) ** 2).sum() / ((y - y.mean()) ** 2).sum()
        ratios[k] = ratio
        print("   %s  %.3f   (demand %.1f mN.m rms, R^2 = %.3f)" % (name, ratio, 1000 * demand_rms, r2))
    print("python3 docs/scripts/delay_budget_reference.py --scenario roll_step --moment-gain %.3f --drag %.4f --drag-arm %.4f --rotor-tau %.4f %.4f"
          % (ratios.get(0, 1.0), b, h, tau_up, tau_down))


if __name__ == "__main__":
    main()
