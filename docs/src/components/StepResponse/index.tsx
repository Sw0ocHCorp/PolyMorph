/**
 * Interactive step response of a second-order loop tuned by (τ, ζ).
 *
 * Two loop structures share the same poles (ωn = 1/τ, damping ζ) but not the same zeros:
 *
 *  - "attitude" — P on the error, D on the MEASURED rate (the attitude law):
 *        Y/R = ωn² / (s² + 2ζωn·s + ωn²)                      no zero
 *  - "velocity" — PI on the error (the velocity law, kp = 2ζ/τ, ki = 1/τ²):
 *        Y/R = (2ζωn·s + ωn²) / (s² + 2ζωn·s + ωn²)           one zero
 *
 * With y0(t) the step response of the first one and h(t) = dy0/dt its impulse response, the
 * second one is exactly y(t) = y0(t) + (2ζ/ωn)·h(t): the zero adds a multiple of the
 * derivative, which is where the overshoot comes from even at ζ = 1.
 *
 * Everything is computed in closed form (no numerical integration), for the three regimes
 * ζ < 1, ζ = 1, ζ > 1.
 */
import React, {useMemo, useState} from 'react';
import styles from './styles.module.css';

type Curve = {t: number[]; y: number[]};

/** Step response y0 and impulse response h of ωn²/(s² + 2ζωn s + ωn²). */
function secondOrder(t: number, wn: number, zeta: number): {y0: number; h: number} {
  if (Math.abs(zeta - 1) < 1e-6) {
    const e = Math.exp(-wn * t);
    return {y0: 1 - (1 + wn * t) * e, h: wn * wn * t * e};
  }
  if (zeta < 1) {
    const wd = wn * Math.sqrt(1 - zeta * zeta);
    const e = Math.exp(-zeta * wn * t);
    return {
      y0: 1 - e * (Math.cos(wd * t) + ((zeta * wn) / wd) * Math.sin(wd * t)),
      h: ((wn * wn) / wd) * e * Math.sin(wd * t),
    };
  }
  const s = wn * Math.sqrt(zeta * zeta - 1);
  const r1 = -zeta * wn + s;
  const r2 = -zeta * wn - s;
  const e1 = Math.exp(r1 * t);
  const e2 = Math.exp(r2 * t);
  return {y0: 1 + (r2 * e1 - r1 * e2) / (r1 - r2), h: (wn * wn * (e1 - e2)) / (r1 - r2)};
}

/** Exact response at time t: y0 for the attitude structure, y0 + (2ζ/ωn)·h with the PI zero. */
function valueAt(t: number, tau: number, zeta: number, withZero: boolean): number {
  const wn = 1 / tau;
  const {y0, h} = secondOrder(t, wn, zeta);
  return withZero ? y0 + ((2 * zeta) / wn) * h : y0;
}

function response(tau: number, zeta: number, withZero: boolean, horizon: number, n: number): Curve {
  const t: number[] = [];
  const y: number[] = [];
  for (let i = 0; i <= n; i++) {
    const ti = (horizon * i) / n;
    t.push(ti);
    y.push(valueAt(ti, tau, zeta, withZero));
  }
  return {t, y};
}

/** Figures of merit, the same ones the validation ladder checks. The milestones at τ, 2τ, 3τ
 *  are evaluated exactly (closed form); peak and settling time are read on the sampled curve. */
function metrics(c: Curve, tau: number, zeta: number, withZero: boolean) {
  const at = (k: number) => valueAt(k * tau, tau, zeta, withZero);
  let peak = -Infinity;
  let tPeak = 0;
  c.y.forEach((yi, i) => {
    if (yi > peak) {
      peak = yi;
      tPeak = c.t[i];
    }
  });
  let settle = 0;
  for (let i = c.y.length - 1; i >= 0; i--) {
    if (Math.abs(c.y[i] - 1) > 0.02) {
      settle = c.t[Math.min(i + 1, c.t.length - 1)];
      break;
    }
  }
  return {y1: at(1), y2: at(2), y3: at(3), overshoot: Math.max(0, peak - 1), tPeak, settle};
}

const pct = (x: number) => `${(100 * x).toFixed(1)} %`;
const inTau = (t: number, tau: number) => `${(t / tau).toFixed(2)} τ`;

export default function StepResponse({defaultTau = 0.2, defaultZeta = 1}: {defaultTau?: number; defaultZeta?: number}) {
  const [tau, setTau] = useState(defaultTau);
  const [zeta, setZeta] = useState(defaultZeta);
  const [showAttitude, setShowAttitude] = useState(true);
  const [showVelocity, setShowVelocity] = useState(true);

  // Plot over 8τ; metrics over 25τ so that the 2 % settling time exists down to ζ = 0.3.
  const plotted = useMemo(
    () => ({
      attitude: response(tau, zeta, false, 8 * tau, 400),
      velocity: response(tau, zeta, true, 8 * tau, 400),
    }),
    [tau, zeta],
  );
  const measured = useMemo(
    () => ({
      attitude: metrics(response(tau, zeta, false, 25 * tau, 5000), tau, zeta, false),
      velocity: metrics(response(tau, zeta, true, 25 * tau, 5000), tau, zeta, true),
    }),
    [tau, zeta],
  );

  // Geometry of the plot (SVG user units; the SVG scales to the column width).
  const W = 640;
  const H = 300;
  const m = {l: 48, r: 16, t: 14, b: 40};
  const yMax = Math.max(
    1.2,
    ...(showAttitude ? plotted.attitude.y : [1]),
    ...(showVelocity ? plotted.velocity.y : [1]),
  ) * 1.04;
  const x = (t: number) => m.l + (t / (8 * tau)) * (W - m.l - m.r);
  const y = (v: number) => H - m.b - (v / yMax) * (H - m.t - m.b);
  const path = (c: Curve) => c.t.map((ti, i) => `${i ? 'L' : 'M'}${x(ti).toFixed(1)} ${y(c.y[i]).toFixed(1)}`).join(' ');

  const rows = [
    {key: 'attitude', show: showAttitude, label: 'Attitude: D on the measured rate', cls: styles.attitude, r: measured.attitude},
    {key: 'velocity', show: showVelocity, label: 'Velocity: PI on the error', cls: styles.velocity, r: measured.velocity},
  ];

  return (
    <div className={styles.box}>
      <div className={styles.controls}>
        <label>
          <span>τ = {tau.toFixed(2)} s</span>
          <input type="range" aria-label="Time constant tau, seconds" min={0.05} max={2} step={0.05} value={tau} onChange={(e) => setTau(Number(e.target.value))} />
        </label>
        <label>
          <span>ζ = {zeta.toFixed(2)}</span>
          <input type="range" aria-label="Damping ratio zeta" min={0.3} max={2} step={0.05} value={zeta} onChange={(e) => setZeta(Number(e.target.value))} />
        </label>
        <label className={styles.check}>
          <input type="checkbox" checked={showAttitude} onChange={(e) => setShowAttitude(e.target.checked)} />
          <span className={styles.attitude}>attitude structure</span>
        </label>
        <label className={styles.check}>
          <input type="checkbox" checked={showVelocity} onChange={(e) => setShowVelocity(e.target.checked)} />
          <span className={styles.velocity}>velocity structure</span>
        </label>
      </div>

      <svg viewBox={`0 0 ${W} ${H}`} className={styles.plot} role="img"
           aria-label={`Step responses for tau ${tau.toFixed(2)} s and zeta ${zeta.toFixed(2)}`}>
        {/* ±2 % band around the setpoint */}
        <rect x={m.l} y={y(1.02)} width={W - m.l - m.r} height={y(0.98) - y(1.02)} className={styles.band} />
        <line x1={m.l} x2={W - m.r} y1={y(1)} y2={y(1)} className={styles.setpoint} />
        {[1, 2, 3].map((k) => (
          <g key={k}>
            <line x1={x(k * tau)} x2={x(k * tau)} y1={m.t} y2={H - m.b} className={styles.guide} />
            <text x={x(k * tau)} y={H - m.b + 14} textAnchor="middle" className={styles.tick}>{k === 1 ? 'τ' : `${k}τ`}</text>
          </g>
        ))}
        <line x1={m.l} x2={W - m.r} y1={H - m.b} y2={H - m.b} className={styles.axis} />
        <line x1={m.l} x2={m.l} y1={m.t} y2={H - m.b} className={styles.axis} />
        {[0, 0.5, 1].map((v) => (
          <text key={v} x={m.l - 6} y={y(v) + 4} textAnchor="end" className={styles.tick}>{v}</text>
        ))}
        <text x={W - m.r} y={H - 6} textAnchor="end" className={styles.tick}>t = 8τ = {(8 * tau).toFixed(2)} s</text>
        {showAttitude && <path d={path(plotted.attitude)} className={`${styles.curve} ${styles.attitudeStroke}`} />}
        {showVelocity && <path d={path(plotted.velocity)} className={`${styles.curve} ${styles.velocityStroke}`} />}
      </svg>

      <div className={styles.tableScroll}>
      <table className={styles.table}>
        <thead>
          <tr>
            <th>Structure</th><th>y(τ)</th><th>y(2τ)</th><th>y(3τ)</th><th>Overshoot</th><th>Peak at</th><th>2 % settling</th>
          </tr>
        </thead>
        <tbody>
          {rows.filter((r) => r.show).map((r) => (
            <tr key={r.key}>
              <td className={r.cls}>{r.label}</td>
              <td>{pct(r.r.y1)}</td>
              <td>{pct(r.r.y2)}</td>
              <td>{pct(r.r.y3)}</td>
              <td>{pct(r.r.overshoot)}</td>
              <td>{r.r.overshoot > 1e-4 ? inTau(r.r.tPeak, tau) : '—'}</td>
              <td>{inTau(r.r.settle, tau)}</td>
            </tr>
          ))}
        </tbody>
      </table>
      </div>
      <p className={styles.note}>
        Change τ: the curves keep their shape, only the time axis stretches. The shape belongs to ζ
        and to the loop structure alone.
      </p>
    </div>
  );
}
