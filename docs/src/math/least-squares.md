# Constrained least squares

This is the mathematics behind [control allocation](../pipelines/motor-control/mixer.md): the vehicle needs one 6D wrench, it has `n` motors, and the map from motor commands to wrench is (locally) linear.

## The problem

```
find  dx ∈ ℝⁿ     minimising  ‖A·dx − b‖²     subject to   lo ≤ dx ≤ hi
```

- `A` is 6×n: column `a_j` is the wrench the vehicle gains per unit of command on motor `j`;
- `b` is the wrench that is **missing** (desired minus current);
- the bounds are what each motor can physically do.

Three cases, and all three occur on the same vehicle:

| | Meaning |
|---|---|
| `n < 6` or rank-deficient `A` | **under-actuated**: some demands are unreachable |
| `n > 6` with full row rank | **over-actuated**: infinitely many exact solutions, one must be chosen |
| bounds active | the unconstrained optimum is outside the feasible box |

## The geometry: why `Aᵀr = 0`, not `r = 0`

The reachable wrenches form the **column space** of `A` — the set of all combinations of the columns. If `b` lies outside it, no command whatsoever can produce it: the best possible answer is the **orthogonal projection** of `b` onto that space, and the leftover residual is unavoidable.

<figure>
<svg class="diagram" viewBox="0 0 700 290" xmlns="http://www.w3.org/2000/svg" role="img" aria-label="The demand b projected orthogonally onto the column space of A; the residual is perpendicular to the space and cannot be removed">
  <defs>
    <marker id="l-arw" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">
      <path d="M1 1 L9 5 L1 9" fill="none" stroke="currentColor" stroke-width="1.6" stroke-linecap="round" stroke-linejoin="round"/>
    </marker>
    <marker id="l-arw-b" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">
      <path d="M1 1 L9 5 L1 9" fill="none" stroke="#4a90d9" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round"/>
    </marker>
    <marker id="l-arw-c" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">
      <path d="M1 1 L9 5 L1 9" fill="none" stroke="#d4694a" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round"/>
    </marker>
    <marker id="l-arw-t" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">
      <path d="M1 1 L9 5 L1 9" fill="none" stroke="#2aa198" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round"/>
    </marker>
  </defs>
  <polygon points="120,240 470,240 580,150 230,150" fill="#2aa198" fill-opacity="0.10" stroke="#2aa198" stroke-opacity="0.55" stroke-width="1.2"/>
  <text x="140" y="262" font-size="11" fill="#2aa198">column space of A — reachable wrenches</text>
  <circle cx="250" cy="205" r="3.5" fill="currentColor"/>
  <line x1="250" y1="205" x2="380" y2="203" stroke="#2aa198" stroke-width="1.8" marker-end="url(#l-arw-t)"/>
  <text x="330" y="223" font-size="11" fill="#2aa198">a₁</text>
  <line x1="250" y1="205" x2="332" y2="163" stroke="#2aa198" stroke-width="1.8" marker-end="url(#l-arw-t)"/>
  <text x="292" y="158" font-size="11" fill="#2aa198">a₂</text>
  <line x1="250" y1="205" x2="470" y2="72" stroke="#4a90d9" stroke-width="2.2" marker-end="url(#l-arw-b)"/>
  <text x="392" y="112" font-size="12" font-weight="600" fill="#4a90d9">b</text>
  <text x="360" y="132" font-size="10" opacity="0.7">what is missing</text>
  <line x1="250" y1="205" x2="450" y2="184" stroke="currentColor" stroke-opacity="0.75" stroke-width="1.6" stroke-dasharray="5 3" marker-end="url(#l-arw)"/>
  <text x="368" y="200" font-size="11" opacity="0.85">A·dx</text>
  <line x1="455" y1="184" x2="470" y2="80" stroke="#d4694a" stroke-width="2" marker-end="url(#l-arw-c)"/>
  <text x="480" y="140" font-size="12" font-weight="600" fill="#d4694a">r</text>
  <text x="480" y="158" font-size="10" fill="#d4694a" opacity="0.85">unreachable</text>
  <path d="M441 182 L438 168 L452 166" fill="none" stroke="#d4694a" stroke-opacity="0.8" stroke-width="1.1"/>
  <text x="350" y="44" text-anchor="middle" font-size="11.5" opacity="0.8">the best command is the orthogonal projection — the residual r stays forever</text>
</svg>
<figcaption>Optimality is not <code>r = 0</code>; it is <code>r ⟂ span(A)</code>, i.e. <code>Aᵀr = 0</code>.</figcaption>
</figure>

Differentiating `‖A·dx − b‖²` gives the **normal equations**:

```
Aᵀ(A·dx − b) = 0        ⟺        Aᵀ·r = 0
```

The optimality condition is that the residual be **orthogonal to every column**, not that it be zero. The unreachable part of `b` lies in the null space of `Aᵀ`, so it satisfies `Aᵀr = 0` automatically and disappears from the criterion.

> This has a direct practical consequence. Judging convergence on `‖r‖` mixes the part that *can* still be served with the part that *never* can. On a bicopter tilted in roll, the body-frame gravity feedforward has a lateral component `m·g·sin φ` that no column can produce; it dominates `‖r‖`, never moves, and makes a relative-improvement stopping rule fire after a single sweep — leaving reachable moments under-served. Judging on `‖Aᵀr‖` removes it automatically.

## Why the rows must be normalised first

`b` mixes newtons and newton-metres, so `‖A·dx − b‖²` adds N² to (N·m)². The minimiser would change if lengths were expressed in millimetres — the objective is not physically meaningful until every row is brought to a common unit. Dividing the force rows by `m` and moment row `k` by `√(m·I_k)` expresses the residual as an **acceleration**. See [Rigid-body efforts](rigid-body.md).

## Projected coordinate descent

Rather than forming and inverting `AᵀA`, the solver sweeps the motors one at a time. Moving only motor `j` along its own column, the exact minimiser of the residual is:

```
score_j = a_j · r                    how much of what is missing this motor can produce (signed)
step_j  = 1 / ‖a_j‖²                 exact step along column j
dx_j   += score_j · step_j
r      ←  b − A·dx                   refreshed immediately
```

Each coordinate update can only lower the objective, so the sweep converges monotonically. Two rules make it correct:

- **the step is per column**, never global. With mixed units (N for a thruster, rad for a joint) a single step size is either divergent for the large columns or paralysing for the small ones. `1/‖a_j‖²` makes the solver invariant to the unit each motor is commanded in.
- **the residual is refreshed after every motor** (Gauss–Seidel), not once per sweep (Jacobi). Otherwise correlated columns — two rotors both pushing mostly along `+z`, about 83 % correlated on a bicopter — correct the same error twice and overshoot.

`score_j` is exactly the `j`-th entry of `Aᵀr`, which is why the stopping criterion mentioned above costs nothing: it is already computed.

### Degenerate columns

`step_j = 1/‖a_j‖²` explodes when a column vanishes. Two cases must be handled:

| Case | Symptom | Handling |
|---|---|---|
| exactly zero column | `step = ∞` | the motor sits out this cycle |
| **near**-zero column | `step` astronomically large (10¹¹⁷ observed), any residual noise saturates the increment | same treatment — the guard must be **relative**: `‖a_j‖ < 10⁻⁶ · max_k ‖a_k‖` |

The second case is not academic: a tilting arm whose rotor has just started spinning has a tiny but non-zero column, and the missing relative guard once flicked the arms by ±5° at every take-off.

## The bounds are part of the problem

After each coordinate step the increment is clipped to the feasible interval. Clipping is a **projection onto a convex box**, which preserves the descent property: the objective still cannot increase. Two kinds of bound:

- **physical limits** of the motor's own effort (`min_value` … `max_value`), applied to the *result* `current + dx`, hence the shift by the current value;
- a **trust region** for any motor whose column is a linearisation (a joint): the increment must stay inside the domain where the tangent still describes reality. `max_rot_speed · dt` — what the joint can physically travel in one cycle — is both a physical limit and the right trust region. A thruster needs none: its column is exact for any increment.

## Incremental formulation and re-linearisation

Because some columns are tangents, the problem is only valid near the current operating point. It is therefore re-linearised every cycle and solved for an **increment**:

```
A(x_k) · dx  ≈  b = w_desired − w_current(x_k)
```

`w_current` is rebuilt from the **measured** motor state, so the measurement is the linearisation point — a wrong feedback is a wrong plant model. This is a standard sequential-linear-programming structure: linearise, solve a bounded linear least-squares problem for a small step, apply, measure, repeat.

## Alternatives considered

| Method | Verdict |
|---|---|
| Projected coordinate descent (in use) | short loop, reuses the residual and the scores, converges monotonically, handles the bounds natively |
| BVLS (bounded-variable least squares) | same fixed point; kept in reserve if convergence proves too slow |
| Plain pseudo-inverse (SVD) + clamp | rejected: clamping after the fact does not redistribute effort among the remaining motors |
| Simulated annealing | rejected: the problem is convex, and non-determinism destroys debugging by correlation |
