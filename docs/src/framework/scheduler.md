# The scheduler

File: `robomorph/src/core/scheduler.rs`.

## The `Process` trait

A `Process` is the unit of scheduling. Its core method is:

```rust
fn exec(&mut self, input: &Option<AnyMessage>, dt: Duration) -> Option<AnyMessage>;
```

- `input` is the **pipe**: the value returned by the previous process of the same chain, in the same pass.
- The returned value becomes the `input` of the next process. `Some(...)` hands something over; `None` means *nothing to hand over* — an explicit abstention.
- `dt` currently receives the process's nominal **period**, not the measured elapsed time.

The remaining methods (`set_receiver`, `set_sender`, `set_period_from_freq`, `get_period`, `set_name`, `get_name`) are wiring.

## One pass of a chain

<figure>
<svg class="diagram" viewBox="0 0 700 250" xmlns="http://www.w3.org/2000/svg" role="img" aria-label="One scheduler pass: four processes executed in registration order, connected by the pipe, with broadcast channels underneath">
  <defs>
    <marker id="s-arw" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">
      <path d="M1 1 L9 5 L1 9" fill="none" stroke="currentColor" stroke-width="1.6" stroke-linecap="round" stroke-linejoin="round"/>
    </marker>
    <marker id="s-arw-d" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">
      <path d="M1 1 L9 5 L1 9" fill="none" stroke="#4a90d9" stroke-width="1.6" stroke-linecap="round" stroke-linejoin="round"/>
    </marker>
  </defs>
  <text x="52" y="26" font-size="11.5" opacity="0.75">one pass — every process due, in registration order</text>
  <rect x="52" y="40" width="132" height="60" rx="5" fill="currentColor" fill-opacity="0.06" stroke="#cb8b1e" stroke-width="1.2"/>
  <text x="118" y="66" text-anchor="middle" font-size="12" font-weight="600">osprai</text>
  <text x="118" y="84" text-anchor="middle" font-size="10.5" opacity="0.75">publish pose</text>
  <rect x="200" y="40" width="112" height="60" rx="5" fill="currentColor" fill-opacity="0.06" stroke="currentColor" stroke-opacity="0.35"/>
  <text x="256" y="66" text-anchor="middle" font-size="12" font-weight="600">remote</text>
  <text x="256" y="84" text-anchor="middle" font-size="10.5" opacity="0.75">poll gamepad</text>
  <rect x="328" y="40" width="132" height="60" rx="5" fill="currentColor" fill-opacity="0.06" stroke="#4a90d9" stroke-width="1.2"/>
  <text x="394" y="66" text-anchor="middle" font-size="12" font-weight="600">attitude</text>
  <text x="394" y="84" text-anchor="middle" font-size="10.5" opacity="0.75">law → wrench</text>
  <rect x="476" y="40" width="132" height="60" rx="5" fill="currentColor" fill-opacity="0.06" stroke="#2aa198" stroke-width="1.2"/>
  <text x="542" y="66" text-anchor="middle" font-size="12" font-weight="600">mixer</text>
  <text x="542" y="84" text-anchor="middle" font-size="10.5" opacity="0.75">allocation</text>
  <line x1="184" y1="70" x2="196" y2="70" stroke="currentColor" stroke-width="1.3" marker-end="url(#s-arw)"/>
  <line x1="312" y1="70" x2="324" y2="70" stroke="currentColor" stroke-width="1.3" marker-end="url(#s-arw)"/>
  <line x1="460" y1="70" x2="472" y2="70" stroke="#4a90d9" stroke-width="1.6" marker-end="url(#s-arw-d)"/>
  <text x="468" y="30" text-anchor="middle" font-size="10.5" fill="#4a90d9">pipe: wrench</text>
  <path d="M118 100 L118 140 L388 140" fill="none" stroke="currentColor" stroke-width="1.2" stroke-dasharray="5 4" marker-end="url(#s-arw)"/>
  <text x="150" y="156" font-size="10.5" opacity="0.75">telemetry channel — Pose (q, ω)</text>
  <path d="M542 100 L542 190 L112 190 L112 178" fill="none" stroke="currentColor" stroke-width="1.2" stroke-dasharray="5 4" marker-end="url(#s-arw)"/>
  <text x="330" y="206" text-anchor="middle" font-size="10.5" opacity="0.75">command channel — applied on the next pass (one-cycle actuation delay)</text>
  <line x1="52" y1="230" x2="608" y2="230" stroke="currentColor" stroke-opacity="0.4" stroke-width="1"/>
  <line x1="52" y1="224" x2="52" y2="236" stroke="currentColor" stroke-opacity="0.6" stroke-width="1.2"/>
  <line x1="608" y1="224" x2="608" y2="236" stroke="currentColor" stroke-opacity="0.6" stroke-width="1.2"/>
  <text x="330" y="246" text-anchor="middle" font-size="10.5" opacity="0.7">period T = 10 ms at 100 Hz — then sleep until the next due instant</text>
</svg>
<figcaption>Registration order is execution order. The pipe only survives within a pass.</figcaption>
</figure>

## `ProcessesChain`

A chain is an ordered list of `(process, next due instant)`. `run_once`:

1. computes `end_instant`, the latest of the `instant + period` deadlines;
2. loops until `end_instant` is passed: on each pass it walks the processes **in registration order**, executes those whose deadline has come (propagating the pipe), advances their deadline by one period, then sleeps until the nearest deadline.

If a process is late (its next deadline is already in the past after it ran), its deadline is **reset to now**.

### Three properties to know

> **1. `input_state` is local to one iteration of the wait loop.** It is reset to `None` on every iteration and every call to `run_once`. The pipe is therefore only reliable when producer and consumer run **in the same pass**: same chain, same period, registered in order. Making the pipe state a field of `ProcessesChain` would make it reliable in every case.

> **2. Resetting a late process's deadline to *now* shifts its phase** relative to the others. If a pass overruns its period (a very verbose mixer, for example), deadlines can reorder and the pipe breaks. Hence the rule: verbose output behind a flag, and watch for overruns.

> **3. An empty chain spins forever**: `run_finished` is only set inside the `for` loop over the processes. Never start a chain with no process in it.

## `Scheduler`

- `register_process` → **main chain**, run on the caller's thread by `run_main_chain()` (the binary calls it in a `loop`).
- `register_side_process(p, id)` → **side chain** `id`, run on a dedicated thread by `start_all_side_chains()`.
- `register_interface` / `start_all_interfaces` → `HardwareInterface`s are *connected* (they own their threads); they are not scheduled.

### Architectural rule: one chain, one clock

A producer and its consumer must live **in the same chain**. Two chains at the same nominal frequency on two threads drift in phase and produce empty and double ticks — a beat, measured at 30 empty / 30 double ticks out of 454 before the pose producer joined the controllers' chain. Side chains are for decoupled work (heavy perception), not for the links of a control loop.

## Ordering and loop delay

The order `osprai → attitude → mixer` makes the attitude stage consume a **fresh** measurement, published moments earlier in the same pass, and applies the mixer's commands **on the next pass**. Every discrete controller pays a computation delay somewhere; placing it on actuation rather than on measurement is the right choice for a critical loop, and it is covered by the `τ ≥ 10·T` margin.
