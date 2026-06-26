# OSPR-AI Monitor

A Python GUI that listens to your robot's data from two sources at once:

- **Big data straight from Gazebo** — camera images (left/right) and the laser
  scan, subscribed directly from the `gz-transport` topics (full rate, no UDP
  round-trip).
- **Everything else over UDP from the Rust program** — IMU (raw + an estimated
  attitude), GNSS, and per-motor feedback (setpoint, current value, limits,
  status, PID) decoded from the protobuf datagrams your Rust app already emits
  on its `core`/`extended` interfaces.

Everything is drawn in one live window: two camera panels, a top-down lidar
scatter, an artificial-horizon attitude indicator, IMU numerics + accel/gyro
time-series, a GNSS readout, and a motors table — plus per-stream Hz in the
status bar.

```
 Gazebo topics ─▶ GazeboReceiver ─┐
 (cameras, scan)                  ├─▶ SharedState ─▶ Qt GUI (30 Hz repaint)
 Rust UDP ──────▶ UDPReceiver ────┘
 (imu, gnss, motors, lidar)
```

## Files

| file | role |
|------|------|
| `config.py` | **start here** — ports, topics, Gazebo version, framing mode |
| `protocol.py` | pure-Python protobuf decoder + robomorph message schemas |
| `receivers.py` | UDP + Gazebo receiver threads |
| `state.py` | thread-safe shared snapshot |
| `attitude.py` | client-side complementary filter (raw IMU → roll/pitch/yaw) |
| `gz_bindings.py` | version-tolerant loader for `gz.transport*/gz.msgs*` |
| `gui.py` | the window + `main()` |

## Install

```bash
pip install -r requirements.txt
# Gazebo bindings come from your Gazebo install, NOT pip. For Harmonic:
sudo apt install python3-gz-transport13 python3-gz-msgs10
```

The app runs **without** the Gazebo bindings — the camera/lidar panels just show
"waiting" and you can still test the whole UDP path.

## Run

```bash
python gui.py
```

## Three things you'll probably need to tune

Everything below lives in `config.py` / `protocol.py` and is flagged in comments.

### 1. UDP type tagging (important)

Right now the Rust program sends raw `encode_to_vec()` bytes with **no type
tag**, so the `core` port carries GNSS, IMU and MotorFeedBack datagrams with
nothing to tell them apart. The app handles this with a heuristic classifier
(`UDP_TYPED_FRAMING = False`) that guesses the type from the protobuf field
layout — it works, but it's a guess.

The robust fix is one extra byte per message. On the Rust side, prefix each
`trig` with a type id:

```rust
// helper
fn framed(type_id: u8, mut bytes: Vec<u8>) -> Vec<u8> {
    let mut out = Vec::with_capacity(bytes.len() + 1);
    out.push(type_id);
    out.append(&mut bytes);
    out
}

// 0=GNSS 1=IMU 2=MOTOR 3=LIDAR  (must match config.py)
core_frame_publisher.trig(framed(0, state.gnss_measurements.encode_to_vec()));
core_frame_publisher.trig(framed(1, state.imu_measurements.encode_to_vec()));
for arm in &state.arms        { core_frame_publisher.trig(framed(2, arm.encode_to_vec())); }
for prop in &state.propellers { core_frame_publisher.trig(framed(2, prop.encode_to_vec())); }
extended_frame_publisher.trig(framed(3, state.lidar_measurements.encode_to_vec()));
```

Then set `UDP_TYPED_FRAMING = True` and the guessing disappears.

### 2. Protobuf field numbers

`protocol.py` infers the schema for `GNSSMeasurement`, `IMUMeasurements`,
`MotorFeedBack`, `LidarMeasurements` and `Ray` from the struct declaration
order in `main.rs` (fields 1, 2, 3… in order). If your robomorph
`#[prost(tag = "...")]` attributes use different numbers or integer types, edit
the `decode_*` functions — they're a few lines each. (Alternatively, point
`protoc` at the real `.proto` files and swap the decoders for generated
classes; the rest of the app only depends on the small dataclasses.)

### 3. Bind ports

`config.py` binds the ports the Rust program **sends to** (`8090` for core,
`9010` for extended), based on `UDPChannel::new_async(local, local_port,
remote, remote_port)`. If your `new_async` argument order differs, flip
`CORE_UDP_BIND` / `EXTENDED_UDP_BIND`.

## A note on IMU "attitude from the Kalman filter"

The `IMUMeasurements` message your Rust app currently sends contains only **raw**
data (accel xyz, gyro xyz, mag xyz) — there is no attitude/Kalman field in it.
So the attitude shown here is **estimated on the client** by a lightweight
complementary filter (`attitude.py`). If you add a real EKF/Kalman attitude
output to the Rust message, decode those fields in `protocol.Imu` and feed them
to the GUI directly instead of running the estimator (set
`ESTIMATE_ATTITUDE = False`).

## Lidar source

By default the GUI's lidar comes straight from the Gazebo `LaserScan` topic
(`LIDAR_SOURCE = "gazebo"`). Your Rust app also forwards a `LidarMeasurements`
message on the extended UDP port; set `LIDAR_SOURCE = "udp"` to drive the plot
from that instead (useful if you want to see exactly what the Rust side packs).

## Troubleshooting

- **"No Gazebo Python bindings found"** — install `python3-gz-transportXX`, or
  set `GZ_TRANSPORT_VERSION` in `config.py` to match your release.
- **Motors/IMU mislabeled with heuristic framing** — switch to typed framing
  (section 1). It removes all ambiguity.
- **Garbage values** — almost always a field-number mismatch (section 2).
- **Nothing on the UDP panels** — confirm the bind ports (section 3) and that
  the Rust program is actually publishing (`UDPChannel` remote = these ports).
