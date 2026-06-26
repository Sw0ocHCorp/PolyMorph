"""
Central configuration for the OSPR-AI monitor.

Everything that depends on YOUR setup (ports, topic names, Gazebo version,
the robomorph protobuf schema) is gathered here so you don't have to dig
through the rest of the code.
"""

# ---------------------------------------------------------------------------
# UDP interfaces (the data coming FROM the Rust program)
# ---------------------------------------------------------------------------
# In the Rust code:
#     core_interface     = UDPChannel::new_async("127.0.0.1", 8080, "127.0.0.1", 8090)
#     extended_interface = UDPChannel::new_async("127.0.0.1", 9000, "127.0.0.1", 9010)
#
# new_async(local_ip, local_port, remote_ip, remote_port): the program SENDS to
# the *remote* address. So this Python listener must BIND the remote port.
#   - core   -> sends to 127.0.0.1:8090   (GNSS, IMU, motor feedback)
#   - extended -> sends to 127.0.0.1:9010 (lidar measurements)
#
# If your robomorph UDPChannel uses a different argument order, just flip these.
CORE_UDP_BIND = ("127.0.0.1", 8090)
EXTENDED_UDP_BIND = ("127.0.0.1", 9010)

# Each `event.trig(bytes)` is assumed to produce exactly one UDP datagram whose
# payload is the protobuf-encoded message (optionally prefixed by a type byte,
# see UDP_TYPED_FRAMING below). If robomorph adds its own framing/length prefix,
# adjust protocol.split_datagram().

# ---------------------------------------------------------------------------
# How UDP messages are tagged
# ---------------------------------------------------------------------------
# The Rust program currently sends raw `encode_to_vec()` bytes with NO type tag.
# That makes the core stream ambiguous (GNSS vs IMU vs MotorFeedBack all arrive
# on the same port with nothing to tell them apart).
#
#   * UDP_TYPED_FRAMING = True  -> expect a 1-byte type id prefix on every
#     datagram (recommended; see README for the ~3-line Rust change).
#   * UDP_TYPED_FRAMING = False -> fall back to a best-effort heuristic that
#     classifies each datagram by its protobuf field layout. Works, but is not
#     bullet-proof; prefer typed framing for anything real.
UDP_TYPED_FRAMING = True

# Type ids used when UDP_TYPED_FRAMING is True.
# MUST match the Rust `MessageType` enum exactly (same names -> same numbers).
MSG_IMU = 0      # ImuRawMessage
MSG_GNSS = 1     # GNSSRawMessage
MSG_MOTOR = 2    # MotorFeedBackMessage
MSG_POSE = 3     # PoseMessage        (no Python decoder yet -> ignored)
MSG_LIDAR = 4    # LidarMappingMessage

# ---------------------------------------------------------------------------
# Gazebo transport (the BIG data: lidar + camera images)
# ---------------------------------------------------------------------------
# The Python bindings are versioned to match your Gazebo release, e.g.
#   Harmonic -> gz.transport13 / gz.msgs10
#   Ionic    -> gz.transport14 / gz.msgs11
# Leave GZ_TRANSPORT_VERSION = None to auto-detect across known versions.
GZ_TRANSPORT_VERSION = None  # e.g. 13, 14, or None for auto

# If your system protobuf is newer than what gz-msgs was generated against you
# get "Descriptors cannot be created directly". Setting this True forces the
# pure-Python protobuf backend (slower, but no downgrade needed). The cleaner
# fix is `pip install 'protobuf<3.21'` in front of the interpreter you run.
FORCE_PURE_PYTHON_PROTOBUF = False

GZ_TOPIC_LIDAR = "osprai/lidar"
GZ_TOPIC_CAM_LEFT = "osprai/cameras/left"
GZ_TOPIC_CAM_RIGHT = "osprai/cameras/right"

# Where the lidar shown in the GUI comes from:
#   "gazebo" -> subscribe LaserScan directly from the gz topic (full rate/res)
#   "udp"    -> use the LidarMeasurements coming over the extended UDP channel
LIDAR_SOURCE = "gazebo"

# Lidar panel rendering:
#   "3d" -> interactive 3D point cloud (orbit/zoom with the mouse; needs PyOpenGL)
#   "2d" -> top-down scatter (no extra deps)
LIDAR_VIEW = "3d"

# For a 3D (multi-beam) lidar, how to draw it:
#   "all"    -> overlay every vertical beam, each projected onto the ground
#               plane (denser; walls from all beams stack on the same line)
#   "middle" -> only the most-horizontal beam (a clean single 2D ring)
LIDAR_BEAMS = "all"

# ---------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------
GUI_REFRESH_HZ = 30          # how often the GUI redraws from shared state
IMU_HISTORY = 300            # samples kept for the IMU time-series plots
MOTOR_HISTORY = 300          # samples kept for the arm/propeller plots
ESTIMATE_ATTITUDE = True     # client-side complementary filter (see attitude.py)