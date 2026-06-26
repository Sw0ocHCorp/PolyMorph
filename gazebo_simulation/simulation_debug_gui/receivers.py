"""
Receivers: background threads that pull data in and push it into SharedState.

  UDPReceiver      - binds a UDP port, decodes robomorph protobuf datagrams
                     (GNSS / IMU / Motor / Lidar) and routes them to the state.
  GazeboReceiver   - subscribes to the gz-transport topics for camera images
                     and the laser scan; projects the scan to top-down XY.

Both are designed to never crash the app: a decode error on one datagram is
logged and skipped.
"""
from __future__ import annotations

import socket
import threading

import numpy as np

import config
import protocol
from attitude import ComplementaryFilter
from state import SharedState


# ---------------------------------------------------------------------------
# UDP
# ---------------------------------------------------------------------------
class UDPReceiver(threading.Thread):
    def __init__(self, bind_addr: tuple[str, int], state: SharedState,
                 name: str, accept: set[str] | None = None):
        super().__init__(daemon=True, name=name)
        self.bind_addr = bind_addr
        self.state = state
        self.accept = accept            # if set, only handle these kinds
        self._stop = threading.Event()
        self._filter = ComplementaryFilter()
        self.sock: socket.socket | None = None

    def stop(self) -> None:
        self._stop.set()
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass

    def run(self) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock.bind(self.bind_addr)
        except OSError as e:
            print(f"[{self.name}] cannot bind {self.bind_addr}: {e}")
            return
        self.sock.settimeout(0.5)
        print(f"[{self.name}] listening on udp://{self.bind_addr[0]}:{self.bind_addr[1]}")

        while not self._stop.is_set():
            try:
                data, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            self._handle(data)

    def _handle(self, data: bytes) -> None:
        kind, payload = protocol.split_datagram(data)
        if kind is None or (self.accept and kind not in self.accept):
            return
        try:
            msg = protocol.DECODERS[kind](payload)
        except Exception as e:  # noqa: BLE001 - never let one bad packet kill us
            print(f"[{self.name}] decode error ({kind}): {e}")
            return

        if kind == "imu":
            att = self._filter.update(msg) if config.ESTIMATE_ATTITUDE else None
            self.state.set_imu(msg, att)
        elif kind == "gnss":
            self.state.set_gnss(msg)
        elif kind == "motor":
            self.state.set_motor(msg)
        elif kind == "lidar":
            self.state.set_lidar_udp(msg)
            if config.LIDAR_SOURCE == "udp":
                self._project_udp_lidar(msg)

    def _project_udp_lidar(self, lid: protocol.Lidar) -> None:
        if not lid.rays:
            return
        h = np.fromiter((r.horizontal_angle for r in lid.rays), dtype=float)
        v = np.fromiter((r.vertical_angle for r in lid.rays), dtype=float)
        rng = np.fromiter((r.length for r in lid.rays), dtype=float)
        good = np.isfinite(rng) & (rng > 0)
        h, v, rng = h[good], v[good], rng[good]
        horiz = rng * np.cos(v)
        xyz = np.column_stack((horiz * np.cos(h), horiz * np.sin(h), rng * np.sin(v)))
        self.state.set_lidar_points(xyz, rng)


def start_udp_receivers(state: SharedState) -> list[UDPReceiver]:
    """Core port: GNSS/IMU/Motor. Extended port: Lidar."""
    recvs = [
        UDPReceiver(config.CORE_UDP_BIND, state, "core-udp",
                    accept={"gnss", "imu", "motor"}),
        UDPReceiver(config.EXTENDED_UDP_BIND, state, "ext-udp",
                    accept={"lidar"}),
    ]
    for r in recvs:
        r.start()
    return recvs


# ---------------------------------------------------------------------------
# Gazebo transport
# ---------------------------------------------------------------------------
_PIXEL_CHANNELS = {
    # gz.msgs.PixelFormatType -> channel count (common ones)
    1: 1,   # L_INT8
    2: 1,   # L_INT16
    3: 3,   # RGB_INT8
    6: 3,   # BGR_INT8
    7: 4,   # RGBA_INT8 (treated as 4)
}


class GazeboReceiver:
    """
    Subscribes to the camera + lidar gz topics. Runs its own background thread
    (gz-transport delivers callbacks on its own threads anyway, but we keep a
    thread alive so the node isn't garbage-collected).
    """

    def __init__(self, state: SharedState):
        self.state = state
        self.node = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self.available = False
        self.error: str | None = None
        self._scan_logged = False

    def start(self) -> None:
        try:
            from gz_bindings import load_gz, GzUnavailable
            try:
                gz = load_gz()
            except GzUnavailable as e:
                self.error = str(e)
                print(f"[gazebo] {e}")
                return
        except Exception as e:  # noqa: BLE001
            self.error = str(e)
            print(f"[gazebo] {e}")
            return

        Image = gz.msg("image", "Image")
        LaserScan = gz.msg("laserscan", "LaserScan")

        self.node = gz.Node()
        ok_l = self.node.subscribe(Image, config.GZ_TOPIC_CAM_LEFT,
                                   self._make_cam_cb("left"))
        ok_r = self.node.subscribe(Image, config.GZ_TOPIC_CAM_RIGHT,
                                   self._make_cam_cb("right"))
        ok_s = True
        if config.LIDAR_SOURCE == "gazebo":
            ok_s = self.node.subscribe(LaserScan, config.GZ_TOPIC_LIDAR,
                                       self._scan_cb)
        self.available = bool(ok_l or ok_r or ok_s)
        print(f"[gazebo] subscribed cams(L={ok_l},R={ok_r}) scan={ok_s}")

        self._thread = threading.Thread(target=self._spin, daemon=True,
                                        name="gz-spin")
        self._thread.start()

    def _spin(self) -> None:
        # Keep the node alive; callbacks fire on gz-transport's own threads.
        self._stop.wait()

    def stop(self) -> None:
        self._stop.set()

    # -- callbacks ---------------------------------------------------------
    def _make_cam_cb(self, side: str):
        def cb(msg) -> None:
            try:
                frame = self._image_to_numpy(msg)
            except Exception as e:  # noqa: BLE001
                print(f"[gazebo] {side} image decode error: {e}")
                return
            if frame is not None:
                self.state.set_camera(side, frame)
        return cb

    @staticmethod
    def _image_to_numpy(msg):
        w, h = int(msg.width), int(msg.height)
        if w == 0 or h == 0 or not msg.data:
            return None
        ch = _PIXEL_CHANNELS.get(int(getattr(msg, "pixel_format_type", 3)), 3)
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        expected = w * h * ch
        if buf.size < expected:
            return None
        frame = buf[:expected].reshape(h, w, ch)
        if ch == 1:
            frame = np.repeat(frame, 3, axis=2)
        elif ch == 4:
            frame = frame[:, :, :3]
        return np.ascontiguousarray(frame)

    def _scan_cb(self, msg) -> None:
        try:
            ranges = np.asarray(msg.ranges, dtype=float)
            if ranges.size == 0:
                return
            a_min = float(msg.angle_min)
            a_step = float(msg.angle_step)
            v_count = int(getattr(msg, "vertical_count", 1) or 1)
            h_count = int(getattr(msg, "count", 0) or 0)
            if h_count <= 0:
                h_count = ranges.size // max(v_count, 1)

            # gz LaserScan layout is ranges[v * h_count + h] -- the HORIZONTAL
            # index varies fastest. So reshape to (vertical, horizontal).
            if h_count > 0 and ranges.size == v_count * h_count:
                grid = ranges.reshape(v_count, h_count)
            else:                               # not a clean grid -> one ring
                grid = ranges.reshape(1, -1)
                v_count, h_count = 1, grid.shape[1]

            v_min = float(getattr(msg, "vertical_angle_min", 0.0))
            v_step = float(getattr(msg, "vertical_angle_step", 0.0))
            v_angles = v_min + v_step * np.arange(v_count)
            h_angles = a_min + a_step * np.arange(h_count)

            if not self._scan_logged:
                self._scan_logged = True
                print(f"[gazebo] scan: {v_count}x{h_count} "
                      f"(h {np.degrees(a_min):.0f}..{np.degrees(a_min+a_step*(h_count-1)):.0f} deg, "
                      f"v {np.degrees(v_min):.0f}..{np.degrees(v_angles[-1]):.0f} deg), "
                      f"{ranges.size} ranges, beams='{config.LIDAR_BEAMS}'")

            # pick one horizontal ring, or keep all beams
            if config.LIDAR_BEAMS == "middle" and v_count > 1:
                mid = v_count // 2
                grid = grid[mid:mid + 1]
                v_angles = v_angles[mid:mid + 1]

            rmin = float(getattr(msg, "range_min", 0.0))
            rmax = float(getattr(msg, "range_max", 0.0)) or np.inf

            H, V = np.meshgrid(h_angles, v_angles)          # shape (v, h)
            R = grid
            good = (np.isfinite(R) & (R > max(rmin, 1e-3)) & (R < rmax * 0.999))
            # mask first, then project (avoids arithmetic on inf no-returns)
            Rg, Hg, Vg = R[good], H[good], V[good]
            # spherical -> cartesian, z up (full 3D point cloud)
            horiz = Rg * np.cos(Vg)
            x = horiz * np.cos(Hg)
            y = horiz * np.sin(Hg)
            z = Rg * np.sin(Vg)
            self.state.set_lidar_points(np.column_stack((x, y, z)), Rg)
        except Exception as e:  # noqa: BLE001
            print(f"[gazebo] scan error: {e}")