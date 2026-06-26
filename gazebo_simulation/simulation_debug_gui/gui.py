"""
OSPR-AI live monitor GUI.

Panels
------
  * two camera views (left/right)        <- gz topics
  * top-down lidar scatter               <- gz topic (or UDP, per config)
  * artificial-horizon attitude          <- estimated from raw IMU
  * IMU numerics + accel/gyro time-series <- UDP
  * GNSS readout                          <- UDP
  * motors table (id, type, setpoint,     <- UDP
    current, limits, status, PID)
  * status bar with per-stream rates

Uses pyqtgraph's Qt abstraction so it runs on PyQt5/PyQt6/PySide2/PySide6.
The GUI thread only reads SharedState via a periodic timer; all network I/O
happens in the receiver threads.
"""
from __future__ import annotations

import argparse
import faulthandler
import logging
import math
import os
import sys
import time

import config

# Must be set before gz.msgs (and thus google.protobuf) is ever imported.
if config.FORCE_PURE_PYTHON_PROTOBUF:
    os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

log = logging.getLogger("osprai.gui")

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

try:
    import pyqtgraph.opengl as gl
    _GL_OK = True
    _GL_ERR = None
except Exception as e:  # noqa: BLE001 - PyOpenGL missing or no GL context
    gl = None
    _GL_OK = False
    _GL_ERR = str(e)

import protocol
from receivers import GazeboReceiver, start_udp_receivers
from state import SharedState

pg.setConfigOptions(imageAxisOrder="row-major", antialias=True,
                    background="#101418", foreground="#cdd6e0")


# ---------------------------------------------------------------------------
# Camera view
# ---------------------------------------------------------------------------
class CameraView(QtWidgets.QWidget):
    def __init__(self, title: str):
        super().__init__()
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(2, 2, 2, 2)
        self.label = QtWidgets.QLabel(title)
        self.label.setStyleSheet("color:#9fb3c8;font-weight:bold;")
        lay.addWidget(self.label)

        self.glw = pg.GraphicsLayoutWidget()
        self.view = self.glw.addViewBox()
        self.view.setAspectLocked(True)
        self.view.invertY(True)
        self.img = pg.ImageItem()
        self.view.addItem(self.img)
        lay.addWidget(self.glw, 1)

    def update_frame(self, frame, hz: float):
        if frame is not None:
            self.img.setImage(frame, levels=(0, 255), autoLevels=False)
        base = self.label.text().split("  (")[0]
        self.label.setText(f"{base}  ({hz:0.0f} Hz)")


# ---------------------------------------------------------------------------
# Lidar views (2D top-down and 3D point cloud)
# ---------------------------------------------------------------------------
class LidarView(pg.PlotWidget):
    """Top-down scatter. Accepts Nx3 points and uses the x,y columns."""

    def __init__(self):
        super().__init__()
        self.setAspectLocked(True)
        self.showGrid(x=True, y=True, alpha=0.2)
        self.setLabel("bottom", "x (m)")
        self.setLabel("left", "y (m)")
        self.scatter = pg.ScatterPlotItem(size=3, pen=None)
        self.addItem(self.scatter)
        # robot origin marker
        self.addItem(pg.ScatterPlotItem(
            x=[0], y=[0], size=10, symbol="+",
            pen=pg.mkPen("#ff6b6b", width=2)))
        self._cmap = pg.colormap.get("viridis")

    def update_points(self, xyz, rng):
        if xyz is None or len(xyz) == 0:
            self.scatter.setData([], [])
            return
        rmax = float(np.nanmax(rng)) if rng is not None and len(rng) else 1.0
        rmax = rmax if rmax > 0 else 1.0
        colors = self._cmap.map(np.clip(rng / rmax, 0, 1), mode="qcolor") \
            if rng is not None else "w"
        brushes = [pg.mkBrush(c) for c in colors] if rng is not None else None
        self.scatter.setData(x=xyz[:, 0], y=xyz[:, 1], brush=brushes)


if _GL_OK:
    class LidarView3D(gl.GLViewWidget):
        """Interactive 3D point cloud (drag to orbit, scroll to zoom).
        Points are coloured by height (z) so vertical structure stands out."""

        def __init__(self):
            super().__init__()
            self.setBackgroundColor("#101418")
            self.setCameraPosition(distance=25, elevation=22, azimuth=-60)

            grid = gl.GLGridItem()
            grid.setSize(60, 60)
            grid.setSpacing(2, 2)
            grid.setColor((90, 100, 120, 110))
            self.addItem(grid)

            # x (red) and y (green) axes for orientation
            for vec, col in (((6, 0, 0), (1, 0.3, 0.3, 1)),
                             ((0, 6, 0), (0.3, 1, 0.3, 1))):
                ax = gl.GLLinePlotItem(
                    pos=np.array([[0, 0, 0], vec], dtype=float),
                    color=col, width=2, antialias=True)
                self.addItem(ax)

            self.cloud = gl.GLScatterPlotItem(
                pos=np.zeros((1, 3)), size=3.0, pxMode=True)
            self.addItem(self.cloud)
            self.origin = gl.GLScatterPlotItem(
                pos=np.zeros((1, 3)), size=12, color=(1, 0.3, 0.3, 1),
                pxMode=True)
            self.addItem(self.origin)
            self._cmap = pg.colormap.get("viridis")

        def update_points(self, xyz, rng):
            if xyz is None or len(xyz) == 0:
                self.cloud.setData(pos=np.zeros((0, 3)))
                return
            z = xyz[:, 2]
            lo, hi = float(np.min(z)), float(np.max(z))
            span = hi - lo
            t = (z - lo) / span if span > 1e-6 else np.zeros_like(z)
            colors = self._cmap.map(t, mode="float")     # Nx4 floats 0..1
            self.cloud.setData(pos=np.ascontiguousarray(xyz, dtype=np.float32),
                               color=colors, size=3.0, pxMode=True)


# ---------------------------------------------------------------------------
# Artificial horizon (attitude)
# ---------------------------------------------------------------------------
class AttitudeIndicator(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(160, 160)
        self.roll = self.pitch = self.yaw = 0.0

    def set_attitude(self, roll, pitch, yaw):
        self.roll, self.pitch, self.yaw = roll, pitch, yaw
        self.update()

    def paintEvent(self, _):
        w, h = self.width(), self.height()
        r = min(w, h) / 2 - 6
        cx, cy = w / 2, h / 2
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)

        # clip to circle
        path = QtGui.QPainterPath()
        path.addEllipse(QtCore.QRectF(cx - r, cy - r, 2 * r, 2 * r))
        p.setClipPath(path)

        p.translate(cx, cy)
        p.rotate(math.degrees(self.roll))
        pitch_px = max(min(self.pitch, math.pi / 3), -math.pi / 3) / (math.pi / 3) * r
        p.translate(0, pitch_px)

        # sky / ground
        p.fillRect(QtCore.QRectF(-2 * r, -3 * r, 4 * r, 3 * r), QtGui.QColor("#3a7bd5"))
        p.fillRect(QtCore.QRectF(-2 * r, 0, 4 * r, 3 * r), QtGui.QColor("#7a5230"))
        pen = QtGui.QPen(QtGui.QColor("#e8eef5"), 2)
        p.setPen(pen)
        p.drawLine(QtCore.QPointF(-2 * r, 0), QtCore.QPointF(2 * r, 0))

        p.resetTransform()
        # fixed aircraft reference
        p.setPen(QtGui.QPen(QtGui.QColor("#ffd166"), 3))
        p.drawLine(QtCore.QPointF(cx - r * 0.6, cy), QtCore.QPointF(cx - r * 0.2, cy))
        p.drawLine(QtCore.QPointF(cx + r * 0.2, cy), QtCore.QPointF(cx + r * 0.6, cy))
        p.drawPoint(QtCore.QPointF(cx, cy))
        # ring
        p.setClipping(False)
        p.setPen(QtGui.QPen(QtGui.QColor("#5a6b7b"), 2))
        p.drawEllipse(QtCore.QRectF(cx - r, cy - r, 2 * r, 2 * r))
        p.end()


# ---------------------------------------------------------------------------
# Helper: labelled numeric grid
# ---------------------------------------------------------------------------
class ValueGrid(QtWidgets.QWidget):
    def __init__(self, rows: list[str]):
        super().__init__()
        self.grid = QtWidgets.QGridLayout(self)
        self.grid.setContentsMargins(4, 4, 4, 4)
        self.grid.setVerticalSpacing(2)
        self.values: dict[str, QtWidgets.QLabel] = {}
        for i, name in enumerate(rows):
            k = QtWidgets.QLabel(name)
            k.setStyleSheet("color:#8a9bb0;")
            v = QtWidgets.QLabel("--")
            v.setStyleSheet("color:#e8eef5;font-family:monospace;")
            v.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight)
            self.grid.addWidget(k, i, 0)
            self.grid.addWidget(v, i, 1)
            self.values[name] = v

    def set(self, name: str, text: str):
        self.values[name].setText(text)


def _section(title: str, widget: QtWidgets.QWidget) -> QtWidgets.QGroupBox:
    box = QtWidgets.QGroupBox(title)
    box.setStyleSheet(
        "QGroupBox{color:#9fb3c8;font-weight:bold;border:1px solid #2a3340;"
        "border-radius:6px;margin-top:8px;}"
        "QGroupBox::title{subcontrol-origin:margin;left:8px;padding:0 4px;}")
    lay = QtWidgets.QVBoxLayout(box)
    lay.setContentsMargins(6, 14, 6, 6)
    lay.addWidget(widget)
    return box


def _legend_html(series) -> str:
    """series: list of (label, color) -> a one-line colored-swatch legend."""
    parts = [f'<span style="color:{c}">&#9644;</span>'
             f'<span style="color:#cdd6e0"> {lbl}</span>' for lbl, c in series]
    return "&nbsp;&nbsp;&nbsp;".join(parts)


def _legend_label(series=None) -> QtWidgets.QLabel:
    lbl = QtWidgets.QLabel(_legend_html(series) if series else "")
    lbl.setTextFormat(QtCore.Qt.TextFormat.RichText)
    lbl.setStyleSheet("font-size:13px;")
    lbl.setWordWrap(True)
    return lbl


def _section_with_legend(title, plot, legend) -> QtWidgets.QGroupBox:
    """Like _section, but stacks an external legend strip above the plot."""
    box = QtWidgets.QGroupBox(title)
    box.setStyleSheet(
        "QGroupBox{color:#9fb3c8;font-weight:bold;border:1px solid #2a3340;"
        "border-radius:6px;margin-top:8px;}"
        "QGroupBox::title{subcontrol-origin:margin;left:8px;padding:0 4px;}")
    lay = QtWidgets.QVBoxLayout(box)
    lay.setContentsMargins(6, 14, 6, 6)
    lay.setSpacing(2)
    lay.addWidget(legend)
    lay.addWidget(plot)
    return box


# ---------------------------------------------------------------------------
# Motor time-series (arms / propellers): solid = current, dashed = setpoint
# ---------------------------------------------------------------------------
class MotorChart(pg.PlotWidget):
    _PALETTE = ["#ff6b6b", "#4ecdc4", "#ffe66d", "#a78bfa",
                "#5ad6ff", "#ff9f43", "#9fff5a", "#ff6bd6"]

    def __init__(self, ylabel: str, scale: float = 1.0, y_floor: float = 1.0):
        super().__init__()
        self.setMaximumHeight(150)
        self.showGrid(x=False, y=True, alpha=0.2)
        self.setLabel("left", ylabel)
        # don't let autorange zoom closer than y_floor units -> no noise blow-up
        self.getViewBox().setLimits(minYRange=y_floor)
        self.scale = scale
        self._curves: dict[int, tuple] = {}   # id -> (current_curve, setpoint_curve)

    def _ensure(self, mid: int):
        if mid not in self._curves:
            col = self._PALETTE[len(self._curves) % len(self._PALETTE)]
            cur = self.plot(pen=pg.mkPen(col, width=2))
            sp = self.plot(pen=pg.mkPen(col, width=1,
                                        style=QtCore.Qt.PenStyle.DashLine))
            self._curves[mid] = (cur, sp)
        return self._curves[mid]

    def update_motors(self, ids, hist: dict):
        for mid in ids:
            arr = hist.get(mid)
            if arr is None or len(arr) == 0:
                continue
            cur, sp = self._ensure(mid)
            cur.setData(arr[:, 0] * self.scale)
            sp.setData(arr[:, 1] * self.scale)


# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------
class MonitorWindow(QtWidgets.QMainWindow):
    def __init__(self, state: SharedState, gz_error: str | None):
        super().__init__()
        self.state = state
        self.setWindowTitle("OSPR-AI Monitor")
        self.resize(1500, 900)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        self.setCentralWidget(splitter)

        # --- left: cameras ---
        cams = QtWidgets.QWidget()
        clay = QtWidgets.QVBoxLayout(cams)
        self.cam_left = CameraView("Camera Left")
        self.cam_right = CameraView("Camera Right")
        clay.addWidget(self.cam_left, 1)
        clay.addWidget(self.cam_right, 1)
        if gz_error:
            warn = QtWidgets.QLabel("Gazebo bindings unavailable:\n" + gz_error)
            warn.setWordWrap(True)
            warn.setStyleSheet("color:#ff9f43;font-size:10px;")
            clay.addWidget(warn)
        splitter.addWidget(cams)

        # --- middle: lidar ---
        if config.LIDAR_VIEW == "3d" and _GL_OK:
            self.lidar = LidarView3D()
            lidar_title = "Lidar (3D — drag to orbit, scroll to zoom)"
        else:
            self.lidar = LidarView()
            lidar_title = "Lidar (top-down)"
            if config.LIDAR_VIEW == "3d" and not _GL_OK:
                print(f"[gui] 3D lidar unavailable ({_GL_ERR}); using 2D. "
                      f"Install PyOpenGL for the 3D cloud.")
        splitter.addWidget(_section(lidar_title, self.lidar))

        # --- right: telemetry ---
        right = QtWidgets.QWidget()
        rlay = QtWidgets.QVBoxLayout(right)

        self.attitude = AttitudeIndicator()
        self.att_vals = ValueGrid(["roll", "pitch", "yaw"])
        att_row = QtWidgets.QWidget()
        al = QtWidgets.QHBoxLayout(att_row)
        al.addWidget(self.attitude, 1)
        al.addWidget(self.att_vals, 1)
        rlay.addWidget(_section("Attitude (estimated)", att_row))

        self.imu_vals = ValueGrid(["accel x", "accel y", "accel z",
                                   "gyro x", "gyro y", "gyro z",
                                   "mag x", "mag y", "mag z"])
        rlay.addWidget(_section("IMU (raw)", self.imu_vals))

        xyz_series = [("x", "#ff6b6b"), ("y", "#4ecdc4"), ("z", "#ffe66d")]
        self.accel_plot = self._series_plot(xyz_series)
        self.gyro_plot = self._series_plot(xyz_series)
        rlay.addWidget(_section_with_legend("Accel x/y/z", self.accel_plot,
                                            _legend_label(xyz_series)))
        rlay.addWidget(_section_with_legend("Gyro x/y/z", self.gyro_plot,
                                            _legend_label(xyz_series)))

        self.gnss_vals = ValueGrid(["latitude", "longitude", "altitude", "fix"])
        rlay.addWidget(_section("GNSS", self.gnss_vals))

        # arm position + propeller velocity time-series (solid=current, dashed=setpoint)
        self.arms_plot = MotorChart("deg", scale=180.0 / math.pi)
        self.props_plot = MotorChart("rad/s")
        rlay.addWidget(_section(
            "Arms — position  (solid=current, dashed=setpoint)", self.arms_plot))
        rlay.addWidget(_section(
            "Propellers — velocity  (solid=current, dashed=setpoint)", self.props_plot))

        right_scroll = QtWidgets.QScrollArea()
        right_scroll.setWidgetResizable(True)
        right_scroll.setWidget(right)
        right_scroll.setMinimumWidth(360)
        splitter.addWidget(right_scroll)

        splitter.setSizes([420, 600, 420])

        # --- motors table along the bottom ---
        self.motor_table = QtWidgets.QTableWidget(0, 9)
        self.motor_table.setHorizontalHeaderLabels(
            ["id", "type", "setpoint", "current", "min", "max",
             "status", "P/I/D", "Δ err"])
        self.motor_table.horizontalHeader().setStretchLastSection(True)
        self.motor_table.setMaximumHeight(180)
        dock = QtWidgets.QDockWidget("Motors (arms + propellers)")
        dock.setWidget(self.motor_table)
        self.addDockWidget(QtCore.Qt.DockWidgetArea.BottomDockWidgetArea, dock)

        self.status = self.statusBar()

        framing = "typed" if config.UDP_TYPED_FRAMING else "heuristic"
        self._status_prefix = f"UDP framing: {framing} | lidar: {config.LIDAR_SOURCE} | "

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(int(1000 / config.GUI_REFRESH_HZ))
        self._dbg_last = 0.0

    def _series_plot(self, series):
        pw = pg.PlotWidget()
        pw.setMaximumHeight(120)
        pw.showGrid(x=False, y=True, alpha=0.2)
        pw.curves = [pw.plot(pen=pg.mkPen(c, width=1)) for lbl, c in series]
        return pw

    # ---- periodic refresh -------------------------------------------------
    def refresh(self):
        s = self.state.snapshot()
        rates = s["rates"]

        self.cam_left.update_frame(s["cam_left"], rates["cam_left"])
        self.cam_right.update_frame(s["cam_right"], rates["cam_right"])
        self.lidar.update_points(s["lidar_xyz"], s["lidar_range"])

        roll, pitch, yaw = s["attitude"]
        self.attitude.set_attitude(roll, pitch, yaw)
        self.att_vals.set("roll", f"{math.degrees(roll):7.2f}°")
        self.att_vals.set("pitch", f"{math.degrees(pitch):7.2f}°")
        self.att_vals.set("yaw", f"{math.degrees(yaw):7.2f}°")

        imu = s["imu"]
        self.imu_vals.set("accel x", f"{imu.l_accel_x:8.3f}")
        self.imu_vals.set("accel y", f"{imu.l_accel_y:8.3f}")
        self.imu_vals.set("accel z", f"{imu.l_accel_z:8.3f}")
        self.imu_vals.set("gyro x", f"{imu.a_velocity_x:8.3f}")
        self.imu_vals.set("gyro y", f"{imu.a_velocity_y:8.3f}")
        self.imu_vals.set("gyro z", f"{imu.a_velocity_z:8.3f}")
        self.imu_vals.set("mag x", f"{imu.magnetic_field_x:8.3f}")
        self.imu_vals.set("mag y", f"{imu.magnetic_field_y:8.3f}")
        self.imu_vals.set("mag z", f"{imu.magnetic_field_z:8.3f}")

        if s["accel_hist"] is not None:
            a = s["accel_hist"]
            for i, c in enumerate(self.accel_plot.curves):
                c.setData(a[:, i])
        if s["gyro_hist"] is not None:
            g = s["gyro_hist"]
            for i, c in enumerate(self.gyro_plot.curves):
                c.setData(g[:, i])

        gn = s["gnss"]
        self.gnss_vals.set("latitude", f"{gn.latitude:.7f}")
        self.gnss_vals.set("longitude", f"{gn.longitude:.7f}")
        self.gnss_vals.set("altitude", f"{gn.altitude:.2f} m")
        self.gnss_vals.set("fix", str(gn.fix_status))

        self._update_motors(s["motors"])

        # route motors to the two charts: VELOCITY -> propellers, rest -> arms
        motors, mhist = s["motors"], s["motor_hist"]
        prop_ids = sorted(i for i, m in motors.items()
                          if protocol.COMMAND_TYPE.get(m.command_type) == "VELOCITY")
        arm_ids = sorted(i for i in motors if i not in set(prop_ids))
        self.arms_plot.update_motors(arm_ids, mhist)
        self.props_plot.update_motors(prop_ids, mhist)

        self.status.showMessage(
            self._status_prefix +
            f"IMU {rates['imu']:.0f}Hz  GNSS {rates['gnss']:.0f}Hz  "
            f"motor {rates['motor']:.0f}Hz  lidar {rates['lidar']:.0f}Hz")

        # once-a-second diagnostics when --debug is on
        if log.isEnabledFor(logging.DEBUG):
            now = time.monotonic()
            if now - self._dbg_last >= 1.0:
                self._dbg_last = now
                log.debug("rates Hz: %s",
                          " ".join(f"{k}={v:.0f}" for k, v in rates.items()))
                log.debug("motors=%s arms=%s props=%s | cams L/R=%s/%s | lidar_pts=%d",
                          sorted(motors), arm_ids, prop_ids,
                          s["cam_left"] is not None, s["cam_right"] is not None,
                          0 if s["lidar_xyz"] is None else len(s["lidar_xyz"]))

    def _update_motors(self, motors: dict):
        ids = sorted(motors)
        self.motor_table.setRowCount(len(ids))
        for row, mid in enumerate(ids):
            m = motors[mid]
            ctype = protocol.COMMAND_TYPE.get(m.command_type, str(m.command_type))
            stat = protocol.MOTOR_STATUS.get(m.status, str(m.status))
            err = m.setpoint_value - m.current_value
            cells = [
                str(m.id), ctype, f"{m.setpoint_value:.3f}", f"{m.current_value:.3f}",
                f"{m.min_value:.2f}", f"{m.max_value:.2f}", stat,
                f"{m.p:.2f}/{m.i:.2f}/{m.d:.2f}", f"{err:+.3f}",
            ]
            for col, text in enumerate(cells):
                item = self.motor_table.item(row, col)
                if item is None:
                    item = QtWidgets.QTableWidgetItem()
                    self.motor_table.setItem(row, col, item)
                item.setText(text)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="OSPR-AI live monitor")
    parser.add_argument("--debug", action="store_true",
                        help="verbose logging: per-stream rates, decode errors, "
                             "first-packet notices, and crash tracebacks")
    parser.add_argument("--log-file", metavar="PATH", default=None,
                        help="also write logs to this file")
    parser.add_argument("--lidar-view", choices=["2d", "3d"], default=None,
                        help="override config.LIDAR_VIEW")
    args = parser.parse_args()

    # native traceback if Qt/OpenGL segfaults (common source of silent crashes)
    faulthandler.enable()

    handlers = [logging.StreamHandler(sys.stdout)]
    if args.log_file:
        handlers.append(logging.FileHandler(args.log_file))
    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
        datefmt="%H:%M:%S", handlers=handlers)
    log.info("starting OSPR-AI monitor (debug=%s)", args.debug)
    if args.debug:
        log.debug("config: lidar_source=%s lidar_view=%s beams=%s typed_framing=%s "
                  "pure_py_protobuf=%s", config.LIDAR_SOURCE, config.LIDAR_VIEW,
                  config.LIDAR_BEAMS, config.UDP_TYPED_FRAMING,
                  config.FORCE_PURE_PYTHON_PROTOBUF)

    if args.lidar_view:
        config.LIDAR_VIEW = args.lidar_view

    state = SharedState()
    start_udp_receivers(state)

    gz = GazeboReceiver(state)
    gz.start()

    app = QtWidgets.QApplication([sys.argv[0]])   # keep Qt out of our argv
    win = MonitorWindow(state, gz.error)
    win.show()
    code = app.exec() if hasattr(app, "exec") else app.exec_()
    gz.stop()
    sys.exit(code)


if __name__ == "__main__":
    main()