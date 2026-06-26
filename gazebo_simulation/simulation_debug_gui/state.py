"""
Thread-safe snapshot of everything the GUI needs to draw.

Receiver threads (UDP + Gazebo) write here; the GUI's repaint timer reads here.
A single lock keeps it simple and correct. Big buffers (camera frames, lidar
points) are swapped by reference, so the lock is only held briefly.
"""
from __future__ import annotations

import threading
import time
from collections import deque

import numpy as np

import config
from protocol import Gnss, Imu, Lidar, Motor


class RateMeter:
    """Tracks messages-per-second for a stream (for the status bar)."""

    def __init__(self, window: float = 2.0):
        self._stamps: deque[float] = deque()
        self._window = window

    def tick(self) -> None:
        now = time.monotonic()
        self._stamps.append(now)
        while self._stamps and now - self._stamps[0] > self._window:
            self._stamps.popleft()

    def hz(self) -> float:
        if len(self._stamps) < 2:
            return 0.0
        span = self._stamps[-1] - self._stamps[0]
        return (len(self._stamps) - 1) / span if span > 0 else 0.0


class SharedState:
    def __init__(self) -> None:
        self.lock = threading.Lock()

        # Latest decoded values
        self.gnss = Gnss()
        self.imu = Imu()
        self.motors: dict[int, Motor] = {}     # keyed by motor id
        self.motor_hist: dict[int, deque] = {}  # id -> deque of (current, setpoint)
        self.lidar = Lidar()                   # from UDP path (if used)

        # Camera frames as HxWx3 uint8 numpy arrays (or None until first frame)
        self.cam_left: np.ndarray | None = None
        self.cam_right: np.ndarray | None = None

        # Lidar points as Nx3 (x,y,z) in the sensor frame (z up)
        self.lidar_xyz: np.ndarray | None = None
        self.lidar_range: np.ndarray | None = None   # per-point range, for color

        # Attitude estimate (roll, pitch, yaw) in radians
        self.attitude = (0.0, 0.0, 0.0)

        # IMU history for time-series plots
        self.imu_hist_accel = deque(maxlen=config.IMU_HISTORY)
        self.imu_hist_gyro = deque(maxlen=config.IMU_HISTORY)

        # Rate meters
        self.rates = {
            "imu": RateMeter(), "gnss": RateMeter(), "motor": RateMeter(),
            "lidar": RateMeter(), "cam_left": RateMeter(), "cam_right": RateMeter(),
        }

    # ---- writers (called from receiver threads) --------------------------
    def set_gnss(self, g: Gnss) -> None:
        with self.lock:
            self.gnss = g
            self.rates["gnss"].tick()

    def set_imu(self, imu: Imu, attitude=None) -> None:
        with self.lock:
            self.imu = imu
            if attitude is not None:
                self.attitude = attitude
            self.imu_hist_accel.append((imu.l_accel_x, imu.l_accel_y, imu.l_accel_z))
            self.imu_hist_gyro.append((imu.a_velocity_x, imu.a_velocity_y, imu.a_velocity_z))
            self.rates["imu"].tick()

    def set_motor(self, m: Motor) -> None:
        with self.lock:
            self.motors[m.id] = m
            h = self.motor_hist.get(m.id)
            if h is None:
                h = deque(maxlen=config.MOTOR_HISTORY)
                self.motor_hist[m.id] = h
            h.append((m.current_value, m.setpoint_value))
            self.rates["motor"].tick()

    def set_lidar_udp(self, lid: Lidar) -> None:
        with self.lock:
            self.lidar = lid
            self.rates["lidar"].tick()

    def set_lidar_points(self, xyz: np.ndarray, rng: np.ndarray) -> None:
        with self.lock:
            self.lidar_xyz = xyz
            self.lidar_range = rng
            self.rates["lidar"].tick()

    def set_camera(self, side: str, frame: np.ndarray) -> None:
        with self.lock:
            if side == "left":
                self.cam_left = frame
                self.rates["cam_left"].tick()
            else:
                self.cam_right = frame
                self.rates["cam_right"].tick()

    # ---- snapshot (called from GUI thread) -------------------------------
    def snapshot(self) -> dict:
        with self.lock:
            return {
                "gnss": self.gnss,
                "imu": self.imu,
                "attitude": self.attitude,
                "motors": dict(self.motors),
                "motor_hist": {i: np.array(d) for i, d in self.motor_hist.items() if d},
                "cam_left": self.cam_left,
                "cam_right": self.cam_right,
                "lidar_xyz": self.lidar_xyz,
                "lidar_range": self.lidar_range,
                "accel_hist": np.array(self.imu_hist_accel) if self.imu_hist_accel else None,
                "gyro_hist": np.array(self.imu_hist_gyro) if self.imu_hist_gyro else None,
                "rates": {k: v.hz() for k, v in self.rates.items()},
            }