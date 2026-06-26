"""
Attitude estimation from raw IMU.

IMPORTANT: the robomorph IMUMeasurements message sent over UDP carries only RAW
data (accel xyz, gyro xyz, mag xyz) -- there is no Kalman/attitude field in it.
So "attitude from the filter" is estimated HERE on the client with a simple
complementary filter:

    roll/pitch  <- accelerometer (gravity direction), gyro-integrated short term
    yaw         <- magnetometer (tilt-compensated), gyro-integrated short term

If you later add a real Kalman/EKF output to the Rust IMU message, just decode
those fields in protocol.Imu and feed them straight into the GUI instead of
calling this estimator.

This is intentionally lightweight (not an EKF). It's meant for a monitor, not
for control.
"""
from __future__ import annotations

import math
import time

from protocol import Imu


class ComplementaryFilter:
    def __init__(self, alpha: float = 0.98):
        self.alpha = alpha          # weight on the gyro-integrated estimate
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self._last = None

    def update(self, imu: Imu) -> tuple[float, float, float]:
        now = time.monotonic()
        dt = 0.0 if self._last is None else now - self._last
        self._last = now
        dt = min(dt, 0.1)           # guard against long stalls

        ax, ay, az = imu.l_accel_x, imu.l_accel_y, imu.l_accel_z
        gx, gy, gz = imu.a_velocity_x, imu.a_velocity_y, imu.a_velocity_z

        # Accelerometer-derived roll/pitch (valid when acceleration ~ gravity)
        acc_roll = math.atan2(ay, az)
        acc_pitch = math.atan2(-ax, math.hypot(ay, az))

        # Gyro integration
        gyro_roll = self.roll + gx * dt
        gyro_pitch = self.pitch + gy * dt

        a = self.alpha
        self.roll = a * gyro_roll + (1 - a) * acc_roll
        self.pitch = a * gyro_pitch + (1 - a) * acc_pitch

        # Yaw: integrate gyro, correct toward tilt-compensated magnetometer
        self.yaw += gz * dt
        mx, my, mz = imu.magnetic_field_x, imu.magnetic_field_y, imu.magnetic_field_z
        if (mx or my or mz):
            cr, sr = math.cos(self.roll), math.sin(self.roll)
            cp, sp = math.cos(self.pitch), math.sin(self.pitch)
            # tilt-compensated horizontal components
            mxh = mx * cp + mz * sp
            myh = mx * sr * sp + my * cr - mz * sr * cp
            mag_yaw = math.atan2(-myh, mxh)
            self.yaw = a * self.yaw + (1 - a) * mag_yaw

        # wrap yaw to [-pi, pi]
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        return self.roll, self.pitch, self.yaw
