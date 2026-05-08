"""IMU → 6D end-effector twist converter with low-pass filter and dead-zone."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np

from . import config

if TYPE_CHECKING:
    from m5imu import ImuData


def _deadzone(value: float, threshold: float) -> float:
    """Apply a symmetric dead-zone to *value*."""
    if abs(value) < threshold:
        return 0.0
    return math.copysign(abs(value) - threshold, value)


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class ImuTwistConverter:
    """Convert :class:`ImuData` into a 6-D EE twist command.

    Twist convention: ``[vx, vy, vz, ωx, ωy, ωz]``
    - Linear velocities in m/s (base frame)
    - Angular velocities in rad/s (EE frame)

    Mapping
    -------
    - Roll  (tilt around X) = atan2(ax, az) → v_y (lateral)
    - Pitch (tilt around Y) = atan2(ay, az) → v_x (forward/back)
    - Gyro Z                → ω_z (EE yaw)
    - Gyro X                → ω_x (EE roll)
    - Gyro Y                → ω_y (EE pitch)
    """

    def __init__(
        self,
        lpf_alpha: float = config.LPF_ALPHA,
        tilt_deadzone: float = config.TILT_DEADZONE,
        gyro_deadzone: float = config.GYRO_DEADZONE,
        lin_gain: float = config.LIN_GAIN,
        ang_gain: float = config.ANG_GAIN,
        max_lin_vel: float = config.MAX_LIN_VEL,
        max_ang_vel: float = config.MAX_ANG_VEL,
    ) -> None:
        self._alpha = lpf_alpha
        self._tilt_dz = tilt_deadzone
        self._gyro_dz = gyro_deadzone
        self._lin_gain = lin_gain
        self._ang_gain = ang_gain
        self._max_lin = max_lin_vel
        self._max_ang = max_ang_vel

        # Low-pass filter state: [ax, ay, az, gx, gy, gz]
        self._lpf: np.ndarray = np.zeros(6)
        self._initialised = False

    def reset(self) -> None:
        """Reset filter state (call when teleop is re-enabled)."""
        self._lpf[:] = 0.0
        self._initialised = False

    def to_twist(self, imu: "ImuData") -> np.ndarray:
        """Return ``np.ndarray([vx, vy, vz, ωx, ωy, ωz])`` from one sample.

        Parameters
        ----------
        imu:
            Raw :class:`~m5imu.ImuData` sample.
        """
        raw = np.array(
            [
                imu.ax,
                imu.ay,
                imu.az,
                math.radians(imu.gx),
                math.radians(imu.gy),
                math.radians(imu.gz),
            ]
        )

        if not self._initialised:
            self._lpf = raw.copy()
            self._initialised = True
        else:
            self._lpf = self._alpha * raw + (1.0 - self._alpha) * self._lpf

        ax, ay, az, gx, gy, gz = self._lpf

        # Tilt → linear velocity
        pitch = math.atan2(ay, math.hypot(ax, az))  # forward/back tilt
        roll = math.atan2(ax, math.hypot(ay, az))  # lateral tilt

        vx = _clamp(
            _deadzone(pitch, self._tilt_dz) * self._lin_gain,
            self._max_lin,
        )
        vy = _clamp(
            _deadzone(roll, self._tilt_dz) * self._lin_gain,
            self._max_lin,
        )
        vz = 0.0  # reserved — no intuitive tilt maps to z

        # Gyro → angular velocity
        ox = _clamp(
            _deadzone(gx, self._gyro_dz) * self._ang_gain, self._max_ang
        )
        oy = _clamp(
            _deadzone(gy, self._gyro_dz) * self._ang_gain, self._max_ang
        )
        oz = _clamp(
            _deadzone(gz, self._gyro_dz) * self._ang_gain, self._max_ang
        )

        return np.array([vx, vy, vz, ox, oy, oz])
