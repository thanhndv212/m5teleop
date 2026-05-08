"""Error-State Kalman Filter (ESKF) for 6-DOF IMU attitude estimation.

State
-----
Nominal:   unit quaternion q̄  = [w, x, y, z] ∈ SO(3)
           gyro bias       b̄  ∈ ℝ³  (rad/s)
Error:     δx = [δθ(3), δb(3)] ∈ ℝ⁶

Process model (dt step):
    q̄  ← q̄ ⊗ exp(½(ω_meas − b̄) dt)
    b̄  ← b̄   (bias random walk)

    F  = [[I − [ω̄×]dt,  −I·dt],
          [0_{3×3},       I    ]]   (6×6)
    Q  = diag(σ²_ω·dt², σ²_b·dt²)

Measurement model (normalized accelerometer → gravity direction):
    ĝ  = R(q̄)ᵀ · [0,0,1]ᵀ
    H  = [[ĝ×],  0_{3×3}]   (3×6)
    ν  = a_meas_norm − ĝ
    S  = H P Hᵀ + σ²_acc I
    K  = P Hᵀ S⁻¹
    δx = K ν
    q̄  ← q̄ ⊗ [1, δθ/2];   b̄ ← b̄ + δb
    P  ← (I − KH) P

Update is skipped when |‖a‖ − 1| > acc_gate  (free-fall or shock).
"""

from __future__ import annotations

import numpy as np

from . import config


# ---------------------------------------------------------------------------
# Quaternion helpers  (convention [w, x, y, z])
# ---------------------------------------------------------------------------


def _skew(v: np.ndarray) -> np.ndarray:
    """3-vector → 3×3 skew-symmetric matrix."""
    return np.array(
        [
            [0.0,   -v[2],  v[1]],
            [v[2],   0.0,  -v[0]],
            [-v[1],  v[0],  0.0],
        ]
    )


def _qmul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product  a ⊗ b,  [w,x,y,z] convention."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


def q_to_rot(q: np.ndarray) -> np.ndarray:
    """Unit quaternion [w,x,y,z] → 3×3 rotation matrix R (body→world)."""
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)    ],
            [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)    ],
            [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
        ]
    )


# ---------------------------------------------------------------------------
# ESKF
# ---------------------------------------------------------------------------


class ImuEKF:
    """Error-State Kalman Filter for 3-D orientation from a 6-DOF IMU.

    Parameters
    ----------
    sigma_gyro : float
        Gyro measurement noise std  (rad/s).
    sigma_bias : float
        Gyro bias drift noise std   (rad/s per √s).
    sigma_acc  : float
        Accelerometer noise std     (g).
    acc_gate   : float
        Skip accelerometer update when  |‖a‖ − 1g| > acc_gate.
    """

    def __init__(
        self,
        sigma_gyro: float = config.EKF_SIGMA_GYRO,
        sigma_bias: float = config.EKF_SIGMA_BIAS,
        sigma_acc:  float = config.EKF_SIGMA_ACC,
        acc_gate:   float = config.EKF_ACC_GATE,
    ) -> None:
        # Nominal state
        self._q    = np.array([1.0, 0.0, 0.0, 0.0])  # [w, x, y, z]
        self._bias = np.zeros(3)                        # rad/s

        # Error-state covariance 6×6  ([δθ, δb])
        self._P = np.eye(6) * 0.01

        self._sigma_gyro = sigma_gyro
        self._sigma_bias = sigma_bias
        self._sigma_acc  = sigma_acc
        self._acc_gate   = acc_gate

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def step(
        self,
        ax: float, ay: float, az: float,
        gx_dps: float, gy_dps: float, gz_dps: float,
        dt: float,
    ) -> np.ndarray:
        """Run one filter step.

        Parameters
        ----------
        ax, ay, az  : accelerometer reading in *g*
        gx_dps, … : gyroscope reading in *degrees / second*
        dt         : elapsed time in *seconds*

        Returns
        -------
        q : np.ndarray  shape (4,)
            Estimated unit quaternion [w, x, y, z].
        """
        # Convert gyro to rad/s and subtract estimated bias
        omega = np.deg2rad([gx_dps, gy_dps, gz_dps]) - self._bias
        self._predict(omega, dt)
        self._update(np.array([ax, ay, az]))
        return self._q.copy()

    def reset(self) -> None:
        """Re-initialise to identity orientation with zero bias."""
        self._q    = np.array([1.0, 0.0, 0.0, 0.0])
        self._bias = np.zeros(3)
        self._P    = np.eye(6) * 0.01

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def quaternion(self) -> np.ndarray:
        """Estimated orientation as unit quaternion [w, x, y, z]."""
        return self._q.copy()

    @property
    def rotation_matrix(self) -> np.ndarray:
        """Estimated orientation as 3×3 rotation matrix (body → world)."""
        return q_to_rot(self._q)

    @property
    def euler_deg(self) -> tuple[float, float, float]:
        """Estimated orientation as (roll, pitch, yaw) in degrees (ZYX)."""
        R = q_to_rot(self._q)
        pitch = float(np.degrees(np.arcsin(np.clip(-R[2, 0], -1.0, 1.0))))
        roll  = float(np.degrees(np.arctan2(R[2, 1], R[2, 2])))
        yaw   = float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))
        return roll, pitch, yaw

    @property
    def bias_dps(self) -> np.ndarray:
        """Estimated gyro bias in deg/s."""
        return np.degrees(self._bias.copy())

    # ------------------------------------------------------------------
    # Internal steps
    # ------------------------------------------------------------------

    def _predict(self, omega: np.ndarray, dt: float) -> None:
        """Propagate nominal state and error-state covariance."""
        # Integrate quaternion: q ← q ⊗ exp(½ ω dt)
        angle = float(np.linalg.norm(omega)) * dt
        if angle > 1e-9:
            axis = omega / np.linalg.norm(omega)
            dq = np.array([np.cos(angle / 2), *(axis * np.sin(angle / 2))])
        else:
            # Small-angle approximation
            dq = np.array([1.0, *(omega * (dt / 2))])
        dq /= np.linalg.norm(dq)

        self._q = _qmul(self._q, dq)
        self._q /= np.linalg.norm(self._q)

        # Error-state transition matrix  F ∈ ℝ⁶ˣ⁶
        #   F = [[I - [ω×]dt,  -I dt],
        #        [0_{3×3},      I   ]]
        Omega_dt = _skew(omega * dt)
        F = np.block(
            [
                [np.eye(3) - Omega_dt, -np.eye(3) * dt],
                [np.zeros((3, 3)),      np.eye(3)      ],
            ]
        )

        # Process noise
        Q = np.diag(
            [
                *(self._sigma_gyro * dt) ** 2 * np.ones(3),
                *(self._sigma_bias * dt) ** 2 * np.ones(3),
            ]
        )

        self._P = F @ self._P @ F.T + Q

    def _update(self, a_raw: np.ndarray) -> None:
        """Correct orientation with one accelerometer measurement."""
        a_norm = float(np.linalg.norm(a_raw))
        # Skip during free-fall or large dynamic accelerations
        if abs(a_norm - 1.0) > self._acc_gate:
            return

        a_meas = a_raw / a_norm  # normalised

        # Predicted gravity direction in body frame
        R     = q_to_rot(self._q)
        g_hat = R.T @ np.array([0.0, 0.0, 1.0])  # expected = [0,0,1] rotated to body

        # Innovation
        nu = a_meas - g_hat

        # Jacobian  H = [[ĝ×], 0_{3×3}]  (3×6)
        H = np.hstack([_skew(g_hat), np.zeros((3, 3))])

        # Innovation covariance
        R_noise = (self._sigma_acc ** 2) * np.eye(3)
        S = H @ self._P @ H.T + R_noise

        # Kalman gain
        K = self._P @ H.T @ np.linalg.inv(S)

        # Error state update
        dx = K @ nu            # shape (6,)
        dtheta = dx[:3]        # rotation error
        dbias  = dx[3:]        # bias correction

        # Reset: apply δθ to nominal quaternion
        dq = np.array([1.0, *(dtheta / 2)])
        dq /= np.linalg.norm(dq)
        self._q    = _qmul(self._q, dq)
        self._q   /= np.linalg.norm(self._q)
        self._bias += dbias

        # Update covariance (Joseph form for numerical stability)
        IKH      = np.eye(6) - K @ H
        self._P  = IKH @ self._P @ IKH.T + K @ R_noise @ K.T
