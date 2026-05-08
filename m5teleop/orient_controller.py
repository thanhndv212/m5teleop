"""Two-layer cascade orientation controller for EE → IMU frame tracking.

Architecture
------------
Layer 1 — outer (orientation position error → angular velocity setpoint)
    q_err  = q_target ⊗ q_ee⁻¹                (quaternion error)
    err    = rotvec(q_err)                      (axis-angle, ℝ³, avoids gimbal lock)
    ω_set  = Kp_outer · err                     (P controller)

Layer 2 — inner (velocity error → twist command)
    ω_actual = estimated EE angular velocity    (from successive rotation matrices)
    ω_cmd    = ω_set + Kp_inner · (ω_set − ω_actual)

The 6-D twist output has zero linear component (pure orientation tracking).
The caller feeds twist → IKSolver.step() which IS the inner actuator loop.

Reference frame
---------------
At zero_reset() the controller memorises q_imu_ref and q_ee_ref.
At run time the relative IMU rotation is mapped onto the EE:

    q_delta  = q_imu_ref⁻¹ ⊗ q_imu           (IMU rotation since reset)
    q_target = q_ee_ref  ⊗ q_delta            (desired EE orientation)

This formulation is coordinate-system agnostic: any initial misalignment
between the IMU body frame and the EE frame is absorbed by the reset.
"""

from __future__ import annotations

import numpy as np

from . import config


# ---------------------------------------------------------------------------
# Quaternion helpers  ([w, x, y, z] convention throughout)
# ---------------------------------------------------------------------------


def _qmul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
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


def _qinv(q: np.ndarray) -> np.ndarray:
    """Quaternion conjugate (inverse for unit quaternion)."""
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_to_rotvec(q: np.ndarray) -> np.ndarray:
    """Quaternion [w,x,y,z] → rotation vector (axis × angle, radians).

    Chooses the shortest path: |angle| ≤ π, equivalent to 2·arccos(|w|).
    Gimbal-lock-free: no Euler angle extraction.
    """
    q = q / np.linalg.norm(q)
    if q[0] < 0:          # ensure w ≥ 0  →  angle ∈ [0, π]
        q = -q
    w = float(np.clip(q[0], -1.0, 1.0))
    angle = 2.0 * np.arccos(w)
    sin_half = np.sin(angle / 2.0)
    if sin_half < 1e-9:
        return np.zeros(3)
    return (q[1:] / sin_half) * angle


def _rot_from_q(q: np.ndarray) -> np.ndarray:
    """Unit quaternion [w,x,y,z] → 3×3 rotation matrix."""
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)    ],
            [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)    ],
            [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
        ]
    )


# ---------------------------------------------------------------------------
# Cascade controller
# ---------------------------------------------------------------------------


class OrientationController:
    """Two-layer cascade controller: EE orientation tracks IMU orientation.

    Parameters
    ----------
    kp_outer : float
        Outer loop proportional gain  [1/s].
        Maps orientation error (rad) → angular velocity setpoint (rad/s).
    kp_inner : float
        Inner loop proportional gain  [dimensionless].
        Scales the velocity correction: ω_cmd = ω_set + Kp_inner·(ω_set − ω_actual).
    max_omega : float
        Hard clamp on the output angular velocity magnitude [rad/s].
    """

    def __init__(
        self,
        kp_outer:  float = config.ORIENT_KP_OUTER,
        kp_inner:  float = config.ORIENT_KP_INNER,
        max_omega: float = config.ORIENT_MAX_OMEGA,
    ) -> None:
        self.kp_outer  = kp_outer
        self.kp_inner  = kp_inner
        self.max_omega = max_omega

        # Reference frame (updated on zero_reset)
        self._q_imu_ref: np.ndarray = np.array([1.0, 0.0, 0.0, 0.0])
        self._q_ee_ref:  np.ndarray = np.array([1.0, 0.0, 0.0, 0.0])

        # Inner-loop state
        self._R_ee_prev:   np.ndarray = np.eye(3)
        self._omega_actual: np.ndarray = np.zeros(3)

        # Latest error and target (for external logging)
        self._last_err:    np.ndarray = np.zeros(3)
        self._q_target:    np.ndarray = np.array([1.0, 0.0, 0.0, 0.0])

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def zero_reset(self, q_imu: np.ndarray, q_ee: np.ndarray) -> None:
        """Capture the current IMU and EE orientations as the reference frame.

        After this call, the controller will try to keep the EE orientation
        equal to the EE reference orientation plus whatever rotation the IMU
        has made since this reset.

        Parameters
        ----------
        q_imu : array (4,)
            Current IMU orientation quaternion [w, x, y, z].
        q_ee  : array (4,)
            Current EE orientation quaternion  [w, x, y, z].
        """
        self._q_imu_ref    = q_imu / np.linalg.norm(q_imu)
        self._q_ee_ref     = q_ee  / np.linalg.norm(q_ee)
        self._q_target     = self._q_ee_ref.copy()  # target = EE at reset
        self._R_ee_prev    = _rot_from_q(self._q_ee_ref)
        self._omega_actual = np.zeros(3)
        self._last_err     = np.zeros(3)
        print("[OrientController] Zero reset — reference frame captured.")

    def compute_twist(
        self,
        q_imu: np.ndarray,
        q_ee:  np.ndarray,
        dt:    float,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Compute the 6-D EE twist to drive EE orientation toward the IMU target.

        Parameters
        ----------
        q_imu : array (4,)
            Current (EKF-filtered) IMU quaternion [w, x, y, z].
        q_ee  : array (4,)
            Current EE quaternion from forward kinematics [w, x, y, z].
        dt    : float
            Control cycle time [s].

        Returns
        -------
        twist : array (6,)
            [0, 0, 0,  ωx, ωy, ωz] in world frame.
        err_vec : array (3,)
            Orientation error as rotation vector [rad]  (for logging).
        """
        q_imu_n = q_imu / np.linalg.norm(q_imu)
        q_ee_n  = q_ee  / np.linalg.norm(q_ee)

        # ── Target EE orientation ─────────────────────────────────────
        # q_delta:  how much the IMU has rotated since zero-reset
        q_delta  = _qmul(_qinv(self._q_imu_ref), q_imu_n)
        # Apply the same relative rotation to the EE reference orientation
        q_target = _qmul(self._q_ee_ref, q_delta)
        self._q_target = q_target / np.linalg.norm(q_target)

        # ── Layer 1 — outer: orientation error → ω setpoint ──────────
        # q_err = rotation needed to bring q_ee to q_target (in world frame)
        q_err   = _qmul(q_target, _qinv(q_ee_n))
        err_vec = quat_to_rotvec(q_err)          # ℝ³, gimbal-lock-free

        omega_set  = self.kp_outer * err_vec
        omega_norm = float(np.linalg.norm(omega_set))
        if omega_norm > self.max_omega:
            omega_set = omega_set / omega_norm * self.max_omega

        # ── Layer 2 — inner: velocity error correction ────────────────
        # Estimate current EE angular velocity from finite difference of R.
        # ω = vee(Ṙ Rᵀ)  ≈  vee((R_k − R_{k−1}) / dt · R_kᵀ)
        R_ee     = _rot_from_q(q_ee_n)
        dt_safe  = max(dt, 1e-4)
        R_dot    = (R_ee - self._R_ee_prev) / dt_safe
        Omega    = R_dot @ R_ee.T              # skew-symmetric (approximately)
        omega_actual = np.array([Omega[2, 1], Omega[0, 2], Omega[1, 0]])
        self._R_ee_prev    = R_ee
        self._omega_actual = omega_actual

        omega_err = omega_set - omega_actual
        omega_cmd = omega_set + self.kp_inner * omega_err

        # Re-clamp after inner correction
        omega_norm = float(np.linalg.norm(omega_cmd))
        if omega_norm > self.max_omega:
            omega_cmd = omega_cmd / omega_norm * self.max_omega

        self._last_err = err_vec.copy()

        twist = np.zeros(6)
        twist[3:] = omega_cmd
        return twist, err_vec

    @property
    def last_error(self) -> np.ndarray:
        """Last computed orientation error vector [rad]."""
        return self._last_err.copy()

    @property
    def last_target(self) -> np.ndarray:
        """Last computed EE target quaternion [w, x, y, z]."""
        return self._q_target.copy()

    @property
    def last_target_rotation(self) -> np.ndarray:
        """Last computed EE target orientation as 3×3 rotation matrix."""
        return _rot_from_q(self._q_target)
