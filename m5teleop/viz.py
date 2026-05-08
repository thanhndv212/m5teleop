"""Rerun-based data logging for the teleop pipeline."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
import rerun as rr

from . import config

if TYPE_CHECKING:
    from m5imu import ImuData


def _euler_to_rotation_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """ZYX Euler angles → 3×3 rotation matrix.

    Convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll).
    """
    r, p, y = np.deg2rad([roll_deg, pitch_deg, yaw_deg])
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


class TeleopVisualizer:
    """Log IMU, twist, joint positions, and EE pose to Rerun.

    Open the Rerun viewer with::

        rerun --connect

    Or pass ``spawn=True`` to :meth:`init` to auto-launch the viewer.
    """

    def __init__(self, session_name: str = config.RERUN_SESSION) -> None:
        self._session = session_name

    def init(self, spawn: bool = False) -> None:
        """Initialise the Rerun recording session."""
        rr.init(self._session, spawn=spawn)
        if spawn:
            # spawn already starts a viewer and connects; nothing else needed
            return
        try:
            rr.connect_grpc(flush_timeout_sec=0.5)
        except Exception:
            # No viewer running — logging is silently dropped
            pass

    def log_imu(self, imu: "ImuData") -> None:
        """Log raw IMU sensor values."""
        rr.log("imu/accel/x", rr.Scalars(imu.ax))
        rr.log("imu/accel/y", rr.Scalars(imu.ay))
        rr.log("imu/accel/z", rr.Scalars(imu.az))
        rr.log("imu/gyro/x", rr.Scalars(imu.gx))
        rr.log("imu/gyro/y", rr.Scalars(imu.gy))
        rr.log("imu/gyro/z", rr.Scalars(imu.gz))
        rr.log("imu/temp", rr.Scalars(imu.temp))
        rr.log("buttons/btnA", rr.Scalars(int(imu.btn_a)))
        rr.log("buttons/btnB", rr.Scalars(int(imu.btn_b)))
        rr.log("imu/orientation/pitch", rr.Scalars(imu.pitch))
        rr.log("imu/orientation/roll",  rr.Scalars(imu.roll))
        rr.log("imu/orientation/yaw",   rr.Scalars(imu.yaw))

    def log_ekf(self, roll: float, pitch: float, yaw: float, bias: np.ndarray) -> None:
        """Log EKF orientation estimate and estimated gyro bias."""
        rr.log("ekf/orientation/roll",   rr.Scalars(roll))
        rr.log("ekf/orientation/pitch",  rr.Scalars(pitch))
        rr.log("ekf/orientation/yaw",    rr.Scalars(yaw))
        rr.log("ekf/bias/x", rr.Scalars(float(bias[0])))
        rr.log("ekf/bias/y", rr.Scalars(float(bias[1])))
        rr.log("ekf/bias/z", rr.Scalars(float(bias[2])))

    def log_orient_error(
        self, err_vec: np.ndarray, omega_actual: np.ndarray | None = None
    ) -> None:
        """Log orientation tracking error and inner-loop angular velocity.

        Parameters
        ----------
        err_vec      : rotation-vector error [rad] — 3D, quaternion-derived.
        omega_actual : estimated EE angular velocity [rad/s] (optional).
        """
        rr.log("orient/error/x",    rr.Scalars(float(err_vec[0])))
        rr.log("orient/error/y",    rr.Scalars(float(err_vec[1])))
        rr.log("orient/error/z",    rr.Scalars(float(err_vec[2])))
        rr.log("orient/error/norm", rr.Scalars(float(np.linalg.norm(err_vec))))
        rr.log(
            "orient/error/arrow",
            rr.Arrows3D(
                origins=[[0.0, -0.28, 0.28]],
                vectors=[err_vec.tolist()],
                colors=[[255, 200, 0]],
            ),
        )
        if omega_actual is not None:
            rr.log("orient/omega_actual/x", rr.Scalars(float(omega_actual[0])))
            rr.log("orient/omega_actual/y", rr.Scalars(float(omega_actual[1])))
            rr.log("orient/omega_actual/z", rr.Scalars(float(omega_actual[2])))

    def log_imu_frame(self, imu: "ImuData") -> None:
        """Log the estimated IMU body frame as three coloured axis arrows.

        Red = X, Green = Y, Blue = Z.  Arrows are placed at a fixed position
        in the 3-D view so they don't overlap the robot.
        """
        R = _euler_to_rotation_matrix(imu.roll, imu.pitch, imu.yaw)
        scale = 0.08  # arrow length in metres
        origin = np.array([[0.0, 0.28, 0.28]] * 3)
        rr.log(
            "imu/frame",
            rr.Arrows3D(
                origins=origin,
                vectors=R.T * scale,  # rows → [X-axis, Y-axis, Z-axis]
                colors=[[220, 50, 50], [50, 220, 50], [50, 50, 220]],
                labels=["X", "Y", "Z"],
            ),
        )

    def log_twist(self, twist: np.ndarray) -> None:
        """Log the 6D EE twist command [vx,vy,vz,ωx,ωy,ωz]."""
        labels = ["vx", "vy", "vz", "wx", "wy", "wz"]
        for label, value in zip(labels, twist):
            rr.log(f"twist/{label}", rr.Scalars(float(value)))

    def log_joints(self, q: np.ndarray) -> None:
        """Log joint configuration (radians)."""
        for i, name in enumerate(config.JOINT_NAMES[:5]):
            if i < len(q):
                rr.log(f"joints/{name}", rr.Scalars(float(q[i])))

    def log_ee_pose(
        self, translation: np.ndarray, rotation_matrix: np.ndarray
    ) -> None:
        """Log end-effector position (m) and orientation (rotation matrix → euler)."""
        rr.log("ee/x", rr.Scalars(float(translation[0])))
        rr.log("ee/y", rr.Scalars(float(translation[1])))
        rr.log("ee/z", rr.Scalars(float(translation[2])))
        # Log as a 3-D point for the spatial view
        rr.log(
            "ee/position",
            rr.Points3D(
                positions=[translation.tolist()],
                radii=[0.01],
                colors=[[255, 100, 0]],
            ),
        )

    def log_teleop_state(self, active: bool, gripper_open: bool) -> None:
        """Log binary teleop states."""
        rr.log("state/teleop_active", rr.Scalars(int(active)))
        rr.log("state/gripper_open", rr.Scalars(int(gripper_open)))

    def log_all(
        self,
        imu: "ImuData",
        twist: np.ndarray,
        q: np.ndarray,
        ee_translation: np.ndarray,
        ee_rotation: np.ndarray,
        teleop_active: bool,
        gripper_open: bool,
    ) -> None:
        """Convenience: log all channels in one call."""
        self.log_imu(imu)
        self.log_imu_frame(imu)
        self.log_twist(twist)
        self.log_joints(q)
        self.log_ee_pose(ee_translation, ee_rotation)
        self.log_teleop_state(teleop_active, gripper_open)

    def log_all_with_tracking(
        self,
        imu: "ImuData",
        twist: np.ndarray,
        q: np.ndarray,
        ee_translation: np.ndarray,
        ee_rotation: np.ndarray,
        teleop_active: bool,
        gripper_open: bool,
        ekf_euler: tuple[float, float, float],
        ekf_bias: np.ndarray,
        orient_err: np.ndarray,
        omega_actual: np.ndarray,
    ) -> None:
        """Extended log_all that also records EKF and tracking-error channels."""
        self.log_all(
            imu, twist, q, ee_translation, ee_rotation, teleop_active, gripper_open
        )
        self.log_ekf(ekf_euler[0], ekf_euler[1], ekf_euler[2], ekf_bias)
        self.log_orient_error(orient_err, omega_actual)
