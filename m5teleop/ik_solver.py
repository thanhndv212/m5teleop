"""Differential IK solver wrapping pink + pinocchio for the SO-ARM100."""

from __future__ import annotations

import numpy as np
import pinocchio as pin
import pink
from pink.tasks import FrameTask, PostureTask

from . import config


class IKSolver:
    """Velocity-level IK for SO-ARM100 using pink (pinocchio backend).

    Parameters
    ----------
    urdf_path:
        Absolute path to ``so100.urdf``.
    ee_frame:
        Name of the end-effector link in the URDF (default ``"jaw"``).

    Usage
    -----
    ::

        solver = IKSolver()
        solver.reset(q0_radians)      # call once after connecting the arm
        for ...:
            q_new = solver.step(twist, dt=0.02)
    """

    def __init__(
        self,
        urdf_path: str = config.URDF_PATH,
        ee_frame: str = config.EE_FRAME,
    ) -> None:
        self._ee_frame = ee_frame

        # Build pinocchio model from URDF (geometry not needed for IK)
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()

        # pink configuration wrapper
        self.configuration: pink.Configuration | None = None

        # Tasks
        self._ee_task = FrameTask(
            ee_frame,
            position_cost=config.EE_POSITION_COST,
            orientation_cost=config.EE_ORIENTATION_COST,
        )
        self._posture_task = PostureTask(cost=config.POSTURE_COST)

        self._q_neutral = pin.neutral(self.model)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def nq(self) -> int:
        """Number of configuration DOF (joints + any quaternion components)."""
        return self.model.nq

    @property
    def nv(self) -> int:
        """Number of velocity DOF."""
        return self.model.nv

    def reset(self, q0: np.ndarray) -> None:
        """Initialise (or re-initialise) the solver at configuration *q0*.

        Parameters
        ----------
        q0:
            Joint configuration in **radians**, length ``nq``.
        """
        self.configuration = pink.Configuration(self.model, self.data, q0)
        pin.forwardKinematics(self.model, self.data, q0)
        pin.updateFramePlacements(self.model, self.data)

        # Seed tasks at the current EE pose
        current_ee = self.configuration.get_transform_frame_to_world(
            self._ee_frame
        )
        self._ee_task.set_target(current_ee)
        self._posture_task.set_target(self._q_neutral)

    def step(self, twist_world: np.ndarray, dt: float) -> np.ndarray:
        """Advance the IK by one time step.

        Parameters
        ----------
        twist_world:
            6-D array ``[vx, vy, vz, ωx, ωy, ωz]`` in the world frame.
        dt:
            Time step in seconds.

        Returns
        -------
        np.ndarray
            New joint configuration *q* in **radians**, length ``nq``.
        """
        if self.configuration is None:
            raise RuntimeError("Call reset(q0) before step().")

        # Integrate the EE target by the commanded twist
        current_ee = self.configuration.get_transform_frame_to_world(
            self._ee_frame
        )
        v_lin = twist_world[:3]
        v_ang = twist_world[3:]
        delta_pos = v_lin * dt
        delta_ang = v_ang * dt
        angle = float(np.linalg.norm(delta_ang))

        new_translation = current_ee.translation + delta_pos
        if angle > 1e-9:
            axis = delta_ang / angle
            dR = pin.AngleAxis(angle, axis).toRotationMatrix()
            new_rotation = dR @ current_ee.rotation
        else:
            new_rotation = current_ee.rotation

        target_se3 = pin.SE3(new_rotation, new_translation)
        self._ee_task.set_target(target_se3)

        # Solve IK
        velocity = pink.solve_ik(
            self.configuration,
            [self._ee_task, self._posture_task],
            dt,
            solver=config.IK_SOLVER,
        )
        self.configuration.integrate_inplace(velocity, dt)
        return np.array(self.configuration.q)

    def get_ee_pose(self) -> pin.SE3:
        """Return current EE pose (world frame) as ``pin.SE3``."""
        if self.configuration is None:
            return pin.SE3.Identity()
        return self.configuration.get_transform_frame_to_world(self._ee_frame)

    def q_to_degrees(self, q: np.ndarray) -> dict[str, float]:
        """Convert pinocchio joint config *q* to a degrees dict for lerobot.

        Only the 5 revolute joints are exported (gripper handled separately).
        """
        # SO100 joints 1-5 map to indices 0-4 in q
        result: dict[str, float] = {}
        for i, name in enumerate(config.JOINT_NAMES[:5]):
            result[f"{name}.pos"] = float(np.degrees(q[i]))
        return result

    def degrees_to_q(self, deg_dict: dict[str, float]) -> np.ndarray:
        """Build a pinocchio config vector from a lerobot degrees dict."""
        q = pin.neutral(self.model).copy()
        for i, name in enumerate(config.JOINT_NAMES[:5]):
            key = f"{name}.pos"
            if key in deg_dict:
                q[i] = np.radians(deg_dict[key])
        return q
