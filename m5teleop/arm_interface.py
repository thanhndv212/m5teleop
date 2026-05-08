"""Thin wrapper around lerobot SO100Follower for use in m5teleop."""

from __future__ import annotations

import sys
from typing import TYPE_CHECKING

import numpy as np

from . import config

# Make lerobot importable regardless of install state
if config.LEROBOT_SRC not in sys.path:
    sys.path.insert(0, config.LEROBOT_SRC)

if TYPE_CHECKING:
    pass


class ArmInterface:
    """Connect to, read from, and command the SO-ARM100 via lerobot.

    Parameters
    ----------
    port:
        Serial port for the servo bus (e.g. ``/dev/cu.usbserial-XXXX``).
        Overrides :data:`config.SERVO_PORT`.
    dry_run:
        When ``True``, skip serial connection (useful for offline testing).
    """

    def __init__(
        self,
        port: str | None = None,
        dry_run: bool = False,
    ) -> None:
        self._dry_run = dry_run
        self._port = port or config.SERVO_PORT
        self._robot = None
        self._gripper_open = True

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def connect(self) -> None:
        """Open serial connection and calibrate if needed."""
        if self._dry_run:
            return
        if self._port is None:
            raise ValueError(
                "No servo port specified. Pass --servo-port /dev/cu.usbserial-XXXX"
            )
        from lerobot.robots.so_follower.config_so_follower import (
            SOFollowerRobotConfig,
        )
        from lerobot.robots.so_follower.so_follower import SOFollower

        robot_cfg = SOFollowerRobotConfig(
            port=self._port,
            use_degrees=True,
            max_relative_target=config.MAX_RELATIVE_TARGET,
        )
        self._robot = SOFollower(robot_cfg)
        self._robot.connect(calibrate=False)

    def disconnect(self) -> None:
        """Gracefully disconnect from the arm."""
        if self._robot is not None:
            try:
                self._robot.disconnect()
            except Exception:
                pass
            self._robot = None

    def __enter__(self) -> "ArmInterface":
        self.connect()
        return self

    def __exit__(self, *_) -> None:
        self.disconnect()

    # ------------------------------------------------------------------
    # State read-back
    # ------------------------------------------------------------------

    def get_joint_degrees(self) -> dict[str, float]:
        """Return present joint positions as ``{name.pos: degrees}``."""
        if self._dry_run or self._robot is None:
            return {f"{n}.pos": 0.0 for n in config.JOINT_NAMES}
        obs = self._robot.get_observation()
        return {k: float(v) for k, v in obs.items() if k.endswith(".pos")}

    def get_joint_radians(self) -> np.ndarray:
        """Return present joint positions (5 arm joints) in radians."""
        deg = self.get_joint_degrees()
        return np.array(
            [
                np.radians(deg.get(f"{n}.pos", 0.0))
                for n in config.JOINT_NAMES[:5]
            ]
        )

    # ------------------------------------------------------------------
    # Command
    # ------------------------------------------------------------------

    def send_joint_degrees(self, deg_dict: dict[str, float]) -> None:
        """Send joint position targets (degrees) to the arm.

        Automatically appends the current gripper position so every
        ``send_action`` call includes all 6 joints.
        """
        if self._dry_run or self._robot is None:
            return
        full = dict(deg_dict)
        full["gripper.pos"] = (
            config.GRIPPER_OPEN_DEG
            if self._gripper_open
            else config.GRIPPER_CLOSED_DEG
        )
        self._robot.send_action(full)

    def set_gripper(self, open_gripper: bool) -> None:
        """Set gripper state; position is applied on the next send_joint_degrees call."""
        self._gripper_open = open_gripper

    def toggle_gripper(self) -> None:
        """Toggle gripper between open and closed."""
        self.set_gripper(not self._gripper_open)

    @property
    def gripper_is_open(self) -> bool:
        return self._gripper_open
