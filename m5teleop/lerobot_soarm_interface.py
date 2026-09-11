"""Arm transport for m5teleop, on top of ``soarm_sdk``'s robot interface.

This used to be a hand-rolled wrapper around lerobot's ``SOFollower`` that
implemented no shared contract, which is why teleop could not be pointed at
a planner's robot, a simulation, or an offline stand-in without rewriting
the call site. The transport now lives in the SDK
(:class:`soarm_sdk.LeRobotRobot`, and :class:`soarm_sdk.NullRobot` for
``--dry-run``), both of which satisfy ``soarm_sdk.RobotInterface``.

What stays here is only what is specific to this application: the
degrees-dict shape the IK solver already speaks, the 5-arm-joint view
(teleop solves for the arm and drives the jaw separately), and the
open-loop gripper flag driven by the M5 button.

Why the gripper is not driven through ``Robot.set_gripper()``
-------------------------------------------------------------
Two reasons, both deliberate:

1. lerobot's gripper is a *normalized* 0-100 scale where larger means more
   closed — the opposite sense to the jaw's URDF radians, which is what the
   SDK's ``gripper:`` config block describes. Passing one through the
   other's frame would command the wrong end of the travel.
2. The jaw here is commanded open-loop from a button toggle at 50 Hz.
   ``Robot.gripper_is_open`` reads back from hardware, which would put a
   bus read in the control loop for state this module already knows.

So the open/closed values stay in :mod:`m5teleop.config`, in lerobot
degrees, and are appended to every action.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from . import config


class ArmInterface:
    """Connect to, read from, and command the SO-ARM100 for teleoperation.

    Parameters
    ----------
    port:
        Serial port for the servo bus (e.g. ``/dev/cu.usbserial-XXXX``).
        Overrides :data:`config.SERVO_PORT`.
    dry_run:
        When ``True``, run against :class:`soarm_sdk.NullRobot` instead of
        hardware — no serial port, and no lerobot import. Commands are
        tracked in memory, so read-back reflects what was last commanded
        rather than always reporting zeros.
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
        """Open the connection (or the in-memory stand-in under dry-run)."""
        if self._robot is not None:
            return

        if self._dry_run:
            from soarm_sdk import NullRobot

            self._robot = NullRobot()
        else:
            if self._port is None:
                raise ValueError(
                    "No servo port specified. Pass --servo-port /dev/cu.usbserial-XXXX"
                )
            from soarm_sdk import LeRobotRobot

            self._robot = LeRobotRobot(
                port=self._port,
                motor_names=config.JOINT_NAMES,
                max_relative_target=config.MAX_RELATIVE_TARGET,
            )

        self._robot.connect()

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

    @property
    def port(self) -> Optional[str]:
        """Serial port. Assign before :meth:`connect` to retarget the arm."""
        return self._port

    @port.setter
    def port(self, value: Optional[str]) -> None:
        self._port = value

    @property
    def robot(self):
        """The underlying ``soarm_sdk`` robot (after :meth:`connect`)."""
        return self._robot

    # ------------------------------------------------------------------
    # State read-back
    # ------------------------------------------------------------------

    def get_joint_degrees(self) -> dict[str, float]:
        """Return present joint positions as ``{name.pos: degrees}``."""
        if self._robot is None:
            return {f"{n}.pos": 0.0 for n in config.JOINT_NAMES}
        if self._dry_run:
            q = self._robot.get_joint_positions()
            return {
                f"{n}.pos": float(np.degrees(v))
                for n, v in zip(config.JOINT_NAMES, q)
            }
        return self._robot.get_joint_degrees()

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

        Automatically appends the current gripper position so every command
        includes all 6 joints.
        """
        if self._robot is None:
            return
        full = dict(deg_dict)
        full["gripper.pos"] = (
            config.GRIPPER_OPEN_DEG
            if self._gripper_open
            else config.GRIPPER_CLOSED_DEG
        )
        if self._dry_run:
            q = np.array(
                [
                    np.radians(full.get(f"{n}.pos", 0.0))
                    for n in config.JOINT_NAMES
                ]
            )
            self._robot.set_joint_positions(q)
        else:
            self._robot.send_joint_degrees(full)

    def set_gripper(self, open_gripper: bool) -> None:
        """Set gripper state; applied on the next send_joint_degrees call."""
        self._gripper_open = open_gripper

    def toggle_gripper(self) -> None:
        """Toggle gripper between open and closed."""
        self.set_gripper(not self._gripper_open)

    @property
    def gripper_is_open(self) -> bool:
        return self._gripper_open
