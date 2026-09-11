"""Unit tests for m5teleop.lerobot_soarm_interface.ArmInterface.

Runs entirely on the dry-run path (soarm_sdk.NullRobot), so it needs no
serial port, no lerobot, and none of the heavy IK dependencies.
"""

from __future__ import annotations

import numpy as np
import pytest

from m5teleop import config
from m5teleop.lerobot_soarm_interface import ArmInterface

ARM_JOINTS = config.JOINT_NAMES[:5]


def _arm_degrees(value: float = 10.0) -> dict[str, float]:
    """A command in the shape the IK solver emits: the 5 arm joints."""
    return {f"{n}.pos": value for n in ARM_JOINTS}


def test_dry_run_uses_the_sdk_null_backend():
    from soarm_sdk import NullRobot, RobotInterface

    arm = ArmInterface(dry_run=True)
    arm.connect()
    assert isinstance(arm.robot, NullRobot)
    assert isinstance(arm.robot, RobotInterface)
    arm.disconnect()


def test_dry_run_starts_at_the_configured_home_pose():
    arm = ArmInterface(dry_run=True)
    arm.connect()
    q = arm.get_joint_radians()
    assert q.shape == (5,)
    # Home pose from soarm_sdk's configs/soarm100.yaml, not a zero vector.
    assert not np.allclose(q, 0.0)
    arm.disconnect()


def test_read_back_before_connect_is_zeros():
    arm = ArmInterface(dry_run=True)
    np.testing.assert_allclose(arm.get_joint_radians(), np.zeros(5))


def test_commands_are_tracked_under_dry_run():
    arm = ArmInterface(dry_run=True)
    arm.connect()
    arm.send_joint_degrees(_arm_degrees(10.0))
    np.testing.assert_allclose(
        arm.get_joint_radians(), np.full(5, np.radians(10.0)), atol=1e-9
    )
    arm.disconnect()


def test_gripper_position_is_appended_to_every_command():
    arm = ArmInterface(dry_run=True)
    arm.connect()

    arm.send_joint_degrees(_arm_degrees())
    assert arm.gripper_is_open is True
    assert arm.get_joint_degrees()["gripper.pos"] == pytest.approx(
        config.GRIPPER_OPEN_DEG
    )

    arm.toggle_gripper()
    arm.send_joint_degrees(_arm_degrees())
    assert arm.gripper_is_open is False
    assert arm.get_joint_degrees()["gripper.pos"] == pytest.approx(
        config.GRIPPER_CLOSED_DEG
    )
    arm.disconnect()


def test_set_gripper_applies_on_next_command_not_immediately():
    arm = ArmInterface(dry_run=True)
    arm.connect()
    arm.send_joint_degrees(_arm_degrees())
    arm.set_gripper(False)
    # Not yet sent — still reads back the previously commanded position.
    assert arm.get_joint_degrees()["gripper.pos"] == pytest.approx(
        config.GRIPPER_OPEN_DEG
    )
    arm.send_joint_degrees(_arm_degrees())
    assert arm.get_joint_degrees()["gripper.pos"] == pytest.approx(
        config.GRIPPER_CLOSED_DEG
    )
    arm.disconnect()


def test_port_property_retargets_before_connect():
    arm = ArmInterface(dry_run=True)
    arm.port = "/dev/somewhere-else"
    assert arm.port == "/dev/somewhere-else"


def test_hardware_path_without_a_port_raises():
    arm = ArmInterface(port=None)
    if config.SERVO_PORT is None:
        with pytest.raises(ValueError, match="No servo port specified"):
            arm.connect()


def test_connect_is_idempotent_and_disconnect_is_safe():
    arm = ArmInterface(dry_run=True)
    arm.connect()
    first = arm.robot
    arm.connect()
    assert arm.robot is first
    arm.disconnect()
    arm.disconnect()  # no-op, must not raise
    assert arm.robot is None


def test_context_manager():
    with ArmInterface(dry_run=True) as arm:
        assert arm.robot is not None
        assert arm.get_joint_radians().shape == (5,)
