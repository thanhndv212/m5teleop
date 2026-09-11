# Changelog

All notable changes to `m5teleop` are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed

- **`ArmInterface` now delegates transport to `soarm_sdk`** instead of
  wrapping lerobot's `SOFollower` itself: `soarm_sdk.LeRobotRobot` on
  hardware, `soarm_sdk.NullRobot` under `--dry-run`. Both satisfy
  `soarm_sdk.RobotInterface`, so teleop's arm is now the same kind of
  object a planner or an RL policy drives — the two servo stacks in this
  workspace stop being parallel universes. `soarm_sdk` is a new dependency;
  lerobot stays an optional one, still imported only when actually talking
  to hardware.
- `ArmInterface`'s public API is unchanged (`get_joint_degrees`,
  `get_joint_radians`, `send_joint_degrees`, the gripper toggle), so
  `teleop.py` needed no changes beyond using the new `arm.port` property
  instead of reaching into `arm._port`.
- **`--dry-run` now tracks commanded state** rather than reporting zeros:
  read-back reflects the last command, and the arm starts at the configured
  home pose instead of a zero vector. More representative offline, but it
  does mean IK is seeded from home rather than zeros in dry-run.
- The gripper stays driven by this package's `GRIPPER_OPEN_DEG` /
  `GRIPPER_CLOSED_DEG` in lerobot degrees, **not** through the SDK's new
  `Robot.set_gripper()`. lerobot's normalized gripper scale runs the
  opposite way to the jaw's URDF radians that the SDK's `gripper:` config
  block describes, and the jaw here is commanded open-loop from a button at
  50 Hz, where a read-back-based `gripper_is_open` would add a bus read to
  the control loop. Both reasons are written up in the module docstring.

### Added

- `tests/` — first tests for this package: `test_arm_interface.py` covers
  the arm transport on the dry-run path, so it needs no serial port, no
  lerobot, and none of the heavy IK dependencies.
- `LICENSE` (MIT) and a `license`/`authors` block in `pyproject.toml`,
  which previously declared neither.
- This file.

## [0.1.0] — 2026-07-14

Reconstructed from git history; this package had no changelog before now.

### Added

- The 50 Hz teleoperation loop (`teleop.py`) that is this workspace's
  integration point — the only package here that imports another one
  directly (`imu_sdk`). It chains: IMU → `ImuEKF` → `OrientationController`
  → `IKSolver` → `ArmInterface` (real servos) and `SimInterface` (viser),
  with everything logged to Rerun via `viz.py`.
- `imu_ekf.py` — error-state Kalman filter with ZARU bias correction.
- `orient_controller.py` — two-layer cascade P-P controller on quaternions.
- `ik_solver.py` — differential IK via `pink` and `pinocchio`.
- `lerobot_soarm_interface.py` — `ArmInterface`, wrapping lerobot's
  `SOFollower`. Note this reaches the servo bus through lerobot, **not**
  through `soarm_sdk`; the two are parallel ways to talk to the same bus,
  not layers.
- `config.py` — every tunable in one place (EKF noise and gate
  thresholds, LPF alpha, gains, joint names, gripper open/closed angles).
- `tune_ekf.py` — offline ESKF tuning tool: record a stationary session,
  then grid-search and optionally scipy-refine the noise parameters
  against it. Has live/stationary/sweep/optimize modes, writes results
  straight back into `config.py`, and spawns a Rerun viewer with the LPF
  output, a 3-D frame and raw sensors in live mode.
- `--record` flag, hooking in `soarm_lerobot.TeleopRecorder` to write
  LeRobotDataset episodes alongside the live loop. Gated behind an
  optional import so teleop still runs without `soarm_lerobot` or
  `lerobot` installed.
- Buttons: BTN_A toggles teleop (and delimits recorded episodes), BTN_B
  toggles the gripper.
- `IMPLEMENTATION.md` — phase-by-phase build log and tuning guide.
  Worth reading before touching the EKF or the controller: it records
  *why* each parameter holds its current value.

### Changed

- Centralised the low-pass filter into `lpf.py`; it had been duplicated
  everywhere raw IMU data needed smoothing.
- Import path updated for the `m5imu` → `imu_sdk` rename, and again for
  `soarm_learn` → `soarm_lerobot`.
- Dependencies moved from `requirements.txt` to `pyproject.toml`.
- `imu_twist.py`, the original IMU→6D-twist converter, is kept but is no
  longer used by the cascade-controller design that replaced it.

### Fixed

- Yaw drift, via ZARU in the EKF identifying and removing Z-axis gyro bias.
- Rerun tracking: EKF frame display, the end-effector target frame, and
  zero-reset alignment.
- The gripper joint not moving in simulation when BTN_B was pressed.
