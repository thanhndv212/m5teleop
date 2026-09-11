# Changelog

All notable changes to `m5teleop` are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

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
