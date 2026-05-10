# M5StickC Plus → SO-ARM100 IMU Teleoperation

## Overview

Teleoperate the SO-ARM100 robot arm by tilting and rotating an **M5StickC Plus 1.1**.
IMU data (MPU6886) is streamed over USB serial, filtered through an **Error-State Kalman
Filter (ESKF)**, and tracked by a **two-layer cascade orientation controller** that drives
the EE orientation to follow the IMU frame. Joint positions are resolved via differential
IK (pink + pinocchio) at **50 Hz**. The pipeline runs simultaneously on **real hardware**
(via lerobot SO100Follower) and in a **3-D browser simulation** (viser). All data is
logged to **Rerun** for analysis.

```
M5StickC Plus 1.1
 ├── MPU6886 (accel + gyro)
 │    └─► ImuEKF (ESKF attitude filter + ZARU bias correction)
 │              └─► OrientationController (cascade P-P, quaternion)
 │                        └─► IKSolver (pink / pinocchio)
 └── BTN_A / BTN_B                │
      ├─ toggle teleop ───────────┤
      └─ toggle gripper    ┌──────┴──────┐
                      ArmInterface   SimInterface
                    (SO100Follower)  (viser browser)
                      real servos     3-D viewer
                           │              │
                           └── viz.py (Rerun) ──┘
```

---

## Repository Layout

```
soarm-ws/
├── m5imu/                         Phase 1–4 – IMU driver + firmware
│   ├── firmware/
│   │   └── m5imu_firmware/
│   │       └── m5imu_firmware.ino ← Flash to M5StickC Plus 1.1
│   └── m5imu/
│       ├── imu_data.py            ← ImuData dataclass (accel, gyro, temp,
│       │                              btn_a, btn_b; pitch/roll/yaw kept as
│       │                              0.0 defaults for backward-compat)
│       └── reader.py              ← ImuReader (USB serial, JSON, 100 Hz)
│
├── m5teleop/                      Phase 3–11 – Teleop package
│   ├── m5teleop/
│   │   ├── __init__.py
│   │   ├── config.py              ← All tunable parameters
│   │   ├── lpf.py                 ← Centralised EMA low-pass filter (Lpf)
│   │   ├── imu_ekf.py             ← ESKF attitude filter + ZARU
│   │   ├── orient_controller.py   ← Two-layer cascade P-P controller
│   │   ├── imu_twist.py           ← Legacy IMU → 6D twist (kept, unused)
│   │   ├── ik_solver.py           ← pink IK wrapper (pinocchio)
│   │   ├── arm_interface.py       ← lerobot SO100Follower wrapper
│   │   ├── sim_interface.py       ← viser 3-D simulation + GUI
│   │   └── viz.py                 ← Rerun data logger
│   ├── teleop.py                  ← Main entry point (50 Hz loop)
│   └── requirements.txt
│
└── SO-ARM100/
    └── Simulation/SO100/so100.urdf  ← Robot model (6-DOF)
```

---

## Environment Setup

```bash
# All commands run in the gosim conda environment
conda activate gosim

# Install dependencies (once)
conda install -c conda-forge pinocchio -y
pip install pin-pink quadprog viser rerun-sdk pyserial loop-rate-limiters
```

---

## Implementation Phases

### Phase 1 — Firmware ✅
**File:** `m5imu/firmware/m5imu_firmware/m5imu_firmware.ino`

**Current state — minimal raw IMU streamer (CF/orientation stripped):**

- MPU6886 reads accelerometer + gyroscope at ~100 Hz
- JSON output over USB serial (115200 baud):
  `{"ax":.., "ay":.., "az":.., "gx":.., "gy":.., "gz":.., "temp":.., "btnA":0, "btnB":0}`
  — no `pitch/roll/yaw`; orientation is handled entirely by the Python ESKF
- **Gyro bias calibration** at startup: averages 500 samples (~2.5 s), subtracts per-axis bias before streaming. LCD shows `CAL gyro...` with progress counter.
- **Button detection** latches `wasPressed()` so short taps are never missed at 100 Hz
- **LCD display** (landscape): Hz / Temperature / `Bz` (Z-gyro bias) / sample count / button states
- **Timing stability:** `delay(1)` when idle prevents tight-loop `M5.update()` calls from saturating the I2C bus between samples (~100 kHz idle spin → ~1 kHz). LCD update is skipped in the same iteration as an IMU sample to prevent SPI writes stalling the next sample.

**How to flash:**
1. Open `m5imu_firmware.ino` in Arduino IDE
2. Board: **M5Stick-C-Plus** · Port: your M5StickC USB serial
3. Upload — LCD shows **IMU OK**, then **CAL gyro...** (hold still ~2.5 s), then live Hz/T/Bz

**Verify:**
```bash
conda activate gosim
cd soarm-ws/m5imu
python m5imu/reader.py --debug
# RAW: b'{"ax":...,"gz":...,"temp":...,"btnA":0,"btnB":0}\n'
```

---

### Phase 2 — ImuData + ImuReader ✅
**Files:** `m5imu/m5imu/imu_data.py`, `m5imu/m5imu/reader.py`

**What was done:**
- `ImuData` dataclass: `ax, ay, az, gx, gy, gz, temp, btn_a, btn_b`; `pitch/roll/yaw` kept as `0.0` defaults for backward-compatibility but not populated by current firmware
- `ImuReader` parses JSON; mandatory keys: `ax…gz, temp, btnA, btnB`; optional `pitch/roll/yaw` via `.get()` with 0.0 fallback
- Serial opened with `dsrdtr=False, rtscts=False` to suppress auto-reset on macOS
- 0.1 s sleep after open + `reset_input_buffer()` to discard stale bytes

---

### Phase 2b — Low-Pass Filter (`lpf.py`) ✅
**File:** `m5teleop/m5teleop/lpf.py`

Centralised single source of truth for all raw IMU smoothing across the pipeline.

- `Lpf(channels, alpha)` — first-order EMA: `y = α·x + (1−α)·y_prev`
- Seeds from first sample → no startup transient
- `reset()` clears state (call when teleop re-enables after a pause)
- `alpha` defaults to `config.LPF_ALPHA = 0.3`
- Replaces inline numpy EMA in `imu_twist.py` and private `_Lpf` dict class in `tune_ekf.py`

```python
from m5teleop.lpf import Lpf
lpf = Lpf(channels=6)           # ax, ay, az, gx, gy, gz
filtered = lpf.update(raw_arr)  # numpy array in, same shape out
```

---

### Phase 3a — Config ✅
**File:** `m5teleop/m5teleop/config.py`

Key parameters:

| Group | Parameter | Default | Description |
|-------|-----------|---------|-------------|
| EKF | `EKF_SIGMA_GYRO` | 0.005 rad/s | Gyro noise std |
| EKF | `EKF_SIGMA_BIAS` | 0.0001 rad/s² | Bias-drift noise std |
| EKF | `EKF_SIGMA_ACC` | 0.05 g | Accelerometer noise std |
| EKF | `EKF_ACC_GATE` | 0.30 g | Skip accel update if `|‖a‖−1| >` this |
| EKF | `EKF_ZARU_THRESHOLD` | 0.08 rad/s | ZARU active when `|ω| <` this |
| EKF | `EKF_SIGMA_ZARU` | 0.02 rad/s | ZARU measurement noise |
| Controller | `ORIENT_KP_OUTER` | 2.5 1/s | Outer error→ω gain |
| Controller | `ORIENT_KP_INNER` | 0.5 — | Inner velocity-error gain |
| Controller | `ORIENT_MAX_OMEGA` | 1.2 rad/s | Output clamp |
| IK | `POSTURE_COST` | 0.1 | Neutral-pose regularisation |
| Loop | `CONTROL_HZ` | 50 | Loop rate |

---

### Phase 3b — IKSolver ✅
**File:** `m5teleop/m5teleop/ik_solver.py`

- Loads `so100.urdf` with `pin.buildModelFromUrdf` (kinematics-only; no mesh needed)
- `pink.FrameTask("jaw")` + `PostureTask` as regulariser
- `step(twist, dt)` integrates EE target SE3, solves QP, returns joint config (rad)
- `q_to_degrees()` → lerobot-compatible dict (5 revolute joints; gripper handled separately)
- `get_ee_pose()` → `pin.SE3` for controller feedback

---

### Phase 3c — ArmInterface ✅
**File:** `m5teleop/m5teleop/arm_interface.py`

- Wraps `lerobot SOFollowerRobotConfig(port, use_degrees=True, max_relative_target=5.0)`
- `send_joint_degrees(dict)` always merges `gripper.pos` (0 = closed, 100 = open)
- `toggle_gripper()` → BTN_B handler
- `get_joint_radians()` → seeds IK from current physical pose on startup

---

### Phase 3d — SimInterface (viser) ✅
**File:** `m5teleop/m5teleop/sim_interface.py`

- `ViserUrdf(server, Path(urdf_path))` loads robot into browser scene
- GUI panels: **Connection** (IMU port + servo port text boxes with connect/disconnect buttons) and **Teleoperation** (status display + **⊕ Zero IMU** reset button)
- `update(q, teleop_active, gripper_open)` updates joint angles at 50 Hz
- **Gripper joint fix:** the IK solver only writes joints 0–4; `q[5]` is always 0.0 from the IK output. `_run()` copies the cfg slice and overwrites the last element with `np.radians(GRIPPER_OPEN/CLOSED_DEG)` based on `gripper_open` before calling `update_cfg` — otherwise the jaw never moves in the viewer.
- `update_imu_frame(pitch, roll, yaw)` updates a floating coordinate frame (EKF orientation)
- `on_zero_reset: Callable` callback — called when Zero IMU button is pressed; wired to `_do_zero_reset()` in `teleop.py`
- `on_imu_connect / on_imu_disconnect / on_servo_connect / on_servo_disconnect` callbacks push commands to a thread-safe queue so the main loop handles all serial I/O

---

### Phase 3e — TeleopVisualizer (Rerun) ✅
**File:** `m5teleop/m5teleop/viz.py`

Logged channels:

| Path | Content |
|------|---------|
| `imu/accel/{x,y,z}` | Accelerometer (g) |
| `imu/gyro/{x,y,z}` | Gyroscope (°/s) |
| `imu/temp` | Temperature (°C) |
| `imu/orientation/{pitch,roll,yaw}` | Firmware CF angles (°) — always 0.0 with current firmware |
| `imu/frame` | EKF body-frame axes — Arrows3D (RGB) |
| `ee_target/frame` | Controller target frame — Arrows3D (orange) |
| `ekf/orientation/{roll,pitch,yaw}` | EKF attitude estimate (°) |
| `ekf/bias/{x,y,z}` | Estimated gyro bias (°/s) |
| `orient/error/{x,y,z,norm}` | Tracking error rotation-vector (rad) |
| `orient/error/arrow` | Error as 3-D arrow |
| `orient/omega_actual/{x,y,z}` | EE angular velocity (rad/s) — only when teleop active |
| `twist/{vx…wz}` | 6-D EE twist command |
| `joints/<name>` | Joint angles (rad) |
| `ee/{x,y,z}`, `ee/position` | EE position (m) |
| `buttons/btnA`, `buttons/btnB` | Button state |
| `state/teleop_active`, `state/gripper_open` | Binary state |

**IMU frame vs EE target frame:** both are placed as 3-D arrows in the spatial panel.
After a zero-reset the two frames are identical; any divergence reveals tracking error visually.

---

### Phase 3f — Main Loop (`teleop.py`) ✅
**File:** `m5teleop/teleop.py`

**Control flow (50 Hz):**
1. Drain connection-command queue from viser GUI (non-blocking)
2. Get latest `ImuData` from background thread queue (non-blocking)
3. Edge-detect BTN_A → toggle `teleop_active` (+ auto zero-reset on activation); BTN_B → toggle gripper
4. **EKF step** every tick (even when teleop is off): `ekf.step(ax…gz, dt)` → quaternion
5. If teleop active: `orient_ctrl.compute_twist(ekf.quaternion, _get_ee_quaternion(), dt)` → twist + error
6. `q_new = ik.step(twist, dt)` (zero twist when inactive)
7. `arm.send_joint_degrees(deg_dict)` — real hardware
8. `sim.update(q_new, ...)` + `sim.update_imu_frame(...)` — viser
9. `viz.log_all_with_tracking(...)` — Rerun

---

### Phase 4 — ESKF Attitude Filter ✅
**File:** `m5teleop/m5teleop/imu_ekf.py`

**Error-State Kalman Filter** — 6-D error state: `[δθ (3), δb (3)]` (attitude error + gyro bias).

**State:** nominal quaternion `q̄` [w,x,y,z] + gyro bias `b̄` [rad/s].

| Step | Details |
|------|---------|
| Predict | Integrate `q̄` via exact rotation; propagate `P` with `F = [[I−[ω×]dt, −I dt], [0, I]]` |
| Accel update | Gravity direction error `ν = â − ĝ`; `H = [ĝ×, 0]`; Joseph-form `P` update. Gated: skip when `abs(‖a‖−1) > acc_gate` (free-fall / shock) |
| **ZARU** | When `abs(ω_measured) < zaru_threshold` (device stationary): inject pseudo-measurement `ω_true ≈ 0` → `H = [0, I]`, `ν = ω_meas − b̄`. Drives Z-axis bias identification → **eliminates yaw drift** |

**Properties:** `quaternion`, `rotation_matrix`, `euler_deg` (roll, pitch, yaw), `bias_dps`.

**Benchmark:** with 2 dps Z-axis bias, 10 s stationary: without ZARU → 20° yaw drift; with ZARU → 0.05° drift, bias identified to 0.001 dps.

---

### Phase 5 — Cascade Orientation Controller ✅
**File:** `m5teleop/m5teleop/orient_controller.py`

**Two-layer cascade P-P controller** — gimbal-lock-free, quaternion-based.

```
q_imu, q_ee  →  zero_reset()  →  stores q_imu_ref, q_ee_ref
                                  q_target = q_ee_ref  (error = 0 at reset)

Per tick:
  q_delta  = q_imu_ref⁻¹ ⊗ q_imu          (IMU rotation since reset)
  q_target = q_ee_ref ⊗ q_delta            (desired EE orientation)

Layer 1 (outer): q_err = q_target ⊗ q_ee⁻¹
                 err_vec = rotvec(q_err)    (ℝ³, ‖err‖ ∈ [0,π])
                 ω_set = Kp_outer × err_vec

Layer 2 (inner): ω_actual = vee(Ṙ Rᵀ)     (finite difference)
                 ω_cmd = ω_set + Kp_inner × (ω_set − ω_actual)
                 clamp ‖ω_cmd‖ ≤ max_omega
```

**zero_reset behaviour:** captures `q_imu_ref = q_imu`, `q_ee_ref = q_ee`, sets `q_target = q_ee_ref` so **orientation error is immediately zero**; the EE then smoothly follows any subsequent IMU motion.

**Properties:** `last_error` (rotation-vector, rad), `last_target` (quaternion), `last_target_rotation` (3×3 R).

---

### Phase 6 — Zero-Reset Button (viser) ✅

- **Viser GUI** → Teleoperation folder → **⊕ Zero IMU** button (blue)
- `sim.on_zero_reset` callback → `_do_zero_reset()` in `teleop.py`
- `_do_zero_reset()` calls `orient_ctrl.zero_reset(ekf.quaternion, _get_ee_quaternion())`
- Also triggered automatically on every BTN_A activation (teleop ON)

---

### Phase 7 — Yaw Drift Mitigation ✅

**Python ESKF ZARU only** — firmware CF/yaw code has been fully stripped:

The firmware no longer computes or streams pitch/roll/yaw. All orientation estimation — including yaw drift correction — is handled by the Python ESKF:

- **ZARU** (Zero Angular Rate Update): when `|ω_measured| < EKF_ZARU_THRESHOLD`, the EKF injects a pseudo-measurement `ω_true ≈ 0`, driving Z-axis bias identification to near-zero drift. This is the sole yaw correction mechanism.
- **Firmware gyro bias calibration** (500-sample average at boot) removes the bulk of the constant bias before streaming, giving ZARU a smaller residual to correct.

> The old firmware `YAW_DECAY` and `YAW_GYRO_THRESHOLD` constants have been removed. They only affected the firmware LCD display and were never used by the Python controller.

---

### Phase 8 — Rerun Tracking Fixes ✅

**Problem:** `imu/frame` arrows were computed from firmware CF Euler angles; the controller used `ekf.quaternion` — the displayed frame did not match what the controller tracked. No EE target frame was visible.

**Fixes:**
- `log_imu_frame(R)` now accepts a 3×3 rotation matrix (from EKF) directly
- `log_ee_target_frame(R)` added — logs controller's `last_target_rotation` as orange-tinted arrows placed next to IMU frame
- `log_all_with_tracking` derives EKF rotation matrix from `ekf_euler` before logging; suppresses stale `omega_actual` when teleop is inactive
- `orient_ctrl.last_target` / `last_target_rotation` exposed as properties

**In Rerun after these fixes:**
- `imu/frame` ↔ `ee_target/frame` are **visually identical** immediately after zero-reset
- Any separation between them directly represents tracking error

---

### Phase 9 — ESKF Parameter Tuning Script ✅
**File:** `m5teleop/tune_ekf.py`

**Purpose:** find the EKF parameter set that minimises stationary yaw drift and pitch/roll error on real or synthetic IMU data.

**Modes:**

| Mode | Description |
|------|-------------|
| `live` | Stream live orientation, bias, ZARU status; shows yaw deviation accumulating in real time |
| `stationary` | Record N s of still data; score current `config.py` params |
| `sweep` | Grid-search ~240 combinations offline; print top-5 + config snippet |
| `optimize` | Nelder-Mead fine-tune from sweep winner (requires `scipy`) |

**Scoring function (lower = better):**
```
score = 3·yaw_peak_to_peak + pitch_rms + roll_rms + 0.5·|bias_z_final| + 0.3·converge_fraction
```

**Sweep grid** (fixed: `sigma_acc`, `acc_gate`):

| Parameter | Values searched |
|-----------|----------------|
| `sigma_gyro` | 0.002, 0.005, 0.010, 0.020 |
| `sigma_bias` | 0.00005, 0.0001, 0.0003, 0.0008 |
| `zaru_threshold` | 0.04, 0.06, 0.08, 0.12, 0.18 rad/s |
| `sigma_zaru` | 0.01, 0.02, 0.04 |

→ 4 × 4 × 5 × 3 = **240 combinations**, ~1.5 min on a 60 s recording.

**Offline workflow (recommended):**
```bash
conda activate gosim
cd soarm-ws/m5teleop

# 1. Record once (device must be completely still)
python tune_ekf.py stationary --duration 90 --save rec.npz

# 2. Grid search (offline, fast)
python tune_ekf.py sweep --load rec.npz

# 3. Fine-tune (optional, requires scipy)
python tune_ekf.py optimize --load rec.npz
```

**Dry-run (no hardware):**
```bash
python tune_ekf.py live --dry-run            # watch live with synthetic bias
python tune_ekf.py sweep --dry-run --duration 90
```

**Benchmark on synthetic data** (1.5 dps Z-bias, 30 s):
- Baseline (default config): score 15.77, yaw drift 4.4°/min
- After sweep: score 7.18, yaw drift **1.6°/min**  (+54% improvement)
- Bias convergence improves when `sigma_bias` and `sigma_zaru` are tuned together

---

## Running

### Dry-run (simulation + Rerun only, no hardware needed)

```bash
conda activate gosim
cd soarm-ws/m5teleop
python teleop.py --dry-run
# Open http://localhost:8080 in a browser
```

### Simulation + IMU (no arm)

```bash
python teleop.py --no-rerun
# Plug in M5StickC, firmware flashed, auto-detects port
```

### Full pipeline (hardware + simulation + Rerun)

```bash
# Find your servo port:
ls /dev/cu.usbserial-*   # the one NOT used by the M5StickC

python teleop.py --servo-port /dev/cu.usbserial-XXXX
```

### Runtime controls

| Control | Action |
|---------|--------|
| BTN_A (side) | Toggle teleoperation ON/OFF; auto zero-resets on activation |
| BTN_B (top) | Toggle gripper open/closed |
| **⊕ Zero IMU** (viser GUI) | Manual zero-reset — aligns EE target to current EE pose |
| Ctrl-C | Graceful shutdown |

### CLI flags

| Flag | Default | Description |
|------|---------|-------------|
| `--imu-port` | auto-detect | M5StickC serial port |
| `--servo-port` | None | SO100 servo bus port |
| `--dry-run` | False | Skip all serial, IMU feeds zeros |
| `--no-sim` | False | Disable viser window |
| `--no-rerun` | False | Disable Rerun logging |
| `--no-rerun-spawn` | False | Don't auto-launch Rerun viewer |
| `--hz` | 50 | Control loop frequency |

---

## Tuning Guide

**Arm oscillates / overshoots** → lower `ORIENT_KP_OUTER` (e.g. 1.5) or `ORIENT_MAX_OMEGA`

**Arm too sluggish** → raise `ORIENT_KP_OUTER`; if still slow, raise `ORIENT_KP_INNER`

**Small tremors when holding still** → raise `EKF_ZARU_THRESHOLD` to suppress more motion;
lower `EKF_SIGMA_ZARU` to trust ZARU more

**EKF yaw drifts in controller** → ensure ZARU is active: verify `|ω| < EKF_ZARU_THRESHOLD` when stationary. Raise `EKF_SIGMA_GYRO` if gyro is noisier than expected.

**IK jumps to strange poses** → increase `POSTURE_COST` in `config.py`

**Servo overload warning** → lower `MAX_RELATIVE_TARGET` in `config.py`

**Tracking error stays large after zero-reset** → confirm zero-reset prints `[OrientController] Zero reset` in console; if not, `on_zero_reset` callback is not connected

---

## Dependencies

| Package | Version | Purpose |
|---------|---------|---------|
| `pinocchio` | ≥4.0 (conda-forge) | Rigid-body kinematics |
| `pin-pink` | ≥4.2 (pip) | Differential IK QP solver |
| `quadprog` | ≥0.1.12 | QP backend for pink |
| `viser` | ≥1.0 | Browser-based 3-D robot viewer |
| `rerun-sdk` | ≥0.26 | Time-series + spatial data viz |
| `pyserial` | ≥3.5 | USB serial (IMU + servos) |
| `numpy` | — | Linear algebra (EKF, controller) |
| lerobot | openarm-ws | SO100Follower robot interface |

> **Note:** use `pin.buildModelFromUrdf(urdf_path)` (kinematics-only). The mesh-loading
> variant `buildModelsFromUrdf` fails when STL paths are not resolvable.

---

## Known Issues / TODOs

- [ ] Calibrate SO100 with lerobot before first real run (`SOFollower.calibrate()`)
- [ ] Find and pass `--servo-port` (run `ls /dev/cu.usbserial-*` with arm plugged in)
- [ ] No magnetometer → yaw still drifts during sustained rotation; ZARU only corrects bias at rest. Consider offline heading reference (ArUco marker, visual odometry) for long sessions.
- [ ] Linear position teleoperation not yet implemented (controller drives orientation only); EE position remains at reset-time value
- [ ] `--record` flag for saving a lerobot dataset during a run
