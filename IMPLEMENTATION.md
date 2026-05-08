# M5StickC Plus → SO-ARM100 IMU Teleoperation

## Overview

Teleoperate the SO-ARM100 robot arm by tilting and rotating an **M5StickC Plus 1.1**.
IMU data (MPU6886) is streamed over USB serial, converted to an end-effector twist,
and resolved into joint positions via differential IK (pink + pinocchio) at **50 Hz**.
The pipeline runs simultaneously on **real hardware** (via lerobot SO100Follower) and
in a **3-D browser simulation** (viser). All data is logged to **Rerun** for analysis.

```
M5StickC Plus 1.1
 ├── MPU6886 (accel + gyro)  ──► ImuTwistConverter
 └── BTN_A / BTN_B           ──►  toggle teleop / gripper
                                        │
                                  IKSolver (pink)
                                        │
                         ┌─────────────┴─────────────┐
                   ArmInterface                 SimInterface
                 (SO100Follower)               (viser browser)
                   real servos                  3-D viewer
                         │                          │
                         └────────── viz.py (Rerun) ┘
```

---

## Repository Layout

```
soarm-ws/
├── m5imu/                         Phase 1 & 2 – IMU driver
│   ├── firmware/
│   │   └── m5imu_firmware/
│   │       └── m5imu_firmware.ino ← Flash to M5StickC Plus 1.1
│   └── m5imu/
│       ├── imu_data.py            ← ImuData dataclass (+ btn_a, btn_b)
│       └── reader.py              ← ImuReader (sync / callback)
│
├── m5teleop/                      Phase 3 – Teleop package
│   ├── m5teleop/
│   │   ├── __init__.py
│   │   ├── config.py              ← All tunable parameters
│   │   ├── imu_twist.py           ← IMU → 6D twist (LPF + deadzone)
│   │   ├── ik_solver.py           ← pink IK wrapper (pinocchio)
│   │   ├── arm_interface.py       ← lerobot SO100Follower wrapper
│   │   ├── sim_interface.py       ← viser 3-D simulation
│   │   └── viz.py                 ← Rerun data logger
│   ├── teleop.py                  ← Main entry point
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

**What was done:**
- Added `M5.BtnA.isPressed()` and `M5.BtnB.isPressed()` to the serial JSON output
- JSON now includes `"btnA":0/1,"btnB":0/1` on every line (~100 Hz)
- `M5.update()` is called each loop to latch button state correctly

**How to flash:**
1. Open `m5imu_firmware.ino` in Arduino IDE
2. Select board: **M5Stick-C-Plus**, port: your M5StickC USB port
3. Upload
4. LCD should show **IMU OK** and live sensor values

**Verify:**
```bash
conda activate gosim
cd soarm-ws/m5imu
python m5imu/reader.py --debug
# Should see: RAW: b'{"ax":...,"btnA":0,"btnB":0}\n'
```

---

### Phase 2 — ImuData + ImuReader ✅
**Files:** `m5imu/m5imu/imu_data.py`, `m5imu/m5imu/reader.py`

**What was done:**
- Added `btn_a: bool = False` and `btn_b: bool = False` to `ImuData`
- `ImuReader._parse()` extracts `btnA`/`btnB` from JSON (defaults `False` if absent)
- Backward compatible with older firmware that lacks button fields

**Verify:**
```python
from m5imu import ImuReader
with ImuReader() as r:
    s = r.read_one()
    print(s.btn_a, s.btn_b)
```

---

### Phase 3a — Config + ImuTwistConverter ✅
**Files:** `m5teleop/m5teleop/config.py`, `m5teleop/m5teleop/imu_twist.py`

**Config key parameters (edit `config.py` to tune):**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `LPF_ALPHA` | 0.3 | Low-pass filter (0=frozen, 1=raw) |
| `TILT_DEADZONE` | 0.04 rad | Ignore small tilts |
| `GYRO_DEADZONE` | 0.10 rad/s | Ignore small gyro drift |
| `LIN_GAIN` | 0.08 m/s/rad | Tilt → EE linear velocity |
| `ANG_GAIN` | 0.5 rad/s/rad/s | Gyro → EE angular velocity |
| `MAX_LIN_VEL` | 0.12 m/s | Saturation |
| `MAX_ANG_VEL` | 1.2 rad/s | Saturation |
| `CONTROL_HZ` | 50 | Loop rate |

**Twist mapping:**
- Hold the M5StickC vertically. Tilt forward/back → EE moves forward/back.
- Tilt left/right → EE moves laterally.
- Rotate the device (gyro) → EE rotates correspondingly.

---

### Phase 3b — IKSolver ✅
**File:** `m5teleop/m5teleop/ik_solver.py`

**What was done:**
- Loads `SO-ARM100/Simulation/SO100/so100.urdf` via pinocchio
- Uses `pink.FrameTask` on the `"jaw"` frame + `PostureTask` for regularisation
- `step(twist, dt)` integrates the EE target, solves QP, returns new joint config (rad)
- `q_to_degrees()` converts pinocchio config to lerobot-compatible degrees dict

**IK tasks:**
- `FrameTask("jaw")`: tracks the commanded EE pose
- `PostureTask`: pulls joints towards neutral to avoid singularities

---

### Phase 3c — ArmInterface ✅
**File:** `m5teleop/m5teleop/arm_interface.py`

**What was done:**
- Wraps `lerobot.robots.so_follower.so_follower.SOFollower`
- `connect()` / `disconnect()` lifecycle
- `get_joint_radians()` → reads present positions for IK seeding
- `send_joint_degrees(dict)` → sends IK result to the physical servos
- `toggle_gripper()` → BTN_B handler

**Dry-run mode:** pass `--dry-run` to skip serial entirely for offline testing.

---

### Phase 3d — SimInterface (viser) ✅
**File:** `m5teleop/m5teleop/sim_interface.py`

**What was done:**
- Starts a `viser.ViserServer` in a background thread
- Loads the SO100 URDF into the browser scene via `add_robot_urdf()`
- Updates all joint angles at 50 Hz via `update(q, ...)`
- Shows an EE frame marker (orange axes) at the current EE pose
- GUI panel shows teleop status and gripper state

**View:** open `http://localhost:8080` in a browser while teleop is running.

---

### Phase 3e — TeleopVisualizer (Rerun) ✅
**File:** `m5teleop/m5teleop/viz.py`

**Logged channels:**

| Path | Content |
|------|---------|
| `imu/accel/{x,y,z}` | Accelerometer (g) |
| `imu/gyro/{x,y,z}` | Gyroscope (°/s) |
| `imu/temp` | Temperature (°C) |
| `buttons/btnA`, `buttons/btnB` | Button state |
| `twist/{vx,vy,vz,wx,wy,wz}` | EE twist command |
| `joints/{shoulder_pan,...}` | Joint angles (rad) |
| `ee/{x,y,z}` | EE position (m) |
| `ee/position` | 3-D point (spatial panel) |
| `state/teleop_active` | 0/1 |
| `state/gripper_open` | 0/1 |

**View:** open the Rerun viewer or use `--rerun-spawn` to auto-launch it.

---

### Phase 3f — Main Loop (`teleop.py`) ✅
**File:** `m5teleop/teleop.py`

**Control flow (50 Hz):**
1. Get latest `ImuData` from background queue (non-blocking)
2. Edge-detect BTN_A → toggle `teleop_active`; BTN_B → toggle gripper
3. If active: compute twist from IMU; else: zero twist
4. `q_new = ik.step(twist, dt)`
5. `arm.send_joint_degrees(deg_dict)` (real hardware)
6. `sim.update(q_new)` (viser browser)
7. `viz.log_all(...)` (Rerun)

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
python teleop.py --no-rerun     # with viser only
# Plug in M5StickC, flash firmware first
```

### Full pipeline (hardware + simulation + Rerun)

```bash
# Find your servo port:
ls /dev/cu.usbserial-*   # look for the one NOT used by M5StickC

python teleop.py --servo-port /dev/cu.usbserial-XXXX
```

### CLI flags

| Flag | Default | Description |
|------|---------|-------------|
| `--imu-port` | auto-detect | M5StickC serial port |
| `--servo-port` | None | SO100 servo bus port |
| `--dry-run` | False | Skip all serial, IMU feeds zeros |
| `--no-sim` | False | Disable viser window |
| `--no-rerun` | False | Disable Rerun logging |
| `--rerun-spawn` | False | Auto-launch Rerun viewer |
| `--hz` | 50 | Control loop frequency |

---

## Tuning Guide

**Arm moves too fast / oscillates** → lower `LIN_GAIN`, `ANG_GAIN` in `config.py`

**Arm drifts even when still** → increase `TILT_DEADZONE` and `GYRO_DEADZONE`

**Lag / sluggish response** → increase `LPF_ALPHA` (less filtering)

**IK jumps to strange poses** → increase `POSTURE_COST` (stronger neutral pull)

**Servo overload warning** → lower `MAX_RELATIVE_TARGET` in `config.py`

---

## Dependencies

| Package | Version | Purpose |
|---------|---------|---------|
| `pinocchio` | ≥3.0 (conda-forge) | Rigid-body kinematics |
| `pin-pink` | ≥4.2 | Differential IK QP solver |
| `quadprog` | ≥0.1.12 | QP backend for pink |
| `viser` | ≥1.0 | Browser-based 3-D robot viewer |
| `rerun-sdk` | ≥0.24 | Time-series + spatial data viz |
| `pyserial` | ≥3.5 | USB serial (IMU + servos) |
| lerobot | openarm-ws | SO100Follower robot interface |

---

## Known Issues / TODOs

- [ ] Calibrate SO100 with lerobot before first run (`SOFollower.calibrate()`)
- [ ] Verify `SERVO_PORT` in `config.py` or always pass `--servo-port`
- [ ] `viser.add_robot_urdf` requires mesh files next to URDF — check that STL paths resolve
- [ ] Gravity compensation: tilt due to gravity vs. intentional tilt not yet separated; consider complementary filter or Madgwick
- [ ] Add `--record` flag to save lerobot dataset during teleoperation
