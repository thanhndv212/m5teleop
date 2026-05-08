"""Central configuration for m5teleop."""

import os
from pathlib import Path

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

_HERE = Path(__file__).parent

# Absolute path to the SO100 URDF
URDF_PATH: str = str(
    _HERE.parent.parent / "SO-ARM100" / "Simulation" / "SO100" / "so100.urdf"
)

# Path to lerobot source (override via env var LEROBOT_SRC)
LEROBOT_SRC: str = os.environ.get(
    "LEROBOT_SRC",
    str(Path.home() / "Develop" / "openarm-ws" / "lerobot" / "src"),
)

# ---------------------------------------------------------------------------
# Serial ports  (set to None for auto-detect)
# ---------------------------------------------------------------------------

IMU_PORT: str | None = None  # M5StickC Plus (auto-detected)
SERVO_PORT: str | None = None  # SO100 servo bus (set via --servo-port)

IMU_BAUDRATE: int = 115_200
CONTROL_HZ: int = 50  # Target control loop frequency

# ---------------------------------------------------------------------------
# IMU → Twist gains and filters
# ---------------------------------------------------------------------------

# Low-pass filter coefficient (0 = frozen, 1 = no filtering)
LPF_ALPHA: float = 0.3

# Dead-zones (in radians for tilt, rad/s for gyro)
TILT_DEADZONE: float = 0.04  # rad
GYRO_DEADZONE: float = 0.10  # rad/s

# Scaling: tilt angle (rad) → linear EE velocity (m/s)
LIN_GAIN: float = 0.08

# Scaling: gyro rate (rad/s) → angular EE velocity (rad/s)
ANG_GAIN: float = 0.5

# Saturation limits
MAX_LIN_VEL: float = 0.12  # m/s
MAX_ANG_VEL: float = 1.2  # rad/s

# ---------------------------------------------------------------------------
# Robot / servo
# ---------------------------------------------------------------------------

# Joint names as used by lerobot SO100Follower (in order 1-6)
JOINT_NAMES: list[str] = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

# EE frame name in the URDF
EE_FRAME: str = "gripper"

# Gripper positions in lerobot degrees (motor 6, range 0–100 %)
GRIPPER_OPEN_DEG: float = 10.0
GRIPPER_CLOSED_DEG: float = 80.0

# Max allowed joint step per cycle (degrees) – safety clamp in lerobot
MAX_RELATIVE_TARGET: float = 5.0

# IK – PostureTask regularisation weight
POSTURE_COST: float = 1e-3

# IK – FrameTask costs [position_cost, orientation_cost]
EE_POSITION_COST: float = 1.0
EE_ORIENTATION_COST: float = 0.5

# IK solver
IK_SOLVER: str = "quadprog"

# ---------------------------------------------------------------------------
# EKF attitude filter
# ---------------------------------------------------------------------------

EKF_SIGMA_GYRO: float = 0.005   # rad/s gyro noise std
EKF_SIGMA_BIAS: float = 0.0001  # rad/s² bias-drift noise std
EKF_SIGMA_ACC:  float = 0.05    # g accelerometer noise std
EKF_ACC_GATE:   float = 0.30    # skip accel update if |a_norm - 1| > this (g)

# ---------------------------------------------------------------------------
# Orientation cascade controller
# ---------------------------------------------------------------------------

ORIENT_KP_OUTER: float = 2.5   # quaternion error → angular velocity (1/s)
ORIENT_KP_INNER: float = 0.5   # inner velocity error gain
ORIENT_MAX_OMEGA: float = 1.2  # rad/s clamp on output command

# ---------------------------------------------------------------------------
# Simulation (viser)
# ---------------------------------------------------------------------------

VISER_HOST: str = "localhost"
VISER_PORT: int = 8080

# ---------------------------------------------------------------------------
# Rerun
# ---------------------------------------------------------------------------

RERUN_SESSION: str = "m5teleop"
