"""m5teleop – IMU teleoperation of SO-ARM100 via M5StickC Plus 1.1."""

from importlib.metadata import version, PackageNotFoundError

try:
    __version__ = version("m5teleop")
except PackageNotFoundError:
    __version__ = "0.1.0"
