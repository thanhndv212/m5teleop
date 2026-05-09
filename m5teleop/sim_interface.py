"""Viser-based 3-D simulation / visualisation of SO-ARM100.

Runs in a background thread and mirrors the current IK solution at each
control step without blocking the main loop.

Open http://localhost:8080 (or the configured port) in a browser to view
the robot in real time.

GUI panels
----------
* **Connections** – select serial port, connect / disconnect each device,
  live status indicators for the M5StickC IMU and the servo driver.
* **Teleoperation** – active/inactive state and gripper status.
"""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import TYPE_CHECKING, Callable

import numpy as np
import pinocchio as pin
import serial.tools.list_ports
import viser
from viser.extras import ViserUrdf

from . import config

if TYPE_CHECKING:
    from .ik_solver import IKSolver


def _scan_ports() -> list[str]:
    """Return sorted list of available serial port device names."""
    devices = sorted(p.device for p in serial.tools.list_ports.comports())
    return devices if devices else ["(none)"]


def _euler_to_wxyz(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """ZYX Euler angles (degrees) → quaternion [w, x, y, z]."""
    r, p, y = np.deg2rad([roll_deg, pitch_deg, yaw_deg])
    cr, sr = np.cos(r / 2), np.sin(r / 2)
    cp, sp = np.cos(p / 2), np.sin(p / 2)
    cy, sy = np.cos(y / 2), np.sin(y / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,   # w
        sr * cp * cy - cr * sp * sy,   # x
        cr * sp * cy + sr * cp * sy,   # y
        cr * cp * sy - sr * sp * cy,   # z
    ])


class SimInterface:
    """Browser-based 3-D visualisation using *viser* + pinocchio.

    The GUI includes a **Connections** panel so the user can select ports,
    connect / disconnect the IMU and servo driver live from the browser.

    Register callbacks *before* calling :meth:`start`::

        sim.on_imu_connect    = lambda port: ...
        sim.on_imu_disconnect = lambda: ...
        sim.on_servo_connect  = lambda port: ...
        sim.on_servo_disconnect = lambda: ...

    Call :meth:`update_connection_status` from the main loop to keep the
    status indicators in sync.
    """

    def __init__(
        self,
        ik_solver: "IKSolver",
        host: str = config.VISER_HOST,
        port: int = config.VISER_PORT,
    ) -> None:
        self._solver = ik_solver
        self._host = host
        self._port = port
        self._server: viser.ViserServer | None = None
        self._viser_urdf: ViserUrdf | None = None
        self._ee_frame_handle = None
        self._imu_frame_handle = None
        self._lock = threading.Lock()
        self._latest_q: np.ndarray | None = None
        self._teleop_active: bool = False
        self._gripper_open: bool = True
        self._imu_pitch: float = 0.0
        self._imu_roll: float = 0.0
        self._imu_yaw: float = 0.0
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()

        # Connection state (updated from main loop via update_connection_status)
        self._imu_connected: bool = False
        self._imu_port_str: str = ""
        self._servo_connected: bool = False
        self._servo_port_str: str = ""

        # GUI handles (populated in _build_gui)
        self._imu_status_md = None
        self._imu_port_dd = None
        self._imu_connect_btn = None
        self._imu_disconnect_btn = None
        self._servo_status_md = None
        self._servo_port_dd = None
        self._servo_connect_btn = None
        self._servo_disconnect_btn = None
        self._teleop_status_text = None
        self._gripper_text = None

        # Callbacks – set by teleop.py before start()
        self.on_imu_connect: Callable[[str], None] | None = None
        self.on_imu_disconnect: Callable[[], None] | None = None
        self.on_servo_connect: Callable[[str], None] | None = None
        self.on_servo_disconnect: Callable[[], None] | None = None
        self.on_zero_reset: Callable[[], None] | None = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Start the viser server and background update thread."""
        self._server = viser.ViserServer(host=self._host, port=self._port)
        self._server.scene.world_axes.visible = True

        # Load URDF via viser.extras.ViserUrdf
        self._viser_urdf = ViserUrdf(
            self._server,
            Path(config.URDF_PATH),
            root_node_name="/robot",
        )

        # EE target marker
        self._ee_frame_handle = self._server.scene.add_frame(
            name="ee_target",
            axes_length=0.04,
            axes_radius=0.003,
        )

        # IMU body-frame marker (floats above and in front of the robot)
        self._imu_frame_handle = self._server.scene.add_frame(
            name="imu_frame",
            axes_length=0.06,
            axes_radius=0.004,
            origin_radius=0.008,
        )
        self._imu_frame_handle.position = (0.0, 0.28, 0.28)

        self._build_gui()

        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, daemon=True, name="ViserUpdate"
        )
        self._thread.start()
        print(
            f"[SimInterface] Viser server at http://{self._host}:{self._port}"
        )

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._server:
            self._server.stop()

    # ------------------------------------------------------------------
    # GUI construction
    # ------------------------------------------------------------------

    def _build_gui(self) -> None:
        assert self._server is not None
        gui = self._server.gui

        # ── Connections panel ─────────────────────────────────────────
        with gui.add_folder("Connections"):

            # IMU sub-folder
            with gui.add_folder("IMU  (M5StickC)"):
                self._imu_status_md = gui.add_markdown("🔴 **Disconnected**")
                ports = _scan_ports()
                self._imu_port_dd = gui.add_dropdown(
                    "Port", options=ports, initial_value=ports[0]
                )
                imu_refresh = gui.add_button("↺ Refresh", color="gray")
                with gui.add_folder("_imu_actions"):
                    self._imu_connect_btn = gui.add_button(
                        "Connect", color="green"
                    )
                    self._imu_disconnect_btn = gui.add_button(
                        "Disconnect", color="red", disabled=True
                    )

            # Servo sub-folder
            with gui.add_folder("Servo driver"):
                self._servo_status_md = gui.add_markdown("🔴 **Disconnected**")
                self._servo_port_dd = gui.add_dropdown(
                    "Port", options=ports, initial_value=ports[0]
                )
                servo_refresh = gui.add_button("↺ Refresh", color="gray")
                with gui.add_folder("_servo_actions"):
                    self._servo_connect_btn = gui.add_button(
                        "Connect", color="green"
                    )
                    self._servo_disconnect_btn = gui.add_button(
                        "Disconnect", color="red", disabled=True
                    )

        # ── Teleoperation status ──────────────────────────────────────
        with gui.add_folder("Teleoperation"):
            self._teleop_status_text = gui.add_text(
                "Status", initial_value="Teleop OFF"
            )
            self._gripper_text = gui.add_text("Gripper", initial_value="open")
            self._zero_reset_btn = gui.add_button("⊙ Zero IMU", color="blue")

        # ── Wire up button callbacks ──────────────────────────────────
        @imu_refresh.on_click
        def _(_):
            new_ports = _scan_ports()
            self._imu_port_dd.options = new_ports
            self._servo_port_dd.options = new_ports

        @servo_refresh.on_click
        def _(_):
            new_ports = _scan_ports()
            self._imu_port_dd.options = new_ports
            self._servo_port_dd.options = new_ports

        @self._imu_connect_btn.on_click
        def _(_):
            port = self._imu_port_dd.value
            if port and port != "(none)" and self.on_imu_connect:
                self.on_imu_connect(port)

        @self._imu_disconnect_btn.on_click
        def _(_):
            if self.on_imu_disconnect:
                self.on_imu_disconnect()

        @self._servo_connect_btn.on_click
        def _(_):
            port = self._servo_port_dd.value
            if port and port != "(none)" and self.on_servo_connect:
                self.on_servo_connect(port)

        @self._servo_disconnect_btn.on_click
        def _(_):
            if self.on_servo_disconnect:
                self.on_servo_disconnect()

        @self._zero_reset_btn.on_click
        def _(_):
            if self.on_zero_reset:
                self.on_zero_reset()

    # ------------------------------------------------------------------
    # Update API (called from main loop)
    # ------------------------------------------------------------------

    def update(
        self,
        q: np.ndarray,
        teleop_active: bool = False,
        gripper_open: bool = True,
    ) -> None:
        """Push new joint configuration to the visualiser."""
        with self._lock:
            self._latest_q = q.copy()
            self._teleop_active = teleop_active
            self._gripper_open = gripper_open

    def update_imu_frame(self, pitch: float, roll: float, yaw: float) -> None:
        """Push the latest IMU orientation (degrees) to the visualiser."""
        with self._lock:
            self._imu_pitch = pitch
            self._imu_roll = roll
            self._imu_yaw = yaw

    def update_connection_status(
        self,
        *,
        imu_connected: bool | None = None,
        imu_port: str | None = None,
        servo_connected: bool | None = None,
        servo_port: str | None = None,
    ) -> None:
        """Reflect hardware connection state in the GUI status indicators."""
        with self._lock:
            if imu_connected is not None:
                self._imu_connected = imu_connected
            if imu_port is not None:
                self._imu_port_str = imu_port
            if servo_connected is not None:
                self._servo_connected = servo_connected
            if servo_port is not None:
                self._servo_port_str = servo_port

    # ------------------------------------------------------------------
    # Background thread
    # ------------------------------------------------------------------

    def _run(self) -> None:
        model = self._solver.model
        data = model.createData()

        while not self._stop.is_set():
            with self._lock:
                q = (
                    self._latest_q.copy()
                    if self._latest_q is not None
                    else None
                )
                active = self._teleop_active
                g_open = self._gripper_open
                imu_conn = self._imu_connected
                imu_port = self._imu_port_str
                servo_conn = self._servo_connected
                servo_port = self._servo_port_str
                imu_pitch = self._imu_pitch
                imu_roll = self._imu_roll
                imu_yaw = self._imu_yaw

            if q is not None and self._viser_urdf is not None:
                n_joints = len(self._viser_urdf.get_actuated_joint_names())
                cfg = q[:n_joints].copy()
                # Inject gripper joint (last DOF = URDF joint 6, gripper→jaw)
                gripper_rad = np.radians(
                    config.GRIPPER_OPEN_DEG if g_open else config.GRIPPER_CLOSED_DEG
                )
                cfg[n_joints - 1] = gripper_rad
                self._viser_urdf.update_cfg(cfg)

                # EE frame marker
                pin.forwardKinematics(model, data, q)
                pin.updateFramePlacements(model, data)
                ee_id = model.getFrameId(config.EE_FRAME)
                if ee_id < len(data.oMf):
                    se3 = data.oMf[ee_id]
                    t = se3.translation
                    R = se3.rotation
                    wxyz = pin.Quaternion(R).coeffs()[[3, 0, 1, 2]]
                    if self._ee_frame_handle:
                        self._ee_frame_handle.position = tuple(t)
                        self._ee_frame_handle.wxyz = tuple(wxyz)

            # IMU orientation frame
            if self._imu_frame_handle is not None:
                imu_wxyz = _euler_to_wxyz(imu_roll, imu_pitch, imu_yaw)
                self._imu_frame_handle.wxyz = tuple(imu_wxyz)

            # Connection status indicators
            if self._imu_status_md:
                self._imu_status_md.content = (
                    f"🟢 **Connected**  `{imu_port}`"
                    if imu_conn
                    else "🔴 **Disconnected**"
                )
            if self._imu_connect_btn:
                self._imu_connect_btn.disabled = imu_conn
            if self._imu_disconnect_btn:
                self._imu_disconnect_btn.disabled = not imu_conn

            if self._servo_status_md:
                self._servo_status_md.content = (
                    f"🟢 **Connected**  `{servo_port}`"
                    if servo_conn
                    else "🔴 **Disconnected**"
                )
            if self._servo_connect_btn:
                self._servo_connect_btn.disabled = servo_conn
            if self._servo_disconnect_btn:
                self._servo_disconnect_btn.disabled = not servo_conn

            # Teleop status
            if self._teleop_status_text:
                self._teleop_status_text.value = (
                    "Teleop ON" if active else "Teleop OFF"
                )
            if self._gripper_text:
                self._gripper_text.value = "open" if g_open else "closed"

            time.sleep(1.0 / config.CONTROL_HZ)
