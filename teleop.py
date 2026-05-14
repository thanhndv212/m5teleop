#!/usr/bin/env python
"""
m5teleop – Teleoperate SO-ARM100 via M5StickC Plus 1.1 IMU.

Usage (sim only, no hardware):
    conda activate gosim
    cd soarm-ws/m5teleop
    python teleop.py --dry-run

Usage (full hardware + viser + rerun):
    python teleop.py --servo-port /dev/cu.usbserial-XXXX

Controls:
    BTN_A (side button) : toggle teleoperation on/off
    BTN_B (top button)  : toggle gripper open/closed
    Ctrl-C              : quit
"""

import argparse
import queue
import signal
import sys
import threading
import time
from pathlib import Path

import serial

import numpy as np

# ---------------------------------------------------------------------------
# Make local packages importable when running from m5teleop/
# ---------------------------------------------------------------------------
_HERE = Path(__file__).parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent / "m5imu"))

from imu_sdk import ImuData, ImuReader, find_port
from m5teleop import config
from m5teleop.lerobot_soarm_interface import ArmInterface
from m5teleop.ik_solver import IKSolver
from m5teleop.imu_ekf import ImuEKF
from m5teleop.imu_twist import ImuTwistConverter
from m5teleop.orient_controller import OrientationController
from m5teleop.sim_interface import SimInterface
from m5teleop.viz import TeleopVisualizer
import pinocchio as pin

# ---------------------------------------------------------------------------
# Graceful shutdown
# ---------------------------------------------------------------------------

_shutdown = threading.Event()


def _sigint_handler(sig, frame):
    print("\n[teleop] Shutdown requested …")
    _shutdown.set()


signal.signal(signal.SIGINT, _sigint_handler)


# ---------------------------------------------------------------------------
# IMU background reader → thread-safe queue
# ---------------------------------------------------------------------------


def _imu_thread(
    reader: ImuReader,
    q: "queue.Queue[ImuData]",
    connected_flag: list,  # connected_flag[0] = bool, mutable
) -> None:
    prev_btn_a = False
    prev_btn_b = False
    try:
        for sample in reader:
            if _shutdown.is_set():
                break
            btn_changed = (sample.btn_a != prev_btn_a) or (
                sample.btn_b != prev_btn_b
            )
            prev_btn_a = sample.btn_a
            prev_btn_b = sample.btn_b
            if btn_changed:
                # Always deliver samples that carry a button edge
                if not q.full():
                    q.put(sample, block=False)
            else:
                # For IMU-only samples, keep only the latest to avoid lag
                while not q.empty():
                    try:
                        q.get_nowait()
                    except queue.Empty:
                        break
                q.put(sample)
    except serial.SerialException as exc:
        print(f"[teleop] IMU serial error: {exc}")
    except Exception as exc:
        print(f"[teleop] IMU reader error: {exc}")
    finally:
        connected_flag[0] = False
        print("[teleop] IMU reader thread stopped")


# ---------------------------------------------------------------------------
# Button edge-detector
# ---------------------------------------------------------------------------


class _EdgeDetect:
    def __init__(self):
        self._prev = False

    def rising(self, current: bool) -> bool:
        rose = current and not self._prev
        self._prev = current
        return rose


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Teleoperate SO-ARM100 via M5StickC Plus 1.1 IMU",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--imu-port", default=None, help="M5StickC serial port (auto-detected)"
    )
    parser.add_argument(
        "--servo-port", default=None, help="SO100 servo bus port"
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Disable serial (offline IK test)",
    )
    parser.add_argument(
        "--no-sim", action="store_true", help="Disable viser simulation window"
    )
    parser.add_argument(
        "--no-rerun", action="store_true", help="Disable Rerun visualisation"
    )
    parser.add_argument(
        "--no-rerun-spawn",
        action="store_true",
        help="Disable auto-spawning of the Rerun viewer process",
    )
    parser.add_argument(
        "--hz",
        type=int,
        default=config.CONTROL_HZ,
        help=f"Control loop rate (default {config.CONTROL_HZ} Hz)",
    )
    args = parser.parse_args()

    dt = 1.0 / args.hz

    # ------------------------------------------------------------------
    # 1. IK solver (always on)
    # ------------------------------------------------------------------
    print("[teleop] Loading URDF and building IK solver …")
    ik = IKSolver()
    print(f"[teleop] Model: {ik.model.nq} dof, EE frame: '{config.EE_FRAME}'")

    # ------------------------------------------------------------------
    # 2. Hardware arm
    # ------------------------------------------------------------------
    arm = ArmInterface(port=args.servo_port, dry_run=args.dry_run)
    arm.connect()
    print(
        f"[teleop] Arm interface: {'DRY-RUN' if args.dry_run else args.servo_port}"
    )

    # Seed IK at current arm pose
    q0 = arm.get_joint_radians()
    q0_full = np.zeros(ik.model.nq)
    q0_full[: len(q0)] = q0
    ik.reset(q0_full)
    q_current = q0_full.copy()

    # ------------------------------------------------------------------
    # 2b. EKF attitude filter + cascade orientation controller
    # ------------------------------------------------------------------
    ekf = ImuEKF()
    orient_ctrl = OrientationController()

    def _get_ee_quaternion() -> np.ndarray:
        """Return current EE orientation as [w, x, y, z] (world frame)."""
        se3   = ik.get_ee_pose()
        q_pin = pin.Quaternion(se3.rotation)   # pinocchio: coeffs = [x,y,z,w]
        return np.array([q_pin.w, q_pin.x, q_pin.y, q_pin.z])

    def _do_zero_reset() -> None:
        """Capture current IMU + EE orientations as the controller reference."""
        q_imu = ekf.quaternion
        q_ee  = _get_ee_quaternion()
        orient_ctrl.zero_reset(q_imu, q_ee)
        print("[teleop] Zero reset done.")

    # ------------------------------------------------------------------
    # 3. Viser simulation + connection command queue
    # ------------------------------------------------------------------
    conn_queue: queue.Queue[tuple] = queue.Queue()

    sim: SimInterface | None = None
    if not args.no_sim:
        sim = SimInterface(ik_solver=ik)

        # Register GUI callbacks — they push commands onto conn_queue so
        # the main loop handles serial operations on its own thread.
        sim.on_imu_connect    = lambda port: conn_queue.put(("connect_imu", port))
        sim.on_imu_disconnect  = lambda: conn_queue.put(("disconnect_imu",))
        sim.on_servo_connect   = lambda port: conn_queue.put(("connect_servo", port))
        sim.on_servo_disconnect = lambda: conn_queue.put(("disconnect_servo",))
        sim.on_zero_reset      = _do_zero_reset

        sim.start()

    # ------------------------------------------------------------------
    # 4. Rerun visualisation
    # ------------------------------------------------------------------
    viz: TeleopVisualizer | None = None
    if not args.no_rerun:
        viz = TeleopVisualizer()
        try:
            viz.init(spawn=not args.no_rerun_spawn)
            print("[teleop] Rerun connected")
        except Exception as exc:
            print(f"[teleop] Rerun unavailable ({exc}), continuing without it")
            viz = None

    # ------------------------------------------------------------------
    # 5. IMU reader
    # ------------------------------------------------------------------
    imu_queue: queue.Queue[ImuData] = queue.Queue(maxsize=5)
    imu_port = args.imu_port or (None if args.dry_run else (config.IMU_PORT or find_port()))

    imu_reader: ImuReader | None = None
    imu_connected = False

    # Shared mutable flag so the IMU thread can signal disconnect
    _imu_connected_flag: list = [False]

    def _start_imu(port: str) -> bool:
        """Open IMU reader on *port*. Returns True on success."""
        nonlocal imu_reader, imu_connected, imu_port
        if imu_reader is not None:
            try:
                imu_reader.close()
            except Exception:
                pass
        # Brief open/close to flush any stale data from another process
        try:
            _flush = serial.Serial(
                port, 115200, timeout=0.1, dsrdtr=False, rtscts=False
            )
            _flush.reset_input_buffer()
            _flush.close()
        except Exception:
            pass
        try:
            r = ImuReader(port=port, debug=False)
            r.open()
            imu_reader = r
            imu_port = port
            imu_connected = True
            _imu_connected_flag[0] = True
            t = threading.Thread(
                target=_imu_thread,
                args=(r, imu_queue, _imu_connected_flag),
                daemon=True,
                name="ImuReader",
            )
            t.start()
            print(f"[teleop] IMU connected on {port}")
            return True
        except Exception as exc:
            print(f"[teleop] IMU connect failed on {port}: {exc}")
            imu_reader = None
            imu_connected = False
            return False

    def _stop_imu() -> None:
        nonlocal imu_reader, imu_connected
        if imu_reader is not None:
            try:
                imu_reader.close()
            except Exception:
                pass
            imu_reader = None
        imu_connected = False
        print("[teleop] IMU disconnected")

    if not args.dry_run:
        if imu_port is None:
            print(
                "[teleop] WARNING: No IMU port found. Running in dry-run mode."
            )
            args.dry_run = True
        else:
            _start_imu(imu_port)

    # ------------------------------------------------------------------
    # 6. State + servo reconnect helpers
    # ------------------------------------------------------------------
    servo_connected = not args.dry_run and args.servo_port is not None

    def _start_servo(port: str) -> bool:
        nonlocal servo_connected
        arm.disconnect()
        arm._port = port
        try:
            arm.connect()
            servo_connected = True
            print(f"[teleop] Servo connected on {port}")
            # Re-seed IK from new arm position
            q0 = arm.get_joint_radians()
            q0_full = np.zeros(ik.model.nq)
            q0_full[: len(q0)] = q0
            ik.reset(q0_full)
            return True
        except Exception as exc:
            print(f"[teleop] Servo connect failed on {port}: {exc}")
            servo_connected = False
            return False

    def _stop_servo() -> None:
        nonlocal servo_connected
        arm.disconnect()
        servo_connected = False
        print("[teleop] Servo disconnected")

    converter = ImuTwistConverter()
    teleop_active = False
    btn_a_edge = _EdgeDetect()
    btn_b_edge = _EdgeDetect()
    zero_twist = np.zeros(6)

    print("[teleop] Ready — BTN_A to start. Ctrl-C to quit.")
    if args.dry_run:
        print(
            "[teleop] DRY-RUN: IK running on zero twist. Open browser for viser."
        )

    # ------------------------------------------------------------------
    # 7. Control loop
    # ------------------------------------------------------------------
    loop_count = 0
    try:
        while not _shutdown.is_set():
            t_start = time.perf_counter()

            # -- Process connection commands from viser GUI (non-blocking)
            while True:
                try:
                    cmd = conn_queue.get_nowait()
                except queue.Empty:
                    break
                action = cmd[0]
                if action == "connect_imu":
                    _start_imu(cmd[1])
                elif action == "disconnect_imu":
                    _stop_imu()
                elif action == "connect_servo":
                    _start_servo(cmd[1])
                elif action == "disconnect_servo":
                    _stop_servo()

            # -- Sync imu_connected from thread flag (handles serial disconnect)
            if imu_connected and not _imu_connected_flag[0]:
                imu_connected = False

            # -- Push connection status to viser GUI
            if sim is not None:
                sim.update_connection_status(
                    imu_connected=imu_connected,
                    imu_port=imu_port or "",
                    servo_connected=servo_connected,
                    servo_port=arm._port or "",
                )

            # -- Get latest IMU sample (non-blocking)
            # Prefer real samples from the queue regardless of dry-run mode;
            # synthesise a zero sample only when nothing is available.
            imu: ImuData | None = None
            try:
                imu = imu_queue.get_nowait()
            except queue.Empty:
                pass

            # Fall back to synthetic zero sample (dry-run or IMU not yet connected)
            if imu is None and args.dry_run:
                imu = ImuData(ax=0, ay=0, az=1, gx=0, gy=0, gz=0, temp=25.0)

            # -- Button edge detection
            if imu is not None:
                if btn_a_edge.rising(imu.btn_a):
                    teleop_active = not teleop_active
                    if teleop_active:
                        # Auto zero-reset on every teleop activation
                        _do_zero_reset()
                    status = "ON" if teleop_active else "OFF"
                    print(f"[teleop] Teleop {status}")
                if btn_b_edge.rising(imu.btn_b):
                    arm.toggle_gripper()
                    print(
                        f"[teleop] Gripper {'open' if arm.gripper_is_open else 'closed'}"
                    )

            # -- EKF: always run when IMU data is available
            if imu is not None:
                ekf.step(
                    imu.ax, imu.ay, imu.az,
                    imu.gx, imu.gy, imu.gz,
                    dt,
                )

            # -- Compute twist via cascade orientation controller
            orient_err = np.zeros(3)
            if teleop_active and imu is not None:
                q_imu = ekf.quaternion
                q_ee  = _get_ee_quaternion()
                twist, orient_err = orient_ctrl.compute_twist(q_imu, q_ee, dt)
            else:
                twist = zero_twist

            # -- IK step
            q_current = ik.step(twist, dt)

            # -- Send to arm (5 revolute joints only, gripper handled separately)
            deg_dict = ik.q_to_degrees(q_current)
            arm.send_joint_degrees(deg_dict)

            # -- Simulation update
            if sim is not None:
                sim.update(q_current, teleop_active, arm.gripper_is_open)
                if imu is not None:
                    ekf_roll, ekf_pitch, ekf_yaw = ekf.euler_deg
                    sim.update_imu_frame(ekf_pitch, ekf_roll, ekf_yaw)

            # -- Rerun logging
            if viz is not None and imu is not None:
                ee_pose = ik.get_ee_pose()
                # EE target rotation: from controller (identity until first reset)
                ee_target_R = orient_ctrl.last_target_rotation
                viz.log_all_with_tracking(
                    imu=imu,
                    twist=twist,
                    q=q_current,
                    ee_translation=ee_pose.translation,
                    ee_rotation=ee_pose.rotation,
                    teleop_active=teleop_active,
                    gripper_open=arm.gripper_is_open,
                    ekf_euler=ekf.euler_deg,
                    ekf_bias=ekf.bias_dps,
                    orient_err=orient_err,
                    omega_actual=orient_ctrl._omega_actual,
                    ee_target_rotation=ee_target_R,
                )

            # -- Rate limiting
            loop_count += 1
            elapsed = time.perf_counter() - t_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            elif loop_count % (args.hz * 5) == 0:
                print(
                    f"[teleop] WARNING: loop overrun by {-sleep_time*1000:.1f} ms"
                )

    finally:
        print("[teleop] Shutting down …")
        _shutdown.set()
        _stop_imu()
        if sim is not None:
            sim.stop()
        arm.disconnect()
        print("[teleop] Done.")


if __name__ == "__main__":
    main()
