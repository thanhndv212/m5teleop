#!/usr/bin/env python
"""
tune_ekf.py — ESKF attitude filter parameter tuning tool.

Modes
-----
  live         Continuous terminal display of orientation, bias, and ZARU status.
  stationary   Record N seconds of still data, score current EKF parameters.
  sweep        Grid-search parameter combinations on recorded data; print ranking.
  optimize     Nelder-Mead fine-tune from the sweep winner (requires scipy).

Typical workflow
----------------
  1. Place M5StickC on a flat, still surface.
  2. python tune_ekf.py stationary --duration 90 --save rec.npz
     → records real data, scores current config.py params, saves for reuse.
  3. python tune_ekf.py sweep --load rec.npz
     → grid-searches ~240 combos offline in ~1 min; prints top-5 + config snippet.
  4. python tune_ekf.py optimize --load rec.npz
     → Nelder-Mead fine-tunes from sweep winner; prints final config snippet.
  5. Copy the suggested values into m5teleop/m5teleop/config.py.

Offline (no hardware)
---------------------
  python tune_ekf.py live --dry-run
  python tune_ekf.py sweep --dry-run --duration 90

Scoring (lower = better)
-------------------------
  score = 3·yaw_peak_to_peak + pitch_rms + roll_rms + 0.5·|bias_z| + 0.3·converge_frac
"""

from __future__ import annotations

import argparse
import itertools
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

_HERE = Path(__file__).parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent / "m5imu"))

from m5teleop.imu_ekf import ImuEKF
from m5teleop import config


# ─────────────────────────────────────────────────────────────────────────────
# Parameter container
# ─────────────────────────────────────────────────────────────────────────────

_PARAM_NAMES = (
    "sigma_gyro", "sigma_bias", "sigma_acc",
    "acc_gate", "zaru_threshold", "sigma_zaru",
)


@dataclass
class EkfParams:
    sigma_gyro:     float = config.EKF_SIGMA_GYRO
    sigma_bias:     float = config.EKF_SIGMA_BIAS
    sigma_acc:      float = config.EKF_SIGMA_ACC
    acc_gate:       float = config.EKF_ACC_GATE
    zaru_threshold: float = config.EKF_ZARU_THRESHOLD
    sigma_zaru:     float = config.EKF_SIGMA_ZARU

    def to_array(self) -> np.ndarray:
        return np.array([getattr(self, k) for k in _PARAM_NAMES], dtype=float)

    @classmethod
    def from_array(cls, x: np.ndarray) -> EkfParams:
        x = np.clip(x, 1e-7, 10.0)
        return cls(**{k: float(v) for k, v in zip(_PARAM_NAMES, x)})

    def make_ekf(self) -> ImuEKF:
        return ImuEKF(
            sigma_gyro=self.sigma_gyro,
            sigma_bias=self.sigma_bias,
            sigma_acc=self.sigma_acc,
            acc_gate=self.acc_gate,
            zaru_threshold=self.zaru_threshold,
            sigma_zaru=self.sigma_zaru,
        )

    def __str__(self) -> str:
        return (
            f"σ_gyro={self.sigma_gyro:.5f}  σ_bias={self.sigma_bias:.6f}  "
            f"σ_acc={self.sigma_acc:.4f}  gate={self.acc_gate:.3f}  "
            f"zaru_thr={self.zaru_threshold:.4f}  σ_zaru={self.sigma_zaru:.4f}"
        )


# ─────────────────────────────────────────────────────────────────────────────
# Drift metrics
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class DriftMetrics:
    yaw_drift_pp:   float   # peak-to-peak yaw deviation over steady-state window (°)
    yaw_drift_rate: float   # °/min
    pitch_rms:      float   # RMS error vs accel-derived reference (°)
    roll_rms:       float   # RMS error vs accel-derived reference (°)
    bias_z_final:   float   # Z-axis bias at end of session (dps)
    converge_frac:  float   # fraction of session elapsed when Z-bias settled [0–1]
    n_samples:      int

    @property
    def score(self) -> float:
        """Scalar cost (lower is better)."""
        return (
            3.0 * self.yaw_drift_pp
            + 1.0 * self.pitch_rms
            + 1.0 * self.roll_rms
            + 0.5 * abs(self.bias_z_final)
            + 0.3 * self.converge_frac
        )

    def one_line(self) -> str:
        return (
            f"yaw_pp={self.yaw_drift_pp:5.2f}° ({self.yaw_drift_rate:4.1f}°/min)  "
            f"p_rms={self.pitch_rms:4.2f}°  r_rms={self.roll_rms:4.2f}°  "
            f"bz={self.bias_z_final:+5.2f} dps  "
            f"conv={self.converge_frac * 100:.0f}%  "
            f"score={self.score:.4f}"
        )


# ─────────────────────────────────────────────────────────────────────────────
# EKF replay engine
# ─────────────────────────────────────────────────────────────────────────────

_WARMUP_FRAC   = 0.20   # discard first 20 % of samples as EKF warm-up
_CONVERGE_WIN  = 50     # consecutive samples below threshold → converged
_CONVERGE_DPS  = 0.005  # bias-change threshold (dps / sample)


def replay(raw: np.ndarray, params: EkfParams, dt: float) -> DriftMetrics:
    """Run EKF on a pre-recorded IMU buffer and return drift metrics.

    Parameters
    ----------
    raw : (N, 6) float64
        Columns: [ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps]
    dt  : float
        Sample interval [s].
    """
    ekf = params.make_ekf()
    N = len(raw)

    rolls    = np.empty(N)
    pitches  = np.empty(N)
    yaws     = np.empty(N)
    biases_z = np.empty(N)
    ref_pitch = np.empty(N)
    ref_roll  = np.empty(N)

    for i in range(N):
        ax, ay, az, gx, gy, gz = raw[i]
        ekf.step(ax, ay, az, gx, gy, gz, dt)
        r, p, y        = ekf.euler_deg
        rolls[i]       = r
        pitches[i]     = p
        yaws[i]        = y
        biases_z[i]    = ekf.bias_dps[2]

        a_n = math.sqrt(ax * ax + ay * ay + az * az)
        if a_n > 0.01:
            ref_roll[i]  = math.degrees(math.atan2(ay, az))
            ref_pitch[i] = math.degrees(math.asin(max(-1.0, min(1.0, -ax / a_n))))
        else:
            ref_roll[i] = ref_pitch[i] = 0.0

    # ── steady-state window ───────────────────────────────────────────────────
    w      = max(1, int(N * _WARMUP_FRAC))
    dur_ss = (N - w) * dt

    # Yaw: centre on first steady-state sample, unwrap, measure peak-to-peak
    yaw_ss   = np.unwrap(np.deg2rad(yaws[w:] - yaws[w])) * (180.0 / math.pi)
    yaw_pp   = float(np.ptp(yaw_ss))
    yaw_rate = yaw_pp / dur_ss * 60.0 if dur_ss > 0 else 0.0

    pitch_rms = float(np.sqrt(np.mean((pitches[w:] - ref_pitch[w:]) ** 2)))
    roll_rms  = float(np.sqrt(np.mean((rolls[w:]  - ref_roll[w:])  ** 2)))

    # ── Z-axis bias convergence ───────────────────────────────────────────────
    bz = biases_z[w:]
    db = np.abs(np.diff(bz))
    converge_frac = 1.0
    for j in range(len(db) - _CONVERGE_WIN):
        if np.all(db[j : j + _CONVERGE_WIN] < _CONVERGE_DPS):
            converge_frac = j / max(1, len(db))
            break

    return DriftMetrics(
        yaw_drift_pp=yaw_pp,
        yaw_drift_rate=yaw_rate,
        pitch_rms=pitch_rms,
        roll_rms=roll_rms,
        bias_z_final=float(biases_z[-1]),
        converge_frac=converge_frac,
        n_samples=N,
    )


# ─────────────────────────────────────────────────────────────────────────────
# Data collection
# ─────────────────────────────────────────────────────────────────────────────

_DT_FIRMWARE = 1.0 / 100.0   # M5StickC target: 100 Hz


def collect_live(duration: float, port: Optional[str]) -> tuple[np.ndarray, float]:
    """Record real IMU data for *duration* seconds. Returns (raw (N,6), dt)."""
    from m5imu import ImuReader, find_port as _find_port  # noqa: PLC0415

    p = port or config.IMU_PORT or _find_port()
    if p is None:
        raise RuntimeError(
            "No M5StickC port detected. Provide --port or use --dry-run."
        )

    print(f"[tune_ekf] Connecting to {p} …")
    reader = ImuReader(port=p, debug=False)
    reader.open()
    time.sleep(0.5)

    samples: list[list[float]] = []
    t0 = time.perf_counter()
    print(
        f"[tune_ekf] Recording {duration:.0f} s "
        f"— keep device STILL on a flat surface …"
    )
    try:
        for imu in reader:
            samples.append([imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz])
            elapsed = time.perf_counter() - t0
            bar = int(elapsed / duration * 50)
            print(
                f"\r  [{'=' * bar}{' ' * (50 - bar)}] "
                f"{elapsed:5.1f}/{duration:.0f} s",
                end="", flush=True,
            )
            if elapsed >= duration:
                break
    finally:
        reader.close()

    print()
    raw = np.array(samples, dtype=np.float64)
    dt  = duration / len(raw)
    print(f"[tune_ekf] {len(raw)} samples  dt≈{dt * 1000:.2f} ms  ({1/dt:.0f} Hz)")
    return raw, dt


def collect_synthetic(duration: float, dt: float = _DT_FIRMWARE) -> np.ndarray:
    """Generate synthetic stationary IMU data with a realistic Z-axis gyro bias.

    Simulates MPU6886 lying flat (az ≈ 1 g), Gaussian noise on all channels,
    and a 1.5 dps Z-axis bias with a slow ±0.3 dps thermal drift.
    """
    rng = np.random.default_rng(42)
    N   = int(duration / dt)
    t   = np.arange(N) * dt

    ax = rng.normal(0.0, 0.015, N)
    ay = rng.normal(0.0, 0.015, N)
    az = rng.normal(1.0, 0.015, N)
    gx = rng.normal(0.0, 0.50,  N)
    gy = rng.normal(0.0, 0.50,  N)
    gz = (
        rng.normal(0.0, 0.50, N)
        + 1.5                                            # constant bias
        + 0.3 * np.sin(2 * math.pi * t / duration)      # slow thermal drift
    )

    return np.column_stack([ax, ay, az, gx, gy, gz])


def _load_or_collect(args) -> tuple[np.ndarray, float]:
    if args.load:
        data = np.load(args.load)
        raw, dt = data["raw"], float(data["dt"])
        print(
            f"[tune_ekf] Loaded {len(raw)} samples from '{args.load}'  "
            f"dt={dt * 1000:.2f} ms"
        )
        return raw, dt
    if args.dry_run:
        dt  = _DT_FIRMWARE
        raw = collect_synthetic(args.duration, dt)
        print(
            f"[tune_ekf] Synthetic data: {len(raw)} samples  "
            f"dt={dt * 1000:.2f} ms  (bias_z=1.5 dps)"
        )
        return raw, dt
    return collect_live(args.duration, getattr(args, "port", None))


def _maybe_save(args, raw: np.ndarray, dt: float) -> None:
    if getattr(args, "save", None):
        np.savez(args.save, raw=raw, dt=dt)
        print(f"[tune_ekf] Recording saved to '{args.save}'")


# ─────────────────────────────────────────────────────────────────────────────
# Default sweep grid
# ─────────────────────────────────────────────────────────────────────────────

# sigma_acc and acc_gate are fixed during the sweep; they have little effect on
# stationary yaw drift and inflating the grid makes the sweep impractically long.
_SWEEP_GRID: dict[str, list[float]] = {
    "sigma_gyro":     [0.002, 0.005, 0.010, 0.020],
    "sigma_bias":     [0.00005, 0.0001, 0.0003, 0.0008],
    "sigma_acc":      [config.EKF_SIGMA_ACC],        # fixed
    "acc_gate":       [config.EKF_ACC_GATE],          # fixed
    "zaru_threshold": [0.04, 0.06, 0.08, 0.12, 0.18],
    "sigma_zaru":     [0.01, 0.02, 0.04],
}


# ─────────────────────────────────────────────────────────────────────────────
# Display helpers
# ─────────────────────────────────────────────────────────────────────────────

def _hr() -> None:
    print("─" * 80)


def _print_config_snippet(params: EkfParams) -> None:
    _hr()
    print("  ▶  Paste into  m5teleop/m5teleop/config.py :")
    _hr()
    print(f"  EKF_SIGMA_GYRO:     float = {params.sigma_gyro!r}")
    print(f"  EKF_SIGMA_BIAS:     float = {params.sigma_bias!r}")
    print(f"  EKF_SIGMA_ACC:      float = {params.sigma_acc!r}")
    print(f"  EKF_ACC_GATE:       float = {params.acc_gate!r}")
    print(f"  EKF_ZARU_THRESHOLD: float = {params.zaru_threshold!r}")
    print(f"  EKF_SIGMA_ZARU:     float = {params.sigma_zaru!r}")
    _hr()


# ─────────────────────────────────────────────────────────────────────────────
# Mode: live
# ─────────────────────────────────────────────────────────────────────────────

def mode_live(port: Optional[str], dry_run: bool, params: EkfParams) -> None:
    """Continuous terminal display: orientation, bias, ZARU status, drift."""
    reader = None

    if dry_run:
        _dt = _DT_FIRMWARE
        rng  = np.random.default_rng()

        def _stream():
            while True:
                yield (
                    rng.normal(0, 0.015),
                    rng.normal(0, 0.015),
                    rng.normal(1, 0.015),
                    rng.normal(0, 0.5),
                    rng.normal(0, 0.5),
                    rng.normal(0, 0.5) + 1.5,
                    _dt,
                )
    else:
        from m5imu import ImuReader, find_port as _find_port  # noqa

        p = port or config.IMU_PORT or _find_port()
        if p is None:
            raise RuntimeError("No port found. Provide --port or use --dry-run.")
        reader = ImuReader(port=p, debug=False)
        reader.open()
        time.sleep(0.5)

        def _stream():
            prev = time.perf_counter()
            for imu in reader:
                now  = time.perf_counter()
                yield imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz, now - prev
                prev = now

    ekf = params.make_ekf()
    yaw_ref     = None
    max_yaw_dev = 0.0
    t0          = time.perf_counter()
    n           = 0
    NLINES      = 9

    print(f"\nParams : {params}")
    print("Ctrl-C to stop.\n")
    print("\n" * NLINES, end="")

    try:
        for ax, ay, az, gx, gy, gz, dt in _stream():
            ekf.step(ax, ay, az, gx, gy, gz, dt)
            roll, pitch, yaw = ekf.euler_deg
            bias             = ekf.bias_dps
            n += 1
            elapsed = time.perf_counter() - t0

            # Wait 1 s before locking reference yaw (let EKF warm up)
            if yaw_ref is None and n > 100:
                yaw_ref = yaw
            if yaw_ref is not None:
                dev = abs(yaw - yaw_ref)
                dev = min(dev, 360.0 - dev)
                max_yaw_dev = max(max_yaw_dev, dev)

            # ZARU indicator
            omega_rads = math.sqrt(gx * gx + gy * gy + gz * gz) * math.pi / 180.0
            zaru_on    = omega_rads < params.zaru_threshold

            # Accel-derived reference (valid when stationary)
            a_n = math.sqrt(ax * ax + ay * ay + az * az)
            if a_n > 0.01:
                ref_r = math.degrees(math.atan2(ay, az))
                ref_p = math.degrees(math.asin(max(-1.0, min(1.0, -ax / a_n))))
            else:
                ref_r = ref_p = 0.0

            print(f"\033[{NLINES}A", end="")
            hz_str = f"{n/elapsed:.0f}" if elapsed > 0 else "--"
            print(f"  Elapsed:    {elapsed:7.1f} s     n={n:7d}     Hz≈{hz_str}")
            print(f"  EKF:        roll={roll:+7.2f}°  pitch={pitch:+7.2f}°  yaw={yaw:+7.2f}°")
            print(f"  Accel ref:  roll={ref_r:+7.2f}°  pitch={ref_p:+7.2f}°")
            print(f"  Err:        Δroll={roll-ref_r:+6.2f}°   Δpitch={pitch-ref_p:+6.2f}°")
            print(
                f"  Bias (dps): bx={bias[0]:+6.3f}  by={bias[1]:+6.3f}  bz={bias[2]:+6.3f}"
                f"  {'← ZARU converging' if zaru_on and abs(bias[2]) > 0.05 else ''}"
            )
            yaw_rate_str = (
                f"  ({max_yaw_dev / elapsed * 60:.2f}°/min)" if elapsed > 0 else ""
            )
            print(
                f"  Yaw dev:    {max_yaw_dev:6.3f}°{yaw_rate_str}"
            )
            zaru_str = (
                "ACTIVE  ← bias correction running"
                if zaru_on
                else "inactive  (motion detected)"
            )
            print(f"  ZARU:       {zaru_str}")
            print(f"  |ω|:        {omega_rads * 180/math.pi:6.3f} dps   "
                  f"threshold={params.zaru_threshold * 180/math.pi:.3f} dps")
            print()

            if dry_run:
                time.sleep(_dt)
    except KeyboardInterrupt:
        pass
    finally:
        if reader is not None:
            try:
                reader.close()
            except Exception:
                pass


# ─────────────────────────────────────────────────────────────────────────────
# Mode: stationary
# ─────────────────────────────────────────────────────────────────────────────

def mode_stationary(
    raw: np.ndarray,
    dt: float,
    params: EkfParams,
    label: str = "current config",
) -> DriftMetrics:
    metrics = replay(raw, params, dt)
    print(f"\n  [{label}]")
    print(f"  Params  : {params}")
    print(f"  Metrics : {metrics.one_line()}")
    return metrics


# ─────────────────────────────────────────────────────────────────────────────
# Mode: sweep
# ─────────────────────────────────────────────────────────────────────────────

def mode_sweep(
    raw: np.ndarray,
    dt: float,
    grid: dict[str, list[float]] | None = None,
) -> tuple[EkfParams, DriftMetrics]:
    """Grid-search all combinations. Returns (best_params, best_metrics)."""
    if grid is None:
        grid = _SWEEP_GRID

    keys   = list(grid.keys())
    combos = list(itertools.product(*grid.values()))
    total  = len(combos)

    # Estimate time
    t_probe = time.perf_counter()
    replay(raw[:min(200, len(raw))], EkfParams(), dt)
    t_per_sample = (time.perf_counter() - t_probe) / min(200, len(raw))
    est_s = t_per_sample * len(raw) * total
    print(
        f"\n[tune_ekf] Sweeping {total} combinations "
        f"(estimated {est_s:.0f} s / {est_s/60:.1f} min) …"
    )

    results: list[tuple[EkfParams, DriftMetrics]] = []
    for i, combo in enumerate(combos):
        p = EkfParams(**dict(zip(keys, combo)))
        m = replay(raw, p, dt)
        results.append((p, m))
        if (i + 1) % 20 == 0 or (i + 1) == total:
            best_s = min(results, key=lambda x: x[1].score)[1].score
            print(
                f"\r  {i+1}/{total}  best score so far: {best_s:.4f}",
                end="", flush=True,
            )

    print()
    results.sort(key=lambda x: x[1].score)

    _hr()
    print(f"  Top 5 results (out of {total}):")
    _hr()
    for rank, (p, m) in enumerate(results[:5], 1):
        print(f"  #{rank}  score={m.score:.4f}  {m.one_line()}")
        print(f"       {p}")
        print()

    return results[0]


# ─────────────────────────────────────────────────────────────────────────────
# Mode: optimize
# ─────────────────────────────────────────────────────────────────────────────

def mode_optimize(
    raw: np.ndarray,
    dt: float,
    x0: EkfParams,
) -> EkfParams:
    """Nelder-Mead minimization starting from *x0*. Requires scipy."""
    try:
        from scipy.optimize import minimize  # noqa: PLC0415
    except ImportError:
        print("[tune_ekf] scipy not available (pip install scipy). Skipping optimize.")
        return x0

    call_count = [0]

    def objective(x: np.ndarray) -> float:
        p = EkfParams.from_array(x)
        m = replay(raw, p, dt)
        call_count[0] += 1
        if call_count[0] % 10 == 0:
            print(f"\r  eval {call_count[0]}  score={m.score:.4f}", end="", flush=True)
        return m.score

    print(f"\n[tune_ekf] Nelder-Mead starting from:\n  {x0}")
    result = minimize(
        objective,
        x0.to_array(),
        method="Nelder-Mead",
        options={"maxiter": 600, "xatol": 1e-5, "fatol": 1e-4},
    )
    print(f"\r  {call_count[0]} evaluations  converged={result.success}  "
          f"final score={result.fun:.4f}")
    return EkfParams.from_array(result.x)


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────

def _add_recording_args(p) -> None:
    p.add_argument("--duration", type=float, default=60.0, metavar="SEC",
                   help="Recording duration in seconds (default 60)")
    p.add_argument("--port", default=None, help="M5StickC serial port (auto-detected)")
    p.add_argument("--dry-run", action="store_true",
                   help="Use synthetic IMU data (no hardware)")
    p.add_argument("--save", default=None, metavar="FILE.npz",
                   help="Save raw recording for offline reuse")
    p.add_argument("--load", default=None, metavar="FILE.npz",
                   help="Load a previously saved recording (skips data collection)")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="ESKF attitude filter parameter tuning tool for M5StickC Plus 1.1",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    sub = parser.add_subparsers(dest="mode", required=True)

    p_live = sub.add_parser("live", help="Continuous orientation + bias display")
    p_live.add_argument("--port",    default=None)
    p_live.add_argument("--dry-run", action="store_true")

    p_stat = sub.add_parser("stationary", help="Score current EKF params on stationary data")
    _add_recording_args(p_stat)

    p_sweep = sub.add_parser("sweep", help="Grid-search EKF parameter space")
    _add_recording_args(p_sweep)

    p_opt = sub.add_parser("optimize", help="Nelder-Mead fine-tune (requires scipy)")
    _add_recording_args(p_opt)

    args = parser.parse_args()

    print("=" * 80)
    print("  M5StickC ESKF Parameter Tuner")
    print("=" * 80)

    # ── live ──────────────────────────────────────────────────────────────────
    if args.mode == "live":
        mode_live(port=args.port, dry_run=args.dry_run, params=EkfParams())
        return

    # ── modes requiring recorded data ─────────────────────────────────────────
    raw, dt = _load_or_collect(args)
    _maybe_save(args, raw, dt)

    defaults = EkfParams()

    if args.mode == "stationary":
        mode_stationary(raw, dt, defaults)

    elif args.mode == "sweep":
        print("\n  Baseline (config.py):")
        baseline = mode_stationary(raw, dt, defaults, label="baseline")
        _hr()
        best_p, best_m = mode_sweep(raw, dt)
        delta = (baseline.score - best_m.score) / max(baseline.score, 1e-9) * 100
        print(f"\n  Improvement over baseline: {baseline.score:.4f} → {best_m.score:.4f}"
              f"  ({delta:+.1f}%)")
        _print_config_snippet(best_p)

    elif args.mode == "optimize":
        print("\n  Baseline (config.py):")
        baseline = mode_stationary(raw, dt, defaults, label="baseline")
        _hr()

        # Coarse sweep first to find a good starting point for the optimizer
        print("\n  Running coarse sweep to seed optimizer …")
        best_p, _ = mode_sweep(raw, dt)

        best_p = mode_optimize(raw, dt, best_p)
        best_m = replay(raw, best_p, dt)

        delta = (baseline.score - best_m.score) / max(baseline.score, 1e-9) * 100
        _hr()
        print(f"\n  Final result:")
        print(f"  {best_p}")
        print(f"  {best_m.one_line()}")
        print(f"\n  Improvement over baseline: {baseline.score:.4f} → {best_m.score:.4f}"
              f"  ({delta:+.1f}%)")
        _print_config_snippet(best_p)


if __name__ == "__main__":
    main()
