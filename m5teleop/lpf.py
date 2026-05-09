"""Exponential moving average (EMA) low-pass filter.

Single source of truth for raw-data smoothing across the m5teleop pipeline.
All modules that need to filter IMU channels (imu_twist, tune_ekf, viz, …)
should import :class:`Lpf` from here.
"""

from __future__ import annotations

import numpy as np

from . import config


class Lpf:
    """First-order exponential moving average filter for N-channel signals.

    Parameters
    ----------
    channels : int
        Number of channels (length of the array passed to :meth:`update`).
    alpha : float
        Smoothing factor in ``(0, 1]``.
        ``alpha = 1`` → pass-through (no filtering).
        ``alpha → 0`` → heavier smoothing / slower response.
        Defaults to :data:`config.LPF_ALPHA`.

    Usage
    -----
    ::

        lpf = Lpf(channels=6)           # ax, ay, az, gx, gy, gz
        for sample in stream:
            filtered = lpf.update(sample)
    """

    def __init__(
        self,
        channels: int = 6,
        alpha: float = config.LPF_ALPHA,
    ) -> None:
        if not (0.0 < alpha <= 1.0):
            raise ValueError(f"alpha must be in (0, 1], got {alpha!r}")
        self._alpha = alpha
        self._channels = channels
        self._state: np.ndarray | None = None

    @property
    def alpha(self) -> float:
        return self._alpha

    def reset(self) -> None:
        """Clear filter state (e.g. when teleop is re-enabled after a pause)."""
        self._state = None

    def update(self, x: np.ndarray) -> np.ndarray:
        """Feed one sample; return the filtered output (same shape).

        Seeds from the first sample so there is no startup transient.

        Parameters
        ----------
        x : array-like, shape (channels,)
            Raw input sample.

        Returns
        -------
        np.ndarray, shape (channels,)
            Filtered sample (a copy — safe to mutate).
        """
        x = np.asarray(x, dtype=float)
        if self._state is None:
            self._state = x.copy()   # seed: no lag on first sample
        else:
            self._state = self._alpha * x + (1.0 - self._alpha) * self._state
        return self._state.copy()
