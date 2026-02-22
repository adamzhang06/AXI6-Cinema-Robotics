"""
core/motion/trajectory.py
--------------------------
High-level trajectory planning for one or two axes.

Produces a time-series of (t, pan_velocity, tilt_velocity) commands
from a SplineInterpolator, ready to stream to a DifferentialDrive.

Reference: legacy/socket/pi_spline_motor.py, legacy/tests/spline_test.py
"""

from __future__ import annotations
import time
import threading
from typing import Callable
from .spline import SplineInterpolator


class TrajectoryPlanner:
    """
    Plays back a pair of splines (pan + tilt) in real time,
    calling a command callback at a fixed update rate.

    Args:
        pan_spline:    SplineInterpolator for pan axis
        tilt_spline:   SplineInterpolator for tilt axis (can be same as pan)
        command_fn:    Callable(pan_velocity, tilt_velocity) — e.g. drive.move
        update_rate:   Hz (default 100)
    """

    def __init__(
        self,
        pan_spline:  SplineInterpolator,
        tilt_spline: SplineInterpolator,
        command_fn:  Callable[[float, float], None],
        update_rate: int = 100,
    ):
        self.pan_spline  = pan_spline
        self.tilt_spline = tilt_spline
        self.command_fn  = command_fn
        self.update_rate = update_rate

        self._running  = False
        self._thread: threading.Thread | None = None

    # ── Playback control ──────────────────────────────────────────────────────

    def start(self, start_time: float = 0.0) -> None:
        """Begin trajectory playback from start_time (seconds)."""
        if self._running:
            return
        self._running   = True
        self._thread = threading.Thread(
            target=self._playback_loop,
            args=(start_time,),
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        """Interrupt playback."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        self.command_fn(0.0, 0.0)

    @property
    def is_playing(self) -> bool:
        return self._running

    # ── Internal ─────────────────────────────────────────────────────────────

    def _playback_loop(self, start_time: float) -> None:
        dt       = 1.0 / self.update_rate
        origin   = time.monotonic() - start_time
        end_time = self.pan_spline.keyframes[-1][0]

        while self._running:
            t = time.monotonic() - origin
            if t > end_time:
                self._running = False
                break

            pan_v  = self.pan_spline.evaluate(t)
            tilt_v = self.tilt_spline.evaluate(t)
            self.command_fn(pan_v, tilt_v)
            time.sleep(dt)

        self.command_fn(0.0, 0.0)
