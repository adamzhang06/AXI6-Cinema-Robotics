"""
core/motion/spline.py
---------------------
Cubic Bézier / spline interpolation utilities.

Reference: legacy/tests/spline_test.py, legacy/socket/mac_spline_server.py
"""

from __future__ import annotations
import numpy as np
from typing import List, Tuple


Point = Tuple[float, float]   # (time, value)


class SplineInterpolator:
    """
    Evaluates a sequence of cubic Bézier segments defined by
    (anchor, control_out, control_in, anchor) point groups — matching
    the format produced by the spline editor UI.

    Args:
        keyframes: List of (t, value) anchor points
        handles:   Optional list of (cp_out, cp_in) handle pairs per segment
                   If None, auto-generates smooth handles (Catmull-Rom style)
    """

    def __init__(
        self,
        keyframes: List[Point],
        handles: List[Tuple[Point, Point]] | None = None,
    ):
        self.keyframes = keyframes
        self.handles   = handles or self._auto_handles(keyframes)

    # ── Evaluation ────────────────────────────────────────────────────────────

    def evaluate(self, t: float) -> float:
        """Return interpolated value at time t."""
        if t <= self.keyframes[0][0]:
            return self.keyframes[0][1]
        if t >= self.keyframes[-1][0]:
            return self.keyframes[-1][1]

        seg = self._find_segment(t)
        return self._cubic_bezier(t, seg)

    def evaluate_range(self, times: np.ndarray) -> np.ndarray:
        return np.array([self.evaluate(t) for t in times])

    # ── Segment helpers ───────────────────────────────────────────────────────

    def _find_segment(self, t: float) -> int:
        for i in range(len(self.keyframes) - 1):
            if self.keyframes[i][0] <= t <= self.keyframes[i + 1][0]:
                return i
        return len(self.keyframes) - 2

    def _cubic_bezier(self, t: float, seg: int) -> float:
        t0, v0 = self.keyframes[seg]
        t1, v1 = self.keyframes[seg + 1]
        cp_out, cp_in = self.handles[seg]

        # Normalise t into [0, 1] for this segment
        u = (t - t0) / (t1 - t0)
        u = max(0.0, min(1.0, u))

        # De Casteljau on value only (time axis is linearised)
        b0 = v0
        b1 = cp_out[1]
        b2 = cp_in[1]
        b3 = v1

        val = (
            (1 - u)**3 * b0
            + 3 * (1 - u)**2 * u * b1
            + 3 * (1 - u) * u**2 * b2
            + u**3 * b3
        )
        return val

    # ── Auto-handle generation (Catmull-Rom) ──────────────────────────────────

    @staticmethod
    def _auto_handles(
        keyframes: List[Point],
    ) -> List[Tuple[Point, Point]]:
        handles = []
        n = len(keyframes)
        for i in range(n - 1):
            t0, v0 = keyframes[i]
            t1, v1 = keyframes[i + 1]
            # Tension 1/3 gives Catmull-Rom-like tangents
            if i > 0:
                _, vp = keyframes[i - 1]
                slope_out = (v1 - vp) / 3.0
            else:
                slope_out = (v1 - v0) / 3.0

            if i < n - 2:
                _, vn = keyframes[i + 2]
                slope_in = (vn - v0) / 3.0
            else:
                slope_in = (v1 - v0) / 3.0

            dt = (t1 - t0)
            cp_out = ((t0 + dt / 3), v0 + slope_out)
            cp_in  = ((t1 - dt / 3), v1 - slope_in)
            handles.append((cp_out, cp_in))
        return handles
