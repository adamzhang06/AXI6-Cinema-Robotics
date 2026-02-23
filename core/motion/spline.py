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



def generate_step_times(spline_data) -> tuple[bool, list[float]]:
    """
    Given an array of absolute positional targets [t, expected_pos_steps],
    convert it into an array of exact times that every single motor step should occur at.
    """
    if len(spline_data) < 2:
        return True, []
        
    times = []
    # Total distance dictates direction 
    total_dp = spline_data[-1][1] - spline_data[0][1]
    direction_is_forward = (total_dp >= 0)
    
    current_step_target = spline_data[0][1]
    step_increment = 1.0 if direction_is_forward else -1.0
    
    # Simple linear interpolation across the spline segments to find exact time for each step boundary.
    for i in range(1, len(spline_data)):
        t_prev, p_prev = spline_data[i-1]
        t_next, p_next = spline_data[i]
        
        # If the segment goes forward and our global direction is forward
        # OR the segment goes backward and our global direction is backward
        if (p_next > p_prev and direction_is_forward) or (p_next < p_prev and not direction_is_forward):
            
            while True:
                next_step = current_step_target + step_increment
                
                # Check if this next discrete integer step falls within the current segment
                if (direction_is_forward and next_step <= p_next) or (not direction_is_forward and next_step >= p_next):
                    # Linear interpolate to find the exact time this step boundary is crossed
                    ratio = (next_step - p_prev) / (p_next - p_prev)
                    exact_t = t_prev + ratio * (t_next - t_prev)
                    times.append(exact_t)
                    current_step_target = next_step
                else:
                    break

    return direction_is_forward, times
