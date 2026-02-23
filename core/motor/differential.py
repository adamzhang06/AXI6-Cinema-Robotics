"""
core/motor/differential.py
--------------------------
Differential pan/tilt drive executor.

Converts pan/tilt spline trajectories into absolute step arrays for both 
co-axial motors, then executes them simultaneously in parallel threads
using busy-wait precision.

Also supports continuous velocity control for live tracking.
"""

import time
import threading
from typing import List
import RPi.GPIO as GPIO
from .axis import Axis


class DifferentialDrive:
    """
    Two-DOF differential drive executor.

    Motor A = Pan + Tilt
    Motor B = Pan - Tilt
    """

    def __init__(
        self,
        axis_a: Axis,
        axis_b: Axis,
    ):
        self.axis_a = axis_a
        self.axis_b = axis_b


    # --- Precise Spline Execution (Blocking) ---

    def execute_dual_timing(
        self, 
        motor_a_forward: bool,
        motor_a_times: List[float],
        motor_b_forward: bool,
        motor_b_times: List[float]
    ) -> None:
        """
        Fires exact calculated step schedules for both motors in parallel.
        Uses busy-wait loops to ensure jitter-free timing.
        

        """
        self.axis_a.set_direction(motor_a_forward)
        self.axis_b.set_direction(motor_b_forward)
        
        print(f"[MOTOR] Executing A: {len(motor_a_times)} steps | B: {len(motor_b_times)} steps")

        # Start execution baseline time, freeze time clock for threads
        start_time = time.time() + 0.1  # Give 100ms for thread spin-up sync

        # Threading wrapper for pure GPIO execution
        def run_gpio(axis: Axis, times: List[float]):
            p = axis.step_pin
            for t in times:
                while time.time() - start_time < t:
                    pass
                GPIO.output(p, GPIO.HIGH)
                time.sleep(0.0000001) # 0.1 microsecond
                GPIO.output(p, GPIO.LOW)

        t_a = threading.Thread(target=run_gpio, args=(self.axis_a, motor_a_times))
        t_b = threading.Thread(target=run_gpio, args=(self.axis_b, motor_b_times))

        t_a.start()
        t_b.start()

        t_a.join()
        t_b.join()

        actual = time.time() - start_time
        print(f"[MOTOR] Move complete. Execution time: {actual:.3f}s")


    def close(self) -> None:
        self.axis_a.close()
        self.axis_b.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
