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

        # State for continuous velocity mode
        self._continuous_running = False
        self._velocity_a_hz = 0.0
        self._velocity_b_hz = 0.0
        self._continuous_thread_a = None
        self._continuous_thread_b = None

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
        
        If continuous mode is running, it must be paused/stopped before calling this.
        """
        # Temporarily halt continuous velocity if it's running
        was_continuous = self._continuous_running
        if was_continuous:
            self.stop_continuous()

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

        # Resume continuous if it was running
        if was_continuous:
            self.start_continuous()


    # --- Continuous Velocity Control (Background) ---

    def start_continuous(self):
        """Starts background threads that pulse the stepper pins dynamically based on current velocity."""
        if self._continuous_running:
            return

        self._continuous_running = True
        self._velocity_a_hz = 0.0
        self._velocity_b_hz = 0.0

        self._continuous_thread_a = threading.Thread(target=self._continuous_worker_a, daemon=True)
        self._continuous_thread_b = threading.Thread(target=self._continuous_worker_b, daemon=True)

        self._continuous_thread_a.start()
        self._continuous_thread_b.start()
        print("[MOTOR-CONT] Started continuous velocity threads.")

    def stop_continuous(self):
        """Stops the continuous velocity threads."""
        self._continuous_running = False
        if self._continuous_thread_a:
            self._continuous_thread_a.join()
        if self._continuous_thread_b:
            self._continuous_thread_b.join()
        print("[MOTOR-CONT] Stopped continuous velocity threads.")

    def set_velocity(self, pan_hz: float, tilt_hz: float):
        """
        Update the target velocities for both motors.
        Pan = (A+B)/2  => A = Pan+Tilt, B = Pan-Tilt
        """
        # Calculate A and B velocities
        a_hz = pan_hz + tilt_hz
        b_hz = pan_hz - tilt_hz

        self._velocity_a_hz = a_hz
        self._velocity_b_hz = b_hz

    def _continuous_worker_a(self):
        """Background thread to pulse Motor A."""
        # Pull local refs for speed
        p = self.axis_a.step_pin
        while self._continuous_running:
            v = self._velocity_a_hz
            if abs(v) < 1.0:
                # If target is ~0 hz, sleep briefly so we don't busy wait a 0 velocity
                time.sleep(0.01)
                continue
                
            # Set direction
            self.axis_a.set_direction(v > 0)
            
            # Calculate sleep time between pulses (1 / hz)
            delay = 1.0 / abs(v)
            
            GPIO.output(p, GPIO.HIGH)
            # Short pulse width
            time.sleep(0.000001)
            GPIO.output(p, GPIO.LOW)
            
            # Wait remainder of the period. We use time.sleep instead of busy wait
            # to save CPU during free-tracking, though it's less precise than spline mode.
            time.sleep(max(0, delay - 0.000001))

    def _continuous_worker_b(self):
        """Background thread to pulse Motor B."""
        p = self.axis_b.step_pin
        while self._continuous_running:
            v = self._velocity_b_hz
            if abs(v) < 1.0:
                time.sleep(0.01)
                continue
                
            self.axis_b.set_direction(v > 0)
            delay = 1.0 / abs(v)
            
            GPIO.output(p, GPIO.HIGH)
            time.sleep(0.000001)
            GPIO.output(p, GPIO.LOW)
            
            time.sleep(max(0, delay - 0.000001))


    def close(self) -> None:
        if self._continuous_running:
            self.stop_continuous()
        self.axis_a.close()
        self.axis_b.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
