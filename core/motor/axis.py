"""
core/motor/axis.py
------------------
Single-axis abstraction built on top of TMC2209.
Manages strict step-timing execution for flawless trajectory playback.

Reference: legacy/tests/sanjay.py
"""

import time
import math
import RPi.GPIO as GPIO
from typing import List
from tmc_driver import Tmc2209, Loglevel, TmcEnableControlPin, TmcMotionControlStepDir


class Axis:
    """
    Represents one physical motor axis using direct GPIO toggles for steps.
    The tmc_driver library is ONLY used to enable/disable the chip.
    """

    def __init__(
        self,
        driver: Tmc2209,
        step_pin: int,
        dir_pin: int,
        invert: bool = False,
    ):
        self.driver = driver
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.invert = invert

        # Setup GPIO for direct high-speed control
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.setup(self.dir_pin, GPIO.OUT)

        # Enable the motor chip
        self.driver.set_motor_enabled(True)

    # ── GPIO Execution ────────────────────────────────────────────────────────

    def set_direction(self, forward: bool) -> None:
        """Physically set the DIR pin."""
        if self.invert:
            forward = not forward
        GPIO.output(self.dir_pin, GPIO.HIGH if forward else GPIO.LOW)

    def execute_step_times(self, step_times: List[float]) -> float:
        """
        Executes a sequence of exact step times using a microsecond precision
        busy-wait loop (to avoid Python time.sleep() jitter).
        
        Args:
            step_times: List of absolute times (seconds) from trajectory start.
            
        Returns:
            Actual execution duration (seconds)
        """
        if not step_times:
            return 0.0

        p = self.step_pin
        start_time_real = time.time()
        
        for target_t in step_times:
            # BUSY WAIT: Do absolutely nothing until the exact microsecond arrives.
            while time.time() - start_time_real < target_t:
                pass 
                
            # Fire the step pulse
            GPIO.output(p, GPIO.HIGH)
            time.sleep(0.000001)  # Hold high for 1 microsecond
            GPIO.output(p, GPIO.LOW)

        return time.time() - start_time_real

    def close(self) -> None:
        self.driver.set_motor_enabled(False)
