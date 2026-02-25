"""
core/motor/axis.py
------------------
Single-axis abstraction for TMC2209 using pure Raspberry Pi GPIO.
Manages strict step-timing execution for flawless trajectory playback.
No tmc_driver library used - direct EN/STEP/DIR pin toggling only.

Reference: legacy/tests/sanjay_threaded.py
"""

import time
import RPi.GPIO as GPIO
from typing import List


class Axis:
    """
    Represents one physical motor axis using direct GPIO toggles for steps.
    """

    def __init__(
        self,
        en_pin: int,
        step_pin: int,
        dir_pin: int,
        invert: bool = False,
    ):
        self.en_pin = en_pin
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.invert = invert

        # Setup GPIO for direct high-speed control
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.en_pin, self.step_pin, self.dir_pin], GPIO.OUT)

        # Enable the motor chip (TMC2209 EN is active LOW)
        GPIO.output(self.en_pin, GPIO.LOW)

    # ── GPIO Execution ────────────────────────────────────────────────────────

    def set_direction(self, forward: bool) -> None:
        """Physically set the DIR pin."""
        if self.invert:
            forward = not forward
        GPIO.output(self.dir_pin, GPIO.HIGH if forward else GPIO.LOW)

    def close(self) -> None:
        """Disable motor (TMC2209 EN is active LOW, so HIGH disables it)."""
        GPIO.output(self.en_pin, GPIO.HIGH)
