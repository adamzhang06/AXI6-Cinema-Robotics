#!/usr/bin/env python3
"""
Trapezoidal Motion with pigpio DMA Stepping
=============================================
Uses pigpio's hardware DMA to generate jitter-free step pulses.
tmc_driver handles TMC2209 setup (enable, config).
pigpio handles the actual step pulse timing via DMA waves.

Run on Pi:  python tests/trapezoidal_pigpio.py

Input: time(s), angle(°), n (2+ for easing)
  n=2   → full triangle (max easing)
  n=100 → nearly instant ramp (sharp)

Requires: pip install pigpio
Also run: sudo pigpiod  (start the pigpio daemon)
"""

import time
import math
import pigpio
from tmc_driver import (
    Tmc2209, Loglevel, MovementAbsRel,
    TmcEnableControlPin, TmcMotionControlStepDir,
)

# ==================== PIN CONFIGURATION ====================
PIN_STEP = 16
PIN_DIR  = 20
PIN_EN   = 21

STEPS_PER_REV = 1600   # Microsteps per 360°
STEP_PULSE_US = 5       # Step pulse width in microseconds (TMC2209 min ~1us)


class SmoothMotor:
    """TMC2209 motor with pigpio DMA-timed trapezoidal motion."""

    def __init__(self):
        # --- pigpio connection ---
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("Cannot connect to pigpiod! Run: sudo pigpiod")

        self.pi.set_mode(PIN_STEP, pigpio.OUTPUT)
        self.pi.set_mode(PIN_DIR, pigpio.OUTPUT)
        self.pi.set_mode(PIN_EN, pigpio.OUTPUT)

        # --- TMC2209 setup (just for enable/config) ---
        self.tmc = Tmc2209(
            TmcEnableControlPin(PIN_EN),
            TmcMotionControlStepDir(PIN_STEP, PIN_DIR),
            loglevel=Loglevel.INFO,
        )
        self.tmc.set_motor_enabled(True)
        print("[MOTOR] Enabled via tmc_driver")

    def _build_trapezoidal_wave(self, total_steps, v_max, a_max):
        """
        Build a pigpio waveform for a trapezoidal velocity profile.
        Uses the AVR446 discrete approximation for step timing.
        
        Returns the wave_id ready to send.
        """
        if total_steps == 0:
            return None

        abs_steps = abs(total_steps)

        # Set direction
        direction = 1 if total_steps > 0 else 0
        self.pi.write(PIN_DIR, direction)
        time.sleep(0.001)  # Direction setup time

        # Calculate accel/decel step counts
        # Steps to accelerate to v_max: n_accel = v_max² / (2 * a_max)
        n_accel = int(v_max * v_max / (2.0 * a_max))
        n_accel = min(n_accel, abs_steps // 2)  # Can't accel more than half
        n_decel = n_accel
        n_cruise = abs_steps - n_accel - n_decel

        # Build pulse list
        pulses = []

        # --- Phase 1: Acceleration ---
        for i in range(n_accel):
            if i == 0:
                # First step: initial speed from rest
                # v = sqrt(2 * a * 1) → delay = 1/v
                speed = math.sqrt(2.0 * a_max * (i + 1))
            else:
                speed = math.sqrt(2.0 * a_max * (i + 1))
            
            speed = min(speed, v_max)
            delay_us = max(int(1_000_000.0 / speed), STEP_PULSE_US * 2)

            # Step pulse: HIGH then LOW
            pulses.append(pigpio.pulse(1 << PIN_STEP, 0, STEP_PULSE_US))
            pulses.append(pigpio.pulse(0, 1 << PIN_STEP, delay_us - STEP_PULSE_US))

        # --- Phase 2: Cruise ---
        if n_cruise > 0:
            cruise_delay_us = max(int(1_000_000.0 / v_max), STEP_PULSE_US * 2)
            for _ in range(n_cruise):
                pulses.append(pigpio.pulse(1 << PIN_STEP, 0, STEP_PULSE_US))
                pulses.append(pigpio.pulse(0, 1 << PIN_STEP, cruise_delay_us - STEP_PULSE_US))

        # --- Phase 3: Deceleration ---
        for i in range(n_decel):
            remaining = n_decel - i
            speed = math.sqrt(2.0 * a_max * remaining)
            speed = min(speed, v_max)
            speed = max(speed, 1.0)  # Don't divide by zero
            delay_us = max(int(1_000_000.0 / speed), STEP_PULSE_US * 2)

            pulses.append(pigpio.pulse(1 << PIN_STEP, 0, STEP_PULSE_US))
            pulses.append(pigpio.pulse(0, 1 << PIN_STEP, delay_us - STEP_PULSE_US))

        # Build wave
        self.pi.wave_clear()
        self.pi.wave_add_generic(pulses)
        wave_id = self.pi.wave_create()
        return wave_id

    def move_trapezoidal(self, theta, duration, n):
        """
        Execute a trapezoidal move that completes in exactly `duration` seconds.
        
        theta:    degrees (negative = reverse)
        duration: seconds
        n:        2+ (2 = full triangle, higher = sharper ramp)
        """
        steps = round(theta * STEPS_PER_REV / 360)
        abs_steps = abs(steps)

        if abs_steps == 0:
            print("  No steps to move!")
            return

        # --- Calculate v_max and a_max from user's formula ---
        v_avg = abs_steps / duration
        v_max = v_avg * (n / (n - 1))
        a_max = (v_max / duration) * n

        # Timing breakdown
        t_accel = v_max / a_max  # = duration / n
        t_cruise = duration - 2 * t_accel

        # Step count breakdown
        n_accel_steps = int(v_max * v_max / (2.0 * a_max))
        n_accel_steps = min(n_accel_steps, abs_steps // 2)
        n_cruise_steps = abs_steps - 2 * n_accel_steps

        print(f"  θ={theta}° → {steps} steps | v_max={v_max:.1f} sps | a={a_max:.1f} sps²")
        print(f"  profile: {t_accel:.2f}s ramp ({n_accel_steps} steps) + "
              f"{t_cruise:.2f}s cruise ({n_cruise_steps} steps) + "
              f"{t_accel:.2f}s ramp ({n_accel_steps} steps)")

        # Build and send the DMA wave
        wave_id = self._build_trapezoidal_wave(steps, v_max, a_max)
        if wave_id is None:
            return

        t_start = time.time()
        self.pi.wave_send_once(wave_id)

        # Wait for wave to complete
        while self.pi.wave_tx_busy():
            time.sleep(0.01)

        actual = time.time() - t_start
        self.pi.wave_delete(wave_id)

        print(f"  actual: {actual:.2f}s (expected {duration:.2f}s, diff={actual - duration:+.3f}s)")

    def shutdown(self):
        self.pi.wave_clear()
        self.tmc.set_motor_enabled(False)
        self.pi.stop()
        print("[SHUTDOWN] Motor disabled, pigpio stopped.")


def main():
    print("=" * 50)
    print("  AXI6 Trapezoidal Motion (pigpio DMA)")
    print("  Input: time(s), angle(°), n(2+)")
    print("  n=2 → smooth triangle, n=100 → sharp")
    print("=" * 50 + "\n")

    motor = SmoothMotor()

    try:
        while True:
            user_input = input("\nTime(s), Angle(°), n  [or 'q']: ").strip()

            if user_input.lower() == 'q':
                break

            try:
                parts = user_input.replace(',', ' ').split()
                if len(parts) != 3:
                    print("  Enter 3 values, e.g: 5 360 2")
                    continue

                duration = float(parts[0])
                theta = float(parts[1])
                n = float(parts[2])

                if duration <= 0:
                    print("  Time must be > 0")
                    continue
                if n < 2:
                    print("  n must be >= 2")
                    continue

                motor.move_trapezoidal(theta, duration, n)

            except ValueError:
                print("  Invalid input. Example: 5 360 2")

    except KeyboardInterrupt:
        print("\nInterrupted.")

    finally:
        motor.shutdown()

if __name__ == "__main__":
    main()
