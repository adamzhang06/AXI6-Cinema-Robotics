# pylint: disable=wildcard-import
# pylint: disable=unused-wildcard-import
"""
Script to execute a flawless, jitter-free trapezoidal profile using exact step timing.
"""

import time
import math
import RPi.GPIO as GPIO
from tmc_driver.tmc_2209 import *

print("---")
print("SCRIPT START")
print("---")

# -----------------------------------------------------------------------
# 1. Math Function: Invert Kinematics to find Exact Step Times
# -----------------------------------------------------------------------
def calculate_step_times(total_steps, total_time, phases=3.0):
    """
    Returns a list of exact absolute times (in seconds) at which each step should occur.
    """
    if total_steps == 0:
        return []

    t_a = total_time / phases
    t_c = total_time - t_a
    
    v_max = total_steps / (total_time - t_a)
    a = v_max / t_a
    
    # Boundaries (in steps) for the different phases
    P_a = 0.5 * a * (t_a**2)
    P_c = P_a + v_max * (t_c - t_a)
    
    step_times = []
    
    for p in range(1, int(total_steps) + 1):
        if p <= P_a:
            # Phase 1: Acceleration
            t = math.sqrt((2 * p) / a)
        elif p <= P_c:
            # Phase 2: Constant Velocity
            t = t_a + ((p - P_a) / v_max)
        else:
            # Phase 3: Deceleration
            dp = p - P_c
            # Prevent negative square root due to minor floating point inaccuracies at the very end
            discriminant = max(0.0, (v_max**2) - (2 * a * dp))
            t_prime = (v_max - math.sqrt(discriminant)) / a
            t = t_c + t_prime
            
        step_times.append(t)
        
    return step_times

# -----------------------------------------------------------------------
# 2. Get User Input
# -----------------------------------------------------------------------
target_angle = float(input("Enter final angle in degrees (e.g., 360): "))
target_time = float(input("Enter total move time in seconds (e.g., 5.0): "))
target_phases = float(input("Enter total phases (e.g., 3.0): "))

steps_per_rev = 400
total_steps_float = abs((target_angle / 360.0) * steps_per_rev)
direction_is_forward = (target_angle >= 0)

step_times = calculate_step_times(total_steps_float, target_time, phases=target_phases)
print(f"\nCalculated {len(step_times)} exact step pulses.")

# -----------------------------------------------------------------------
# 3. Initiate TMC2209 (Only for Driver Enable)
# -----------------------------------------------------------------------
tmc = Tmc2209(
    TmcEnableControlPin(21),
    TmcMotionControlStepDir(16, 20),
    loglevel=Loglevel.INFO,
)

# We use the library just to turn the driver chip on
tmc.set_motor_enabled(True)

# -----------------------------------------------------------------------
# 4. Setup GPIO for Direct High-Speed Control
# -----------------------------------------------------------------------
STEP_PIN = 16
DIR_PIN = 20

# The library likely set this up, but we enforce it for safety
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)

# Set the direction physically
GPIO.output(DIR_PIN, GPIO.HIGH if direction_is_forward else GPIO.LOW)

# -----------------------------------------------------------------------
# 5. Execute using a Microsecond-Precision Busy-Wait Loop
# -----------------------------------------------------------------------
print("Executing precise step-by-step trajectory...")
start_time_real = time.time()

for target_t in step_times:
    # BUSY WAIT: Do absolutely nothing until the exact microsecond arrives.
    # This prevents the 1-2ms jitter caused by Python's time.sleep()
    while time.time() - start_time_real < target_t:
        pass 
        
    # Fire the step pulse
    GPIO.output(STEP_PIN, GPIO.HIGH)
    time.sleep(0.000001) # Hold high for 1 microsecond (TMC requirement)
    GPIO.output(STEP_PIN, GPIO.LOW)

actual_time = time.time() - start_time_real
print(f"Move complete. Expected time: {target_time}s | Actual time: {actual_time:.3f}s")

# -----------------------------------------------------------------------
# Clean up
# -----------------------------------------------------------------------
tmc.set_motor_enabled(False)
del tmc

print("---")
print("SCRIPT FINISHED")
print("---")