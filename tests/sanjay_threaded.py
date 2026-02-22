"""
tests/sanjay_threaded.py
------------------------
Proves that standard Python threads running the exact sanjay.py 
busy-wait GPIO logic can run two motors perfectly in sync without 
relying on the tmc_driver library's built-in loops.
"""

import time
import math
import threading
import RPi.GPIO as GPIO


print("---")
print("SCRIPT START")
print("---")

# -----------------------------------------------------------------------
# 1. Math Function: Exact Step Times (from sanjay.py)
# -----------------------------------------------------------------------
def calculate_step_times(total_steps, total_time, phases=3.0):
    if total_steps == 0:
        return []

    t_a = total_time / phases
    t_c = total_time - t_a
    
    v_max = total_steps / (total_time - t_a)
    a = v_max / t_a
    
    P_a = 0.5 * a * (t_a**2)
    P_c = P_a + v_max * (t_c - t_a)
    
    step_times = []
    for p in range(1, int(total_steps) + 1):
        if p <= P_a:
            t = math.sqrt((2 * p) / a)
        elif p <= P_c:
            t = t_a + ((p - P_a) / v_max)
        else:
            dp = p - P_c
            discriminant = max(0.0, (v_max**2) - (2 * a * dp))
            t_prime = (v_max - math.sqrt(discriminant)) / a
            t = t_c + t_prime
            
        step_times.append(t)
        
    return step_times

# -----------------------------------------------------------------------
# 2. Get User Input & Differential Math
# -----------------------------------------------------------------------
pan_angle = float(input("Enter Pan angle in degrees (e.g., 360): "))
tilt_angle = float(input("Enter Tilt angle in degrees (e.g., 0): "))
target_time = float(input("Enter total move time in seconds (e.g., 5.0): "))

# Differential Math
# Motor A = Tilt + Pan
# Motor B = Tilt - Pan
motor_a_angle = tilt_angle + pan_angle
motor_b_angle = tilt_angle - pan_angle

steps_per_rev = 400
a_steps_float = abs((motor_a_angle / 360.0) * steps_per_rev)
b_steps_float = abs((motor_b_angle / 360.0) * steps_per_rev)

a_direction_forward = (motor_a_angle >= 0)
b_direction_forward = (motor_b_angle >= 0)

a_step_times = calculate_step_times(a_steps_float, target_time, phases=3.0)
b_step_times = calculate_step_times(b_steps_float, target_time, phases=3.0)

print(f"\nCalculated Motor A: {len(a_step_times)} steps | Motor B: {len(b_step_times)} steps.")

# -----------------------------------------------------------------------
# 3. Setup GPIO for Direct High-Speed Control
# -----------------------------------------------------------------------
PAN_STEP = 16
PAN_DIR = 20
PAN_EN = 21

TILT_STEP = 13
TILT_DIR = 19
TILT_EN = 26

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup([PAN_STEP, PAN_DIR, PAN_EN, TILT_STEP, TILT_DIR, TILT_EN], GPIO.OUT)

# Enable the motors (LOW = Enabled for TMC2209)
GPIO.output(PAN_EN, GPIO.LOW)
GPIO.output(TILT_EN, GPIO.LOW)

# Set directions physically
# Note: In our pi/server.py architecture, Motor B is inverted
GPIO.output(PAN_DIR, GPIO.HIGH if a_direction_forward else GPIO.LOW)
GPIO.output(TILT_DIR, GPIO.LOW if b_direction_forward else GPIO.HIGH) # Inverted

# -----------------------------------------------------------------------
# 5. Thread Execution Engine
# -----------------------------------------------------------------------
# We freeze the start time so both threads trigger off the exact same clock
start_time_real = time.time() + 0.5  # Give threads 500ms to spin up

def motor_pulse_loop(step_pin, times_array):
    """The exact sanjay.py logic running inside a dedicated thread"""
    for target_t in times_array:
        # Busy-wait loop for microsecond precision against the global clock
        while time.time() - start_time_real < target_t:
            pass 
            
        # Fire pulse
        GPIO.output(step_pin, GPIO.HIGH)
        time.sleep(0.000001) # 1 microsecond hold
        GPIO.output(step_pin, GPIO.LOW)


print("Launching parallel threads...")

# Create python threads pointing to our GPIO loop
pan_thread = threading.Thread(target=motor_pulse_loop, args=(PAN_STEP, a_step_times))
tilt_thread = threading.Thread(target=motor_pulse_loop, args=(TILT_STEP, b_step_times))

pan_thread.start()
tilt_thread.start()

# Wait for both to finish
pan_thread.join()
tilt_thread.join()

actual_time = time.time() - start_time_real
print(f"Dual-Move complete. Expected time: {target_time}s | Actual time: {actual_time:.3f}s")

# -----------------------------------------------------------------------
# Clean up
# -----------------------------------------------------------------------
# Disable motors (HIGH = Disabled)
GPIO.output(PAN_EN, GPIO.HIGH)
GPIO.output(TILT_EN, GPIO.HIGH)

print("---")
print("SCRIPT FINISHED")
print("---")
