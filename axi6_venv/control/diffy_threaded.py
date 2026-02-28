import time
import math
import threading
import RPi.GPIO as GPIO

# Constants
STEPS_PER_REV = 400

PAN_STEP = 16
PAN_DIR = 20
PAN_EN = 21

TILT_STEP = 13
TILT_DIR = 19
TILT_EN = 26

print("---")
print("SCRIPT START")
print("---")

def motion_profile(total_steps, total_time, phases=3.0):
    """
    Returns a function v(t) for velocity at time t, and a function pos(t) for position at time t.
    """
    if total_steps == 0:
        return lambda t: 0, lambda t: 0

    t_a = total_time / phases # Time spent accelerating
    t_c = total_time - t_a # Time spent at constant speed
    v_max = total_steps / (total_time - t_a)
    a = v_max / t_a
    P_a = 0.5 * a * (t_a**2)
    P_c = P_a + v_max * (t_c - t_a)

    def pos(t):
        if t <= t_a:
            return 0.5 * a * (t**2)
        elif t <= t_c:
            return P_a + v_max * (t - t_a)
        elif t <= total_time:
            dt = t - t_c
            return P_c + v_max * dt - 0.5 * a * (dt**2)
        else:
            return total_steps

    def vel(t):
        if t <= t_a:
            return a * t
        elif t <= t_c:
            return v_max
        elif t <= total_time:
            dt = t - t_c
            return v_max - a * dt
        else:
            return 0

    return vel, pos

pan_angle = float(input("Enter Pan angle in degrees (e.g., 360): "))
tilt_angle = float(input("Enter Tilt angle in degrees (e.g., 0): "))
target_time = float(input("Enter total move time in seconds (e.g., 5.0): "))

motor_a_angle = tilt_angle + pan_angle
motor_b_angle = tilt_angle - pan_angle

a_steps_float = abs((motor_a_angle / 360.0) * STEPS_PER_REV)
b_steps_float = abs((motor_b_angle / 360.0) * STEPS_PER_REV)

a_direction_forward = (motor_a_angle >= 0)
b_direction_forward = (motor_b_angle >= 0)

a_vel_fn, a_pos_fn = motion_profile(a_steps_float, target_time, phases=3.0)
b_vel_fn, b_pos_fn = motion_profile(b_steps_float, target_time, phases=3.0)

print(f"\nCalculated Motor A: {int(a_steps_float)} steps | Motor B: {int(b_steps_float)} steps.")

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup([PAN_STEP, PAN_DIR, PAN_EN, TILT_STEP, TILT_DIR, TILT_EN], GPIO.OUT)

# Enable the motors (LOW = Enabled for TMC2209)
GPIO.output(PAN_EN, GPIO.LOW)
GPIO.output(TILT_EN, GPIO.LOW)

GPIO.output(PAN_DIR, GPIO.HIGH if a_direction_forward else GPIO.LOW)
GPIO.output(TILT_DIR, GPIO.LOW if b_direction_forward else GPIO.HIGH) # Inverted

start_time_real = time.time() + 0.5  # Give threads 500ms to spin up

def motor_position_loop(step_pin, pos_fn, total_steps, total_time):
    position = 0
    last_step_time = time.time()
    start = start_time_real
    while True:
        now = time.time()
        t = now - start
        target_pos = int(round(pos_fn(t)))
        if position < target_pos and position < int(total_steps):
            # Step motor
            GPIO.output(step_pin, GPIO.HIGH)
            time.sleep(0.000001)
            GPIO.output(step_pin, GPIO.LOW)
            position += 1
        if t >= total_time or position >= int(total_steps):
            break
        # Sleep a bit to reduce CPU usage, but not too much to miss steps
        time.sleep(0.0001)


print("Launching parallel threads...")

# Create python threads pointing to our position-based loop
pan_thread = threading.Thread(target=motor_position_loop, args=(PAN_STEP, a_pos_fn, a_steps_float, target_time))
tilt_thread = threading.Thread(target=motor_position_loop, args=(TILT_STEP, b_pos_fn, b_steps_float, target_time))

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