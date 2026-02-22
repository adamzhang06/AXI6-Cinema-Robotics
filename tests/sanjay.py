# pylint: disable=wildcard-import
# pylint: disable=unused-wildcard-import
"""
Script to generate and execute a trapezoidal velocity profile for a stepper motor.
"""

import time
import math
from tmc_driver.tmc_2209 import *

print("---")
print("SCRIPT START")
print("---")

# -----------------------------------------------------------------------
# 1. Math Function: Generate Trapezoidal Profile
# -----------------------------------------------------------------------
def generate_trapezoidal_profile(final_angle_deg, total_time, dt=0.05, steps_per_rev=1600):
    """
    Generates a list of (time, position) tuples forming a trapezoidal velocity profile.
    Uses the 1/3 rule: 1/3 time accelerating, 1/3 cruising, 1/3 decelerating.
    """
    # Convert degrees to steps
    total_steps = (final_angle_deg / 360.0) * steps_per_rev
    
    # Define phase durations
    t_a = total_time / 3.0
    t_cruise_end = total_time - t_a
    
    # Calculate Max Velocity and Acceleration
    v_max = total_steps / (total_time - t_a)
    a = v_max / t_a
    
    trajectory = []
    t = 0.0
    
    while t <= total_time:
        if t <= t_a:
            # Phase 1: Acceleration
            pos = 0.5 * a * t**2
        elif t <= t_cruise_end:
            # Phase 2: Constant Velocity
            pos = (0.5 * a * t_a**2) + v_max * (t - t_a)
        else:
            # Phase 3: Deceleration
            t_dec = t - t_cruise_end 
            pos_at_cruise_end = (0.5 * a * t_a**2) + v_max * (t_cruise_end - t_a)
            pos = pos_at_cruise_end + (v_max * t_dec) - (0.5 * a * t_dec**2)
            
        trajectory.append((t, int(round(pos))))
        t += dt
        
    # Ensure the final point hits the exact target time and position
    if trajectory[-1][0] < total_time:
        trajectory.append((total_time, int(round(total_steps))))
        
    return trajectory

# -----------------------------------------------------------------------
# 2. Get User Input and Generate Trajectory
# -----------------------------------------------------------------------
target_angle = float(input("Enter final angle in degrees (e.g., 360): "))
target_time = float(input("Enter total move time in seconds (e.g., 3.0): "))

# Updated to use 1600 steps per revolution
trajectory = generate_trapezoidal_profile(
    final_angle_deg=target_angle, 
    total_time=target_time, 
    dt=0.05, 
    steps_per_rev=1600 
)

# -----------------------------------------------------------------------
# 3. Initiate the Tmc2209 class
# -----------------------------------------------------------------------
tmc = Tmc2209(
    TmcEnableControlPin(21),
    TmcMotionControlStepDir(16, 20),
    loglevel=Loglevel.INFO,
)

tmc.movement_abs_rel = MovementAbsRel.ABSOLUTE

# Keep acceleration high so the driver strictly follows our mathematical profile
tmc.acceleration_fullstep = 100000 
tmc.set_motor_enabled(True)

# -----------------------------------------------------------------------
# 4. Execute the Trajectory
# -----------------------------------------------------------------------
# Set origin/start
tmc.max_speed_fullstep = 500
tmc.run_to_position_steps(0)

print(f"Executing {len(trajectory)} mathematical segments...")
start_time_real = time.time()

for i in range(1, len(trajectory)):
    t_target, pos_target = trajectory[i]
    t_prev, pos_prev = trajectory[i-1]
    
    dt_seg = t_target - t_prev
    dp_seg = pos_target - pos_prev
    
    required_speed = abs(dp_seg) / dt_seg if dt_seg > 0 else 0
    
    # Timing sync to prevent drift
    expected_start = start_time_real + t_prev
    now = time.time()
    if expected_start > now:
        time.sleep(expected_start - now)
        
    if required_speed > 0:
        tmc.max_speed_fullstep = required_speed
        tmc.run_to_position_steps(pos_target)

# -----------------------------------------------------------------------
# Clean up
# -----------------------------------------------------------------------
tmc.set_motor_enabled(False)
del tmc

print("---")
print("SCRIPT FINISHED")
print("---")