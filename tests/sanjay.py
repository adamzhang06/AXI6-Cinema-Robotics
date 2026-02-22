# pylint: disable=wildcard-import
# pylint: disable=unused-wildcard-import
"""
Script to execute a mathematically perfect trapezoidal profile natively.
"""

import time
from tmc_driver.tmc_2209 import *

print("---")
print("SCRIPT START")
print("---")

# -----------------------------------------------------------------------
# 1. Math Function: Calculate Profile Parameters
# -----------------------------------------------------------------------
def calculate_trapezoidal_parameters(final_angle_deg, total_time, steps_per_rev=400, phases=3.0):
    """
    Calculates the required max velocity and acceleration to complete a move
    in the specified total_time, using a trapezoidal profile.
    """
    # Convert degrees to steps
    total_steps = (final_angle_deg / 360.0) * steps_per_rev
    
    # Define time spent accelerating (t_a)
    t_a = total_time / phases
    
    # Calculate Max Velocity (steps/sec) and Acceleration (steps/sec^2)
    v_max = total_steps / (total_time - t_a)
    a = v_max / t_a
    
    return int(round(total_steps)), v_max, a

# -----------------------------------------------------------------------
# 2. Get User Input
# -----------------------------------------------------------------------
target_angle = float(input("Enter final angle in degrees (e.g., 360): "))
target_time = float(input("Enter total move time in seconds (e.g., 3.0): "))
target_phases = float(input("Enter total phases (e.g., 3.0 for rule of thirds): "))

total_steps, required_v_max, required_accel = calculate_trapezoidal_parameters(
    final_angle_deg=target_angle, 
    total_time=target_time, 
    steps_per_rev=400,
    phases=target_phases
)

print(f"\nCalculated Profile:")
print(f"Target Steps: {total_steps}")
print(f"Max Speed: {required_v_max:.2f} steps/sec")
print(f"Acceleration: {required_accel:.2f} steps/sec^2\n")

# -----------------------------------------------------------------------
# 3. Initiate the Tmc2209 class
# -----------------------------------------------------------------------
tmc = Tmc2209(
    TmcEnableControlPin(21),
    TmcMotionControlStepDir(16, 20),
    loglevel=Loglevel.INFO,
)

tmc.movement_abs_rel = MovementAbsRel.ABSOLUTE
tmc.set_motor_enabled(True)

# -----------------------------------------------------------------------
# 4. Execute the Native Trajectory
# -----------------------------------------------------------------------
# Move to starting position (0) at a default speed
tmc.max_speed_fullstep = 500
tmc.acceleration_fullstep = 2000
tmc.run_to_position_steps(0)

# Apply our mathematically calculated trapezoidal parameters
tmc.max_speed_fullstep = required_v_max
tmc.acceleration_fullstep = required_accel

print("Executing smooth hardware trajectory...")
start_time_real = time.time()

# Send the single command. The library/hardware handles the smooth step generation!
tmc.run_to_position_steps(total_steps)

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