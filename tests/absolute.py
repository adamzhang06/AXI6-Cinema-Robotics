"""
Absolute Position Trajectory via VACTUAL
==========================================
Define a series of (time, position) waypoints.
At each interval, calculates the VACTUAL velocity needed to 
arrive at the next position exactly on time.

The motor follows the trajectory point-by-point.
"""

import time
from tmc_driver import (
    Tmc2209,
    Loglevel,
    Board,
    tmc_gpio,
    MovementAbsRel,
    TmcEnableControlPin,
    TmcMotionControlVActual,
)
from tmc_driver.com import TmcComUart

# ==================== CONSTANTS ====================
STEPS_PER_REV = 400  # microsteps per 360°
VACTUAL_TO_RPS = 0.715  # Approximate: 1 RPS ≈ VACTUAL of ~1398 (12MHz clock)
                         # Adjust this based on your clock. VACTUAL = velocity * (2^24 / fclk)

# ==================== WAYPOINTS ====================
# Define your trajectory as (time_seconds, position_degrees)
# The motor will move to each position, arriving exactly at the specified time.
# Time must be strictly increasing. First point should be (0, starting_position).

WAYPOINTS = [
    # (time_s, position_deg)
    (0,    0),      # Start at 0°
    (3,    90),     # Reach 90° at t=3s
    (8,    270),    # Reach 270° at t=8s
    (11,   360),      # Return to 0° at t=13s
]

def deg_to_steps(deg):
    """Convert degrees to microstep count."""
    return deg * STEPS_PER_REV / 360.0

def steps_to_vactual(steps_per_sec):
    """Convert steps/second to VACTUAL register value.
    TMC2209 VACTUAL: velocity = vactual * (fclk / 2^24)
    For 12MHz clock: 1 step/s ≈ VACTUAL of 1.398
    Adjust the multiplier if your timing is off."""
    return int(steps_per_sec * 1.398)

# ==================== MOTOR SETUP ====================
print("---")
print("AXI6 Absolute Position Trajectory")
print(f"Waypoints: {len(WAYPOINTS)} points, {WAYPOINTS[-1][0]}s total")
print("---\n")

UART_PORT = {
    Board.RASPBERRY_PI: "/dev/serial0",
    Board.RASPBERRY_PI5: "/dev/ttyAMA0",
    Board.NVIDIA_JETSON: "/dev/ttyTHS1",
}

tmc = Tmc2209(
    TmcEnableControlPin(21),
    TmcMotionControlVActual(),
    TmcComUart(UART_PORT.get(tmc_gpio.BOARD, "/dev/serial0")),
    loglevel=Loglevel.INFO,
)

tmc.set_motor_enabled(True)
print("[MOTOR] Enabled\n")

# ==================== TRAJECTORY EXECUTION ====================
def run_trajectory(waypoints):
    """Execute the waypoint trajectory using VACTUAL velocity control."""
    
    print("  Time  |  Target  |  Velocity  |  VACTUAL")
    print("  ------|----------|------------|--------")
    
    t_start = time.time()
    
    for i in range(len(waypoints) - 1):
        t_now, pos_now = waypoints[i]
        t_next, pos_next = waypoints[i + 1]
        
        dt = t_next - t_now
        if dt <= 0:
            continue
        
        # Calculate required velocity (steps/sec)
        delta_steps = deg_to_steps(pos_next) - deg_to_steps(pos_now)
        steps_per_sec = delta_steps / dt
        vactual = steps_to_vactual(steps_per_sec)
        
        print(f"  {t_now:5.1f}s | {pos_now:6.1f}° → {pos_next:6.1f}° | {steps_per_sec:+8.1f} sps | {vactual:+6d}")
        
        # Set velocity
        tmc.tmc_mc.set_vactual_dur(vactual)
        
        # Wait until next waypoint time
        target_time = t_start + t_next
        remaining = target_time - time.time()
        if remaining > 0:
            time.sleep(remaining)
    
    # Stop
    tmc.tmc_mc.set_vactual_dur(0)
    
    actual_total = time.time() - t_start
    print(f"\n  Done! Total time: {actual_total:.2f}s (expected {waypoints[-1][0]}s)")

# ==================== RUN ====================
try:
    run_trajectory(WAYPOINTS)
except KeyboardInterrupt:
    print("\nInterrupted!")
    tmc.tmc_mc.set_vactual_dur(0)
finally:
    tmc.tmc_mc.set_vactual_dur(0)
    tmc.set_motor_enabled(False)
    del tmc
    print("[SHUTDOWN] Motor disabled.")
