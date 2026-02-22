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
    # Segment 1: 0° → 90° over 3s (ease in/out)
    (0.00,   0.0),
    (0.05,   0.1),
    (0.10,   0.3),
    (0.15,   0.7),
    (0.20,   1.3),
    (0.25,   2.0),
    (0.30,   2.8),
    (0.35,   3.8),
    (0.40,   4.9),
    (0.45,   6.1),
    (0.50,   7.5),
    (0.55,   9.0),
    (0.60,  10.6),
    (0.65,  12.3),
    (0.70,  14.1),
    (0.75,  16.1),
    (0.80,  18.1),
    (0.85,  20.2),
    (0.90,  22.4),
    (0.95,  24.7),
    (1.00,  27.0),
    (1.05,  29.4),
    (1.10,  31.8),
    (1.15,  34.2),
    (1.20,  36.7),
    (1.25,  39.1),
    (1.30,  41.5),
    (1.35,  43.9),
    (1.40,  46.2),
    (1.45,  48.5),
    (1.50,  50.7),
    (1.55,  52.8),
    (1.60,  54.9),
    (1.65,  56.9),
    (1.70,  58.8),
    (1.75,  60.6),
    (1.80,  62.4),
    (1.85,  64.0),
    (1.90,  65.6),
    (1.95,  67.1),
    (2.00,  68.5),
    (2.05,  69.8),
    (2.10,  71.0),
    (2.15,  72.2),
    (2.20,  73.3),
    (2.25,  74.3),
    (2.30,  75.3),
    (2.35,  76.2),
    (2.40,  77.1),
    (2.45,  77.9),
    (2.50,  78.7),
    (2.55,  79.4),
    (2.60,  80.1),
    (2.65,  80.8),
    (2.70,  81.5),
    (2.75,  82.2),
    (2.80,  82.9),
    (2.85,  83.7),
    (2.90,  84.6),
    (2.95,  85.8),
    (3.00,  90.0),
    # Segment 2: 90° → 270° over 1s (ease in/out)
    (3.05,  92.7),
    (3.10,  97.2),
    (3.15, 103.5),
    (3.20, 111.6),
    (3.25, 121.5),
    (3.30, 132.9),
    (3.35, 145.2),
    (3.40, 157.8),
    (3.45, 170.1),
    (3.50, 180.0),
    (3.55, 189.9),
    (3.60, 199.8),
    (3.65, 209.1),
    (3.70, 217.1),
    (3.75, 225.0),
    (3.80, 232.9),
    (3.85, 240.6),
    (3.90, 248.4),
    (3.95, 256.5),
    (4.00, 270.0),
    # Segment 3: 270° → 360° over 3s (ease in/out)
    (4.05, 271.2),
    (4.10, 272.2),
    (4.15, 273.0),
    (4.20, 273.8),
    (4.25, 274.5),
    (4.30, 275.2),
    (4.35, 275.9),
    (4.40, 276.6),
    (4.45, 277.3),
    (4.50, 278.0),
    (4.55, 278.8),
    (4.60, 279.6),
    (4.65, 280.5),
    (4.70, 281.5),
    (4.75, 282.6),
    (4.80, 283.8),
    (4.85, 285.0),
    (4.90, 286.4),
    (4.95, 287.8),
    (5.00, 289.3),
    (5.05, 290.9),
    (5.10, 292.5),
    (5.15, 294.2),
    (5.20, 295.9),
    (5.25, 297.7),
    (5.30, 299.4),
    (5.35, 301.2),
    (5.40, 303.0),
    (5.45, 304.7),
    (5.50, 306.4),
    (5.55, 308.1),
    (5.60, 309.7),
    (5.65, 311.2),
    (5.70, 312.7),
    (5.75, 314.1),
    (5.80, 315.4),
    (5.85, 316.7),
    (5.90, 317.9),
    (5.95, 319.0),
    (6.00, 320.0),
    (6.05, 321.0),
    (6.10, 322.0),
    (6.15, 323.0),
    (6.20, 324.0),
    (6.25, 325.1),
    (6.30, 326.2),
    (6.35, 327.4),
    (6.40, 328.8),
    (6.45, 330.3),
    (6.50, 332.1),
    (6.55, 334.0),
    (6.60, 336.2),
    (6.65, 338.5),
    (6.70, 341.1),
    (6.75, 343.9),
    (6.80, 347.0),
    (6.85, 350.2),
    (6.90, 353.6),
    (6.95, 357.0),
    (7.00, 360.0),
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
