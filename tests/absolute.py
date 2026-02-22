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

# ==================== TRAPEZOIDAL POINT GENERATOR ====================
def generate_trapezoidal_points(duration, theta, n, num_points=100):
    """
    Generate (time, position) waypoints for a trapezoidal velocity profile.
    
    duration:    total move time in seconds
    theta:       total angle in degrees
    n:           easing factor (2 = full triangle, higher = sharper ramp)
    num_points:  number of waypoints to generate
    
    Returns: list of (time_s, position_deg) tuples
    
    Physics:
      v_max = (theta/duration) * n/(n-1)
      a     = v_max * n / duration
      t_ramp = duration / n
      
      Accel phase:   pos(t) = 0.5 * a * t²
      Cruise phase:  pos(t) = pos_accel + v_max * (t - t_ramp)
      Decel phase:   pos(t) = theta - 0.5 * a * (duration - t)²
    """
    v_avg = theta / duration
    v_max = v_avg * (n / (n - 1))
    a = (v_max / duration) * n
    t_ramp = duration / n  # time for accel (and decel)
    
    # Position at end of accel phase
    pos_at_accel_end = 0.5 * a * t_ramp * t_ramp
    
    points = []
    for i in range(num_points + 1):
        t = (i / num_points) * duration
        
        if t <= t_ramp:
            # Acceleration phase: p = 0.5 * a * t²
            pos = 0.5 * a * t * t
        elif t <= duration - t_ramp:
            # Cruise phase: p = pos_accel + v_max * (t - t_ramp)
            pos = pos_at_accel_end + v_max * (t - t_ramp)
        else:
            # Deceleration phase: p = theta - 0.5 * a * (duration - t)²
            t_remaining = duration - t
            pos = theta - 0.5 * a * t_remaining * t_remaining
        
        points.append((round(t, 4), round(pos, 4)))
    
    return points


# ==================== WAYPOINTS ====================
# Generate trapezoidal trajectory: 5 seconds, 360°, n=2 (triangle)
WAYPOINTS = generate_trapezoidal_points(duration=5, theta=360, n=3, num_points=50)

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
