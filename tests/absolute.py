"""
Absolute Position Trajectory via VACTUAL
"""

import time
from tmc_driver import (
    Tmc2209, Loglevel, Board, tmc_gpio,
    TmcEnableControlPin, TmcMotionControlVActual,
)
from tmc_driver.com import TmcComUart

STEPS_PER_REV = 400

# ==================== WAYPOINTS ====================
# (time_seconds, position_degrees)
WAYPOINTS = [
    (0,    0),      # Start
    (1,   15),      # Ease in
    (2,   50),      # Cruising
    (3,   90),      # End segment 1
    (3.5, 180),     # Fast middle
    (4,  270),      # End segment 2
    (5,  310),      # Ease in
    (6,  345),      # Cruising
    (7,  360),      # Done
]

# ==================== HELPERS ====================
def deg_to_steps(deg):
    return deg * STEPS_PER_REV / 360.0

def steps_to_vactual(steps_per_sec):
    return int(steps_per_sec * 1.398)

# ==================== MOTOR SETUP ====================
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

# ==================== RUN TRAJECTORY ====================
def run_trajectory(waypoints):
    print(f"Running {len(waypoints)} waypoints, {waypoints[-1][0]}s total")
    t_start = time.time()

    for i in range(len(waypoints) - 1):
        t_now, pos_now = waypoints[i]
        t_next, pos_next = waypoints[i + 1]

        dt = t_next - t_now
        if dt <= 0:
            continue

        delta_steps = deg_to_steps(pos_next) - deg_to_steps(pos_now)
        vactual = steps_to_vactual(delta_steps / dt)

        print(f"  t={t_now:.1f}s | {pos_now:.0f}° → {pos_next:.0f}° | vactual={vactual:+d}")
        tmc.tmc_mc.set_vactual(vactual)

        remaining = (t_start + t_next) - time.time()
        if remaining > 0:
            time.sleep(remaining)

    tmc.tmc_mc.set_vactual(0)
    print(f"Done! {time.time() - t_start:.2f}s")

try:
    run_trajectory(WAYPOINTS)
except KeyboardInterrupt:
    print("\nInterrupted!")
finally:
    tmc.tmc_mc.set_vactual(0)
    tmc.set_motor_enabled(False)
    del tmc
    print("Motor disabled.")
