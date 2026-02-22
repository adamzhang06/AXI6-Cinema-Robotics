#!/usr/bin/env python3
"""
Motor Velocity Ramp Test
========================
Slowly ramps up motor velocity from 0, displaying a live graph.
Press Ctrl+C to stop. Motor is safely disabled on exit.

Configurable:
  STEP_SIZE       - Steps per motor command (1 = same as socket system)
  ACCEL           - Motor acceleration (fullstep)
  RAMP_RATE       - How much speed increases per second
  LOOP_HZ         - Motor command loop frequency
  DIRECTION       - 1 = forward, -1 = reverse
"""

import time
import matplotlib
matplotlib.use('TkAgg')          # Use Tk backend for live window
import matplotlib.pyplot as plt
from collections import deque
from tmc_driver import (
    Tmc2209, Loglevel, MovementAbsRel,
    TmcEnableControlPin, TmcMotionControlStepDir,
)

# ==================== CONFIGURATION ====================
STEP_SIZE   = 1          # Steps per command (match socket system = 1)
ACCEL       = 400        # Motor acceleration (fullstep)
RAMP_RATE   = 10         # Speed increase per second (velocity units/s)
LOOP_HZ     = 500        # Motor loop frequency
DIRECTION   = 1          # 1 = forward, -1 = reverse
MAX_SPEED   = 1000       # Safety cap

# ==================== MOTOR SETUP ====================
print("\n" + "=" * 50)
print("  Motor Velocity Ramp Test")
print(f"  Step Size: {STEP_SIZE} | Accel: {ACCEL}")
print(f"  Ramp Rate: {RAMP_RATE} vel/s | Loop: {LOOP_HZ}Hz")
print(f"  Direction: {'Forward' if DIRECTION > 0 else 'Reverse'}")
print("  Press Ctrl+C to stop.")
print("=" * 50 + "\n")

motor = Tmc2209(
    TmcEnableControlPin(21),
    TmcMotionControlStepDir(16, 20),
    loglevel=Loglevel.INFO
)
motor.acceleration_fullstep = ACCEL
motor.set_motor_enabled(True)

# ==================== LIVE GRAPH SETUP ====================
plt.ion()                        # Interactive mode
fig, ax = plt.subplots(figsize=(10, 5))
fig.patch.set_facecolor('#1a1a2e')
ax.set_facecolor('#16213e')

MAX_POINTS = 500                 # Rolling window of data points
times = deque(maxlen=MAX_POINTS)
velocities = deque(maxlen=MAX_POINTS)

line, = ax.plot([], [], color='#00ff88', linewidth=2, label='Velocity')
ax.set_xlabel('Time (s)', color='white', fontsize=12)
ax.set_ylabel('Motor Speed (fullstep)', color='white', fontsize=12)
ax.set_title('Motor Velocity Ramp', color='white', fontsize=14, fontweight='bold')
ax.tick_params(colors='white')
ax.spines['bottom'].set_color('#444')
ax.spines['left'].set_color('#444')
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)
ax.legend(loc='upper left', facecolor='#16213e', edgecolor='#444', labelcolor='white')
ax.grid(True, alpha=0.2, color='white')

plt.tight_layout()
plt.show(block=False)

# ==================== RAMP LOOP ====================
current_speed = 0.0
start_time = time.time()
loop_period = 1.0 / LOOP_HZ
last_graph_update = 0
GRAPH_UPDATE_HZ = 30             # Refresh graph at 30fps

print("[MOTOR] Ramping up...")

try:
    while True:
        loop_start = time.time()
        elapsed = loop_start - start_time

        # Ramp speed linearly
        current_speed = min(elapsed * RAMP_RATE, MAX_SPEED)
        int_speed = int(current_speed)

        # Set motor speed and fire steps
        motor.max_speed_fullstep = int_speed
        if int_speed > 0:
            motor.run_to_position_steps(
                STEP_SIZE * DIRECTION,
                MovementAbsRel.RELATIVE
            )

        # Record data
        times.append(elapsed)
        velocities.append(int_speed)

        # Update graph at 30fps (not every motor tick)
        if elapsed - last_graph_update > 1.0 / GRAPH_UPDATE_HZ:
            last_graph_update = elapsed
            line.set_data(list(times), list(velocities))
            ax.set_xlim(max(0, elapsed - 20), elapsed + 1)
            ax.set_ylim(0, max(int_speed + 50, 100))
            fig.canvas.draw_idle()
            fig.canvas.flush_events()

            # Print status
            print(f"\r  ⏱ {elapsed:6.1f}s | 🏎 Speed: {int_speed:4d} | Steps/cmd: {STEP_SIZE}", end="", flush=True)

        # Precision timing
        sleep_time = loop_period - (time.time() - loop_start)
        if sleep_time > 0:
            time.sleep(sleep_time)

except KeyboardInterrupt:
    print(f"\n\n[STOP] Final speed: {int(current_speed)} at {time.time() - start_time:.1f}s")

finally:
    motor.max_speed_fullstep = 0
    motor.set_motor_enabled(False)
    print("[SHUTDOWN] Motor safely disabled.")
    plt.ioff()
    plt.show()  # Keep graph visible after stopping
