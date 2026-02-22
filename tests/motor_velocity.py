#!/usr/bin/env python3
"""
Motor Velocity Ramp Test (Local Graph Only)
============================================
Simulates and displays the velocity ramp curve on a graph.
No motor hardware needed — just visualizes what the ramp looks like.

Usage:  python tests/motor_velocity.py
Press Ctrl+C to stop.
"""

import time
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from collections import deque

# ==================== CONFIGURATION ====================
RAMP_RATE   = 10         # Speed increase per second
MAX_SPEED   = 1000       # Safety cap

# ==================== LIVE GRAPH SETUP ====================
plt.ion()
fig, ax = plt.subplots(figsize=(10, 5))
fig.patch.set_facecolor('#1a1a2e')
ax.set_facecolor('#16213e')

MAX_POINTS = 500
times = deque(maxlen=MAX_POINTS)
velocities = deque(maxlen=MAX_POINTS)

line, = ax.plot([], [], color='#00ff88', linewidth=2, label='Commanded Speed')
ax.set_xlabel('Time (s)', color='white', fontsize=12)
ax.set_ylabel('Motor Speed', color='white', fontsize=12)
ax.set_title('Motor Velocity Ramp (Simulation)', color='white', fontsize=14, fontweight='bold')
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
print(f"\nVelocity Ramp Simulation | Rate: {RAMP_RATE} vel/s | Max: {MAX_SPEED}")
print("Press Ctrl+C to stop.\n")

start_time = time.time()

try:
    while True:
        elapsed = time.time() - start_time
        speed = min(int(elapsed * RAMP_RATE), MAX_SPEED)

        times.append(elapsed)
        velocities.append(speed)

        line.set_data(list(times), list(velocities))
        ax.set_xlim(max(0, elapsed - 20), elapsed + 1)
        ax.set_ylim(0, max(speed + 50, 100))
        fig.canvas.draw_idle()
        fig.canvas.flush_events()

        print(f"\r  ⏱ {elapsed:6.1f}s | 🏎 Speed: {speed:4d}", end="", flush=True)
        time.sleep(1.0 / 30)  # 30fps refresh

except KeyboardInterrupt:
    print(f"\n\n[STOP] Final speed: {speed} at {elapsed:.1f}s")
    plt.ioff()
    plt.show()
