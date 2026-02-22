#!/usr/bin/env python3
"""
Motor Velocity Ramp Test (Socket Version)
==========================================
Runs on the Mac — sends velocity ramp commands to the Pi via UDP.
Uses the existing pi_motor_server.py on the Pi.
Terminal output only — no matplotlib needed.

Usage:
  1. Start pi_motor_server.py on the Pi
  2. Run this script on the Mac:  python socket/pi_motor_velocity.py

Press Ctrl+C to stop. Sends a stop command to the Pi on exit.

Configurable:
  STEP_SIZE       - Motor step direction (1 or -1)
  ACCEL           - Motor acceleration sent to Pi
  RAMP_RATE       - How much speed increases per second
  MAX_SPEED       - Safety cap
  DIRECTION       - 1 = forward, -1 = reverse
"""

import socket
import json
import time

# ==================== NETWORK SETUP ====================
BEACON_PORT = 5006
PI_PORT = 5005

def discover_pi(timeout=2):
    """Listen for the Pi's beacon broadcast to auto-discover its IP."""
    print(f"[NETWORK] Searching for Pi motor server...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('', BEACON_PORT))
    sock.settimeout(timeout)
    try:
        data, addr = sock.recvfrom(1024)
        msg = json.loads(data.decode('utf-8'))
        if msg.get('service') == 'axi6_motor_server':
            pi_ip = addr[0]
            pi_port = msg.get('port', PI_PORT)
            print(f"[NETWORK] ✅ Found Pi at {pi_ip}:{pi_port}")
            sock.close()
            return pi_ip, pi_port
    except socket.timeout:
        print(f"[NETWORK] ❌ No Pi found after {timeout}s. Using fallback IP.")
        sock.close()
        return "10.186.143.105", PI_PORT
    sock.close()
    return None, PI_PORT

PI_IP, PI_PORT = discover_pi()
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ==================== CONFIGURATION ====================
STEP_SIZE   = 1          # Direction magnitude (1 or -1, Pi handles steps)
ACCEL       = 400        # Motor acceleration
RAMP_RATE   = 10         # Speed increase per second
MAX_SPEED   = 1000       # Safety cap
DIRECTION   = 1          # 1 = forward, -1 = reverse
SEND_HZ     = 100        # UDP send rate

# ==================== RAMP LOOP ====================
print("\n" + "=" * 50)
print("  Motor Velocity Ramp Test (Socket)")
print(f"  Target: {PI_IP}:{PI_PORT}")
print(f"  Ramp Rate: {RAMP_RATE} vel/s | Send: {SEND_HZ}Hz")
print(f"  Direction: {'Forward' if DIRECTION > 0 else 'Reverse'}")
print("  Press Ctrl+C to stop.")
print("=" * 50 + "\n")

current_speed = 0.0
start_time = time.time()
loop_period = 1.0 / SEND_HZ

def send_command(speed, direction, accel):
    """Send a motor command to the Pi via UDP."""
    payload = json.dumps({
        "slide": {"speed": 0, "dir": 0, "accel": 400},
        "pan":   {"speed": int(speed), "dir": direction, "accel": accel},
        "tilt":  {"speed": 0, "dir": 0, "accel": 400}
    })
    udp_sock.sendto(payload.encode('utf-8'), (PI_IP, PI_PORT))

print("[RAMP] Starting velocity ramp...")

try:
    while True:
        loop_start = time.time()
        elapsed = loop_start - start_time

        # Ramp speed linearly
        current_speed = min(elapsed * RAMP_RATE, MAX_SPEED)
        int_speed = int(current_speed)

        # Send to Pi
        send_command(int_speed, STEP_SIZE * DIRECTION, ACCEL)

        # Print status
        print(f"\r  ⏱ {elapsed:6.1f}s | 🏎 Speed: {int_speed:4d} | Dir: {DIRECTION:+d}", end="", flush=True)

        # Timing
        sleep_time = loop_period - (time.time() - loop_start)
        if sleep_time > 0:
            time.sleep(sleep_time)

except KeyboardInterrupt:
    print(f"\n\n[STOP] Final speed: {int(current_speed)} at {time.time() - start_time:.1f}s")

finally:
    send_command(0, 0, ACCEL)
    print("[SHUTDOWN] Stop command sent to Pi.")
