#!/usr/bin/env python3
"""
Motor Velocity Ramp Test (Pi-side / Terminal Only)
===================================================
Sends a linear velocity ramp to the Pi via UDP.
No GUI — pure terminal output. Runs on Pi or Mac.

Usage:
  1. Start pi_motor_server.py on the Pi
  2. Run: python tests/pi_motor_velocity.py

Press Ctrl+C to stop.
"""

import socket
import json
import time

# ==================== NETWORK SETUP ====================
BEACON_PORT = 5006
PI_PORT = 5005

def discover_pi(timeout=2):
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
        print(f"[NETWORK] ❌ No Pi found. Using fallback IP.")
        sock.close()
        return "10.186.143.105", PI_PORT
    sock.close()
    return None, PI_PORT

PI_IP, PI_PORT = discover_pi()
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ==================== CONFIGURATION ====================
STEP_SIZE   = 1          # Direction (1 = forward, -1 = reverse)
ACCEL       = 400        # Motor acceleration
RAMP_RATE   = 50         # Speed increase per second
MAX_SPEED   = 500        # Safety cap
SEND_HZ     = 100        # UDP send rate

# ==================== HELPERS ====================
def send_command(speed, direction, accel):
    payload = json.dumps({
        "slide": {"speed": 0, "dir": 0, "accel": 400},
        "pan":   {"speed": int(speed), "dir": direction, "accel": accel},
        "tilt":  {"speed": 0, "dir": 0, "accel": 400}
    })
    udp_sock.sendto(payload.encode('utf-8'), (PI_IP, PI_PORT))

# ==================== RAMP ====================
print(f"\n{'=' * 50}")
print(f"  Motor Velocity Ramp Test")
print(f"  Target: {PI_IP}:{PI_PORT}")
print(f"  Ramp: {RAMP_RATE} vel/s | Max: {MAX_SPEED} | Dir: {STEP_SIZE:+d}")
print(f"  Press Ctrl+C to stop.")
print(f"{'=' * 50}\n")

start_time = time.time()
loop_period = 1.0 / SEND_HZ
current_speed = 0

try:
    while True:
        loop_start = time.time()
        elapsed = loop_start - start_time

        current_speed = min(int(elapsed * RAMP_RATE), MAX_SPEED)

        send_command(current_speed, STEP_SIZE, ACCEL)

        bar_len = current_speed // 10
        bar = "#" * bar_len + "." * (50 - bar_len)
        print(f"\r  {elapsed:5.1f}s |{bar}| {current_speed:4d}", end="", flush=True)

        sleep_time = loop_period - (time.time() - loop_start)
        if sleep_time > 0:
            time.sleep(sleep_time)

except KeyboardInterrupt:
    print(f"\n\n[STOP] Final speed: {current_speed}")

finally:
    send_command(0, 0, ACCEL)
    print("[SHUTDOWN] Stop command sent to Pi.")
