#!/usr/bin/env python3
"""
Motor Speed Controller (Mac → Pi via UDP)
==========================================
Control motor speed from your Mac using OpenCV sliders.
Sends commands to pi_motor_velocity.py (or pi_motor_server.py) on the Pi.

Usage:
  1. Start pi_motor_velocity.py on the Pi
  2. Run on Mac: python tests/motor_velocity.py

Controls:
  - Speed slider: 0-500 (motor speed in fullsteps)
  - Direction: 0 = Reverse, 1 = Stop, 2 = Forward
  - Accel slider: 100-2000 (motor acceleration)
  - Press Q to quit
"""

import cv2
import numpy as np
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

# ==================== OPENCV CONTROL WINDOW ====================
WINDOW = "Motor Speed Controller"
cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WINDOW, 600, 350)

cv2.createTrackbar("Speed", WINDOW, 0, 500, lambda x: None)
cv2.createTrackbar("Direction", WINDOW, 1, 2, lambda x: None)  # 0=Rev, 1=Stop, 2=Fwd
cv2.createTrackbar("Accel", WINDOW, 400, 2000, lambda x: None)

SEND_HZ = 100

def send_command(speed, direction, accel):
    """Send command in the same format pi_motor_server.py expects."""
    payload = json.dumps({
        "slide": {"speed": 0, "dir": 0, "accel": 400},
        "pan":   {"speed": speed, "dir": direction, "accel": accel},
        "tilt":  {"speed": 0, "dir": 0, "accel": 400}
    })
    udp_sock.sendto(payload.encode('utf-8'), (PI_IP, PI_PORT))

print(f"\n[READY] Sending to Pi @ {PI_IP}:{PI_PORT}")
print("Drag sliders to control motor. Press Q to quit.\n")

loop_period = 1.0 / SEND_HZ

try:
    while True:
        loop_start = time.time()

        speed = cv2.getTrackbarPos("Speed", WINDOW)
        dir_val = cv2.getTrackbarPos("Direction", WINDOW)
        accel = max(cv2.getTrackbarPos("Accel", WINDOW), 100)  # Minimum 100

        # Map slider: 0 = -1 (reverse), 1 = 0 (stop), 2 = +1 (forward)
        direction = dir_val - 1

        # If direction is 0 (stop), force speed to 0 too
        if direction == 0:
            speed = 0

        send_command(speed, direction, accel)

        # Draw status display
        frame = np.zeros((350, 600, 3), dtype=np.uint8)
        frame[:] = (30, 30, 40)

        dir_text = {-1: "< REVERSE", 0: "STOPPED", 1: "FORWARD >"}[direction]
        dir_color = {-1: (100, 100, 255), 0: (128, 128, 128), 1: (100, 255, 100)}[direction]

        cv2.putText(frame, "AXI6 Motor Speed Controller", (30, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(frame, f"Target: {PI_IP}:{PI_PORT}", (30, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)

        cv2.putText(frame, f"Speed: {speed}", (30, 130),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 136), 3)
        cv2.putText(frame, dir_text, (30, 185),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, dir_color, 2)
        cv2.putText(frame, f"Accel: {accel}", (30, 230),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 100), 2)

        # Speed bar
        bar_w = int((speed / 500) * 540)
        cv2.rectangle(frame, (30, 270), (30 + bar_w, 300), (0, 255, 136), -1)
        cv2.rectangle(frame, (30, 270), (570, 300), (100, 100, 100), 1)

        # Sending indicator
        if speed > 0:
            cv2.circle(frame, (570, 40), 8, (0, 255, 0), -1)
        else:
            cv2.circle(frame, (570, 40), 8, (60, 60, 60), -1)

        cv2.imshow(WINDOW, frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

        sleep_time = loop_period - (time.time() - loop_start)
        if sleep_time > 0:
            time.sleep(sleep_time)

except KeyboardInterrupt:
    pass

finally:
    send_command(0, 0, 400)
    print("[SHUTDOWN] Stop command sent to Pi.")
    cv2.destroyAllWindows()
