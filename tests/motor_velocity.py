#!/usr/bin/env python3
"""
Motor Speed Controller (Mac → Pi via UDP)
==========================================
Control motor speed from your Mac using an OpenCV slider.
Sends commands to pi_motor_server.py on the Pi.

Usage:
  1. Start pi_motor_server.py on the Pi
  2. Run: python tests/motor_velocity.py

Controls:
  - Drag the Speed slider to set motor speed (0-500)
  - Drag the Direction slider: 0 = Reverse, 1 = Stop, 2 = Forward
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

# ==================== CONFIGURATION ====================
ACCEL = 400
SEND_HZ = 100

# ==================== OPENCV CONTROL WINDOW ====================
WINDOW = "Motor Speed Controller"
cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WINDOW, 600, 300)

cv2.createTrackbar("Speed", WINDOW, 0, 500, lambda x: None)
cv2.createTrackbar("Direction", WINDOW, 1, 2, lambda x: None)  # 0=Rev, 1=Stop, 2=Fwd

def send_command(speed, direction):
    payload = json.dumps({
        "slide": {"speed": 0, "dir": 0, "accel": 400},
        "pan":   {"speed": speed, "dir": direction, "accel": ACCEL},
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

        # Map slider: 0 = -1 (reverse), 1 = 0 (stop), 2 = +1 (forward)
        direction = dir_val - 1

        if direction == 0:
            speed = 0

        send_command(speed, direction)

        # Draw status display
        frame = np.zeros((300, 600, 3), dtype=np.uint8)
        frame[:] = (30, 30, 40)

        dir_text = {-1: "< REVERSE", 0: "STOPPED", 1: "FORWARD >"}[direction]
        dir_color = {-1: (100, 100, 255), 0: (128, 128, 128), 1: (100, 255, 100)}[direction]

        cv2.putText(frame, "AXI6 Motor Speed Controller", (30, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(frame, f"Target: {PI_IP}:{PI_PORT}", (30, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)

        cv2.putText(frame, f"Speed: {speed}", (30, 140),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 136), 3)
        cv2.putText(frame, dir_text, (30, 200),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, dir_color, 2)

        # Speed bar
        bar_w = int((speed / 500) * 540)
        cv2.rectangle(frame, (30, 240), (30 + bar_w, 270), (0, 255, 136), -1)
        cv2.rectangle(frame, (30, 240), (570, 270), (100, 100, 100), 1)

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
    send_command(0, 0)
    print("[SHUTDOWN] Stop command sent to Pi.")
    cv2.destroyAllWindows()
