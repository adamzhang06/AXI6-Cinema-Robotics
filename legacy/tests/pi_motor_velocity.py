#!/usr/bin/env python3
"""
Motor Velocity Test Server (Pi-side)
======================================
Standalone motor server for velocity testing.
Listens for UDP commands in the same format as pi_motor_server.py.

Run on Pi:  python tests/pi_motor_velocity.py
Then run motor_velocity.py on Mac to control speed.

Press Ctrl+C to stop.
"""

import socket
import json
import time
import threading
from tmc_driver import Tmc2209, Loglevel, MovementAbsRel, TmcEnableControlPin, TmcMotionControlStepDir

# --- NETWORK ---
UDP_IP = "0.0.0.0"
UDP_PORT = 5005
BEACON_PORT = 5006

class MotorState:
    def __init__(self):
        self.speed = 0
        self.dir = 0
        self.accel = 400
        self.last_update = time.time()
        self.running = True

state = MotorState()

# --- UDP LISTENER ---
def udp_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(1.0)
    print(f"[NETWORK] Listening on port {UDP_PORT}...")

    while state.running:
        try:
            data, addr = sock.recvfrom(1024)
            command = json.loads(data.decode('utf-8'))

            if "pan" in command:
                state.speed = command["pan"].get("speed", 0)
                state.dir   = command["pan"].get("dir", 0)
                state.accel = command["pan"].get("accel", 400)

            state.last_update = time.time()
        except socket.timeout:
            pass
        except Exception:
            pass

# --- BEACON BROADCASTER ---
def beacon_broadcaster():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    print(f"[BEACON] Broadcasting on port {BEACON_PORT}...")
    while state.running:
        try:
            beacon = json.dumps({"service": "axi6_motor_server", "port": UDP_PORT}).encode('utf-8')
            sock.sendto(beacon, ('<broadcast>', BEACON_PORT))
        except Exception:
            pass
        time.sleep(2)

# --- MOTOR LOOP ---
def motor_loop():
    pan_motor = Tmc2209(TmcEnableControlPin(21), TmcMotionControlStepDir(16, 20), loglevel=Loglevel.INFO)
    pan_motor.acceleration_fullstep = 400
    pan_motor.set_motor_enabled(True)

    print("[MOTOR] Ready. Waiting for commands...")

    while state.running:
        start_time = time.time()

        # Failsafe: no packets for 0.5s → stop
        if time.time() - state.last_update > 0.5:
            pan_motor.max_speed_fullstep = 0
        else:
            # Update acceleration
            if state.accel > 0 and pan_motor.acceleration_fullstep != state.accel:
                pan_motor.acceleration_fullstep = state.accel

            # Update speed
            pan_motor.max_speed_fullstep = state.speed

            # Fire step chunk
            if state.speed > 0 and state.dir != 0:
                steps = state.dir * 10
                pan_motor.run_to_position_steps(steps, MovementAbsRel.RELATIVE)

            # Print status every ~1 second
            if int(time.time() * 2) % 2 == 0 and int(start_time * 2) % 2 != 0:
                print(f"  Speed: {state.speed:4d} | Dir: {state.dir:+d} | Accel: {state.accel}")

        # 500Hz loop
        sleep_time = 0.002 - (time.time() - start_time)
        if sleep_time > 0:
            time.sleep(sleep_time)

    pan_motor.set_motor_enabled(False)
    print("\n[SHUTDOWN] Motor disabled.")

if __name__ == "__main__":
    print("\n" + "=" * 50)
    print("  AXI6 Motor Velocity Test Server")
    print(f"  Listening on {UDP_IP}:{UDP_PORT}")
    print("  Press Ctrl+C to stop.")
    print("=" * 50 + "\n")

    net_thread = threading.Thread(target=udp_listener, daemon=True)
    net_thread.start()

    beacon_thread = threading.Thread(target=beacon_broadcaster, daemon=True)
    beacon_thread.start()

    try:
        motor_loop()
    except KeyboardInterrupt:
        state.running = False
