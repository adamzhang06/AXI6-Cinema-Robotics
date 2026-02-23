"""
pi/server.py
------------
Main server running on the Raspberry Pi.
Receives pan/tilt spline trajectories from the Mac via socket, converts them into 
differential step timings, and executes them using precision GPIO pulsing.
Also receives live tracking commands via UDP for continuous velocity.

Reference: legacy/socket/pi_spline_motor.py, legacy/tests/sanjay.py
"""

import socket
import json
import struct
import time
import threading
import sys
import os
# Dynamically add the parent directory (project root) to python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from core.motor.axis import Axis
from core.motor.differential import DifferentialDrive
from core.motion.spline import SplineInterpolator

HOST = "0.0.0.0"
TCP_PORT = 5010
BEACON_PORT = 5011

running = True


# ─ Hardware Init ──────────────────────────────────────────────────────────────
# We use pure RPi.GPIO (configured inside Axis) for precise step timing
# Axis(en_pin, step_pin, dir_pin, invert)
axis_a = Axis(en_pin=21, step_pin=16, dir_pin=20)
axis_b = Axis(en_pin=26, step_pin=13, dir_pin=19, invert=True)

drive = DifferentialDrive(axis_a, axis_b)


# ─ Auto-Discovery Beacon ──────────────────────────────────────────────────────
def beacon_broadcaster():
    """Broadcasts a discovery beacon every 2 seconds."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    print(f"[BEACON] Broadcasting on UDP port {BEACON_PORT}...")

    while running:
        try:
            beacon = json.dumps({
                "service": "axi6_spline_motor",
                "port": TCP_PORT
            }).encode('utf-8')
            sock.sendto(beacon, ('<broadcast>', BEACON_PORT))
        except Exception:
            pass
        time.sleep(2)


# ─ TCP Server Setup ───────────────────────────────────────────────────────────


def receive_trajectory(conn):
    """Reads a length-prefixed JSON trajectory from a TCP connection."""
    length_bytes = b''
    while len(length_bytes) < 4:
        chunk = conn.recv(4 - len(length_bytes))
        if not chunk:
            raise ConnectionError("Connection closed while reading length header")
        length_bytes += chunk

    payload_length = struct.unpack('>I', length_bytes)[0]
    
    payload = b''
    while len(payload) < payload_length:
        chunk = conn.recv(min(4096, payload_length - len(payload)))
        if not chunk:
            raise ConnectionError("Connection closed while reading payload")
        payload += chunk

    data = json.loads(payload.decode('utf-8'))
    return data["a_fwd"], data["a_times"], data["b_fwd"], data["b_times"]



def execute_trajectory(a_fwd, a_times, b_fwd, b_times):
    """Execution is now just enqueuing the exact timings."""
    print(f"[MOTOR] Enqueueing differential steps: A({len(a_times)}), B({len(b_times)})")
    
    # Enqueue through the Drive wrapper
    drive.enqueue_timing(
        motor_a_forward=a_fwd,
        motor_a_times=a_times,
        motor_b_forward=b_fwd,
        motor_b_times=b_times
    )
    
    # 1. Splines come in as [t, pan_position] and [t, tilt_position]
    # We must convert Pan/Tilt into Motor_A / Motor_B
    # Pan = (A+B)/2 -> A+B = 2*Pan
    # Tilt = (A-B)/2 -> A-B = 2*Tilt
    # A = Pan + Tilt
    # B = Pan - Tilt
    
    a_spline = []
    b_spline = []

    # Assuming pan_spline and tilt_spline have the exact same time axis from the new UI
    for i in range(len(pan_spline)):
        t = pan_spline[i][0]
        pan_p = pan_spline[i][1]
        tilt_p = tilt_spline[i][1]
        
        a_p = pan_p + tilt_p
        b_p = pan_p - tilt_p
        
        a_spline.append((t, a_p))
        b_spline.append((t, b_p))
        
    a_fwd, a_times = generate_times(a_spline)
    b_fwd, b_times = generate_times(b_spline)
    
    # Enqueue through the Drive wrapper
    drive.enqueue_timing(
        motor_a_forward=a_fwd,
        motor_a_times=a_times,
        motor_b_forward=b_fwd,
        motor_b_times=b_times
    )


# ─ Main Loop ──────────────────────────────────────────────────────────────────
def main():
    global running

    # Start beacon in background
    beacon_thread = threading.Thread(target=beacon_broadcaster, daemon=True)
    beacon_thread.start()
    

    # Start TCP server
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, TCP_PORT))
    server.listen(1)
    server.settimeout(1.0)

    print(f"[SERVER] Listening for trajectory on TCP port {TCP_PORT}...")
    print("[SERVER] Waiting for Mac to send a trajectory...")

    while running:
        try:
            conn, addr = server.accept()
            print(f"[SERVER] Connected from {addr[0]}:{addr[1]}")

            try:
                a_fwd, a_times, b_fwd, b_times = receive_trajectory(conn)
                print(f"[SERVER] Received Payload.")

                # Calculate times and block to execute via precision GPIO
                execute_trajectory(a_fwd, a_times, b_fwd, b_times)

                # Send ACK back to Mac to indicate readiness for the next frame
                ack = json.dumps({"status": "ok"}).encode('utf-8')
                conn.sendall(ack)
                conn.close()
                print("[SERVER] Ready for next trajectory.")

            except Exception as e:
                print(f"[ERROR] Failed to process trajectory: {e}")
                try:
                    err = json.dumps({"status": "error", "message": str(e)}).encode('utf-8')
                    conn.sendall(err)
                except Exception:
                    pass
                conn.close()

        except socket.timeout:
            continue
        except KeyboardInterrupt:
            break

    running = False
    server.close()
    drive.close()
    print("[SHUTDOWN] Server stopped.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        running = False
        print("\n[SHUTDOWN] Interrupted.")
