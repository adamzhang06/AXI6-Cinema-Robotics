"""
Pi-side TCP server for receiving spline trajectories from the Mac and executing them.
Broadcasts a UDP beacon so the Mac can auto-discover this Pi.

Usage: python socket/pi_spline_motor.py
"""

import socket
import json
import struct
import time
import threading
from tmc_driver import Tmc2209, Loglevel, MovementAbsRel, TmcEnableControlPin, TmcMotionControlStepDir

# --- NETWORK PORTS ---
TCP_PORT = 5010       # Trajectory data in
BEACON_PORT = 5011    # Auto-discovery beacon out

# --- STATE ---
running = True


# -----------------------------------------------------------------------
# 1. Beacon Broadcaster (so Mac can find us)
# -----------------------------------------------------------------------
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


# -----------------------------------------------------------------------
# 2. Receive trajectory over TCP (length-prefixed JSON)
# -----------------------------------------------------------------------
def receive_trajectory(conn):
    """Reads a length-prefixed JSON trajectory from a TCP connection."""
    # Read 4-byte big-endian length header
    length_bytes = b''
    while len(length_bytes) < 4:
        chunk = conn.recv(4 - len(length_bytes))
        if not chunk:
            raise ConnectionError("Connection closed while reading length header")
        length_bytes += chunk

    payload_length = struct.unpack('>I', length_bytes)[0]
    print(f"[NET] Expecting {payload_length} bytes of trajectory data...")

    # Read the full JSON payload
    payload = b''
    while len(payload) < payload_length:
        chunk = conn.recv(min(4096, payload_length - len(payload)))
        if not chunk:
            raise ConnectionError("Connection closed while reading payload")
        payload += chunk

    trajectory = json.loads(payload.decode('utf-8'))
    return trajectory


# -----------------------------------------------------------------------
# 3. Execute trajectory on motor
# -----------------------------------------------------------------------
def execute_trajectory(trajectory):
    """Initializes the TMC2209 and runs through the trajectory array."""
    print(f"[MOTOR] Initializing TMC2209...")

    tmc = Tmc2209(
        TmcEnableControlPin(21),
        TmcMotionControlStepDir(16, 20),
        loglevel=Loglevel.INFO,
    )

    tmc.movement_abs_rel = MovementAbsRel.ABSOLUTE
    tmc.acceleration_fullstep = 100000
    tmc.set_motor_enabled(True)

    # Set origin
    tmc.max_speed_fullstep = 500
    tmc.run_to_position_steps(0)

    print(f"[MOTOR] Executing {len(trajectory)} segments...")
    start_time_real = time.time()

    for i in range(1, len(trajectory)):
        t_target, pos_target = trajectory[i]
        t_prev, pos_prev = trajectory[i - 1]

        dt_seg = t_target - t_prev
        dp_seg = pos_target - pos_prev

        required_speed = abs(dp_seg) / dt_seg if dt_seg > 0 else 0

        # Timing sync to prevent drift
        expected_start = start_time_real + t_prev
        now = time.time()
        if expected_start > now:
            time.sleep(expected_start - now)

        if required_speed > 0:
            tmc.max_speed_fullstep = required_speed
            tmc.run_to_position_steps(pos_target)

    actual_time = time.time() - start_time_real
    print(f"[MOTOR] Done. Actual execution time: {actual_time:.3f}s")

    tmc.set_motor_enabled(False)
    del tmc


# -----------------------------------------------------------------------
# 4. Main TCP Server Loop
# -----------------------------------------------------------------------
def main():
    global running

    # Start beacon in background
    beacon_thread = threading.Thread(target=beacon_broadcaster, daemon=True)
    beacon_thread.start()

    # Start TCP server
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', TCP_PORT))
    server.listen(1)
    server.settimeout(1.0)

    print(f"[SERVER] Listening for trajectory on TCP port {TCP_PORT}...")
    print("[SERVER] Waiting for Mac to send a trajectory...")

    while running:
        try:
            conn, addr = server.accept()
            print(f"[SERVER] Connected from {addr[0]}:{addr[1]}")

            try:
                trajectory = receive_trajectory(conn)
                num_points = len(trajectory)
                print(f"[SERVER] Received {num_points} trajectory points.")

                # Send ACK back to Mac
                ack = json.dumps({"status": "ok", "points": num_points}).encode('utf-8')
                conn.sendall(ack)
                conn.close()

                # Execute the trajectory
                execute_trajectory(trajectory)
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
    print("[SHUTDOWN] Server stopped.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        running = False
        print("\n[SHUTDOWN] Interrupted.")
