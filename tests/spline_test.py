# pylint: disable=wildcard-import
# pylint: disable=unused-wildcard-import
"""
Test script for spline-based trajectory.
Opens the interactive spline editor in a browser, receives trajectory via local HTTP,
then executes it on the motor.
"""

import time
import json
import os
import threading
import webbrowser
from http.server import HTTPServer, SimpleHTTPRequestHandler
from tmc_driver.tmc_2209 import *

print("---")
print("SPLINE TEST START")
print("---")

# -----------------------------------------------------------------------
# 1. Launch Spline Editor & Receive Trajectory via Local Server
# -----------------------------------------------------------------------
trajectory = []
server_ready = threading.Event()
trajectory_received = threading.Event()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PORT = 8742

class TrajectoryHandler(SimpleHTTPRequestHandler):
    """Serves spline_editor.html and receives trajectory data via POST."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=SCRIPT_DIR, **kwargs)

    def do_POST(self):
        if self.path == '/trajectory':
            content_length = int(self.headers['Content-Length'])
            body = self.rfile.read(content_length)
            data = json.loads(body.decode('utf-8'))

            trajectory.clear()
            for point in data:
                trajectory.append((point[0], point[1]))

            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "ok", "points": len(trajectory)}).encode())

            print(f"Received {len(trajectory)} trajectory points from spline editor.")
            trajectory_received.set()
        else:
            self.send_response(404)
            self.end_headers()

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def log_message(self, format, *args):
        pass  # Suppress server logs

def run_server():
    server = HTTPServer(('127.0.0.1', PORT), TrajectoryHandler)
    server_ready.set()
    while not trajectory_received.is_set():
        server.handle_request()
    server.server_close()

# Start server in background thread
server_thread = threading.Thread(target=run_server, daemon=True)
server_thread.start()
server_ready.wait()

# Open the spline editor in the default browser
editor_url = f"http://127.0.0.1:{PORT}/spline_editor.html"
print(f"Opening spline editor at {editor_url}")
print("Design your trajectory, then click 'Send to Motor' to begin execution.")
webbrowser.open(editor_url)

# Wait for the user to send the trajectory
trajectory_received.wait()
print(f"Trajectory loaded: {len(trajectory)} points.")

# -----------------------------------------------------------------------
# 2. Initiate the Tmc2209 class
# -----------------------------------------------------------------------
tmc = Tmc2209(
    TmcEnableControlPin(21),
    TmcMotionControlStepDir(16, 20),
    loglevel=Loglevel.INFO,
)

tmc.movement_abs_rel = MovementAbsRel.ABSOLUTE

# Keep acceleration high so the driver strictly follows our mathematical profile
tmc.acceleration_fullstep = 100000
tmc.set_motor_enabled(True)

# -----------------------------------------------------------------------
# 3. Execute the Trajectory
# -----------------------------------------------------------------------
# Set origin/start
tmc.max_speed_fullstep = 500
tmc.run_to_position_steps(0)

print(f"Executing {len(trajectory)} mathematical segments...")
start_time_real = time.time()

for i in range(1, len(trajectory)):
    t_target, pos_target = trajectory[i]
    t_prev, pos_prev = trajectory[i-1]

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

# -----------------------------------------------------------------------
# Clean up
# -----------------------------------------------------------------------
tmc.set_motor_enabled(False)
del tmc

print("---")
print("SPLINE TEST FINISHED")
print("---")
