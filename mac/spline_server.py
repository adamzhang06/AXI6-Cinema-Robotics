"""
mac/spline_server.py
--------------------
Mac-side server for the spline trajectory editor.
Serves spline_editor.html locally, receives trajectory via HTTP POST,
auto-discovers the Pi, and forwards the trajectory arrays via TCP.

Reference: legacy/socket/mac_spline_server.py, legacy/tests/spline_test.py
"""

import socket
import json
import struct
import os
import threading
import webbrowser
from http.server import HTTPServer, SimpleHTTPRequestHandler

# --- NETWORK CONFIG ---
HTTP_PORT = 8742       # Local web server for spline editor
BEACON_PORT = 5011     # Pi beacon listen port
PI_TCP_PORT = 5010     # Pi trajectory receiver port

# Path to the ui directory where spline_editor.html lives
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
UI_DIR = os.path.join(SCRIPT_DIR, 'ui')

# --- STATE ---
pi_ip = None
pi_port = PI_TCP_PORT


# -----------------------------------------------------------------------
# 1. Auto-Discover Pi via Beacon
# -----------------------------------------------------------------------
def discover_pi(timeout=10):
    """Listens for the Pi's UDP beacon broadcast to find its IP."""
    global pi_ip, pi_port
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # macOS needs SO_REUSEPORT for broadcast binding
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    except AttributeError:
        pass

    sock.bind(('', BEACON_PORT))
    sock.settimeout(timeout)

    print(f"[DISCOVER] Listening for Pi beacon on UDP port {BEACON_PORT}...")

    try:
        while True:
            data, addr = sock.recvfrom(1024)
            try:
                beacon = json.loads(data.decode('utf-8'))
                if beacon.get("service") == "axi6_spline_motor":
                    pi_ip = addr[0]
                    pi_port = beacon.get("port", PI_TCP_PORT)
                    print(f"[DISCOVER] Found Pi at {pi_ip}:{pi_port}")
                    sock.close()
                    return True
            except (json.JSONDecodeError, KeyError):
                continue
    except socket.timeout:
        print("[DISCOVER] No Pi found within timeout. Will retry when sending trajectory.")
        sock.close()
        return False


# -----------------------------------------------------------------------
# 2. Send Trajectory to Pi via TCP
# -----------------------------------------------------------------------
def send_trajectory_to_pi(trajectory_dict):
    """Connects to the Pi and sends the trajectory dict as length-prefixed JSON."""
    global pi_ip, pi_port

    # Try discovery if we haven't found the Pi yet
    if pi_ip is None:
        print("[NET] Pi not discovered yet, attempting discovery...")
        if not discover_pi(timeout=5):
            raise ConnectionError("Could not find Pi on the network. Is pi/server.py running?")

    print(f"[NET] Connecting to Pi at {pi_ip}:{pi_port}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(10)
    sock.connect((pi_ip, pi_port))

    # Encode and send length-prefixed payload
    payload = json.dumps(trajectory_dict).encode('utf-8')
    header = struct.pack('>I', len(payload))
    sock.sendall(header + payload)

    pan_len = len(trajectory_dict.get('pan', []))
    tilt_len = len(trajectory_dict.get('tilt', []))
    print(f"[NET] Sent {pan_len} pan points & {tilt_len} tilt points ({len(payload)} bytes) to Pi.")

    # Wait for ACK
    ack_data = sock.recv(4096)
    sock.close()

    ack = json.loads(ack_data.decode('utf-8'))
    print(f"[NET] Pi acknowledged: {ack}")
    return ack


# -----------------------------------------------------------------------
# 3. HTTP Server — serves spline_editor.html & handles trajectory POST
# -----------------------------------------------------------------------
class SplineHTTPHandler(SimpleHTTPRequestHandler):
    """Serves the spline editor UI and forwards trajectory to Pi."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=UI_DIR, **kwargs)

    def do_POST(self):
        if self.path == '/trajectory':
            content_length = int(self.headers['Content-Length'])
            body = self.rfile.read(content_length)
            trajectory_dict = json.loads(body.decode('utf-8'))
            
            pan_len = len(trajectory_dict.get('pan', []))
            tilt_len = len(trajectory_dict.get('tilt', []))
            print(f"[HTTP] Received {pan_len} pan / {tilt_len} tilt pts from browser.")

            # Forward to Pi
            try:
                ack = send_trajectory_to_pi(trajectory_dict)
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    "status": "ok",
                    "points": pan_len + tilt_len,
                    "message": f"Sent to Pi at {pi_ip}"
                }).encode())
            except Exception as e:
                print(f"[ERROR] Failed to send to Pi: {e}")
                self.send_response(502)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    "status": "error",
                    "message": str(e)
                }).encode())
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
        pass  # Suppress default HTTP logs; we print our own messages


# -----------------------------------------------------------------------
# 4. Main
# -----------------------------------------------------------------------
def main():
    # Start Pi discovery in background (non-blocking)
    discovery_thread = threading.Thread(target=discover_pi, args=(10,), daemon=True)
    discovery_thread.start()

    # Start HTTP server
    server = HTTPServer(('0.0.0.0', HTTP_PORT), SplineHTTPHandler)
    editor_url = f"http://127.0.0.1:{HTTP_PORT}/spline_editor.html"

    print(f"[HTTP] Serving spline editor at {editor_url}")
    print("[HTTP] Design your trajectory, then click 'Send to Motor'.")
    webbrowser.open(editor_url)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[SHUTDOWN] Mac server stopped.")
        server.server_close()


if __name__ == "__main__":
    main()
