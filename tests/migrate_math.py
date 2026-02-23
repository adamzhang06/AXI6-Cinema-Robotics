import sys
import re
import os

# 1. Update core/motion/spline.py to house the core generate_times math
spline_path = '/Users/adamzhang/Projects/AXI6 Cinema Robotics/core/motion/spline.py'
with open(spline_path, 'r') as f:
    spline_py = f.read()

generate_times_func = '''
def generate_step_times(spline_data) -> tuple[bool, list[float]]:
    """
    Given an array of absolute positional targets [t, expected_pos_steps],
    convert it into an array of exact times that every single motor step should occur at.
    """
    if len(spline_data) < 2:
        return True, []
        
    times = []
    # Total distance dictates direction 
    total_dp = spline_data[-1][1] - spline_data[0][1]
    direction_is_forward = (total_dp >= 0)
    
    current_step_target = spline_data[0][1]
    step_increment = 1.0 if direction_is_forward else -1.0
    
    # Simple linear interpolation across the spline segments to find exact time for each step boundary.
    for i in range(1, len(spline_data)):
        t_prev, p_prev = spline_data[i-1]
        t_next, p_next = spline_data[i]
        
        # If the segment goes forward and our global direction is forward
        # OR the segment goes backward and our global direction is backward
        if (p_next > p_prev and direction_is_forward) or (p_next < p_prev and not direction_is_forward):
            
            while True:
                next_step = current_step_target + step_increment
                
                # Check if this next discrete integer step falls within the current segment
                if (direction_is_forward and next_step <= p_next) or (not direction_is_forward and next_step >= p_next):
                    # Linear interpolate to find the exact time this step boundary is crossed
                    ratio = (next_step - p_prev) / (p_next - p_prev)
                    exact_t = t_prev + ratio * (t_next - t_prev)
                    times.append(exact_t)
                    current_step_target = next_step
                else:
                    break

    return direction_is_forward, times
'''
if 'def generate_step_times' not in spline_py:
    spline_py += '\n\n' + generate_times_func
    with open(spline_path, 'w') as f:
        f.write(spline_py)

# 2. Update pi/server.py to REMOVE generate_times and simplify execute_trajectory
pi_path = '/Users/adamzhang/Projects/AXI6 Cinema Robotics/pi/server.py'
with open(pi_path, 'r') as f:
    pi_py = f.read()

# Replace receive_trajectory
receive_new = '''
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
'''
pi_py = re.sub(r'def receive_trajectory\(conn\):.*?    return pan_spline, tilt_spline\n', receive_new, pi_py, flags=re.DOTALL)

# Delete generate_times from pi/server.py
pi_py = re.sub(r'def generate_times\(spline_data\) -> tuple\[bool, list\[float\]\]:.*?    return direction_is_forward, times\n\n\n', '', pi_py, flags=re.DOTALL)

# Replace execute_trajectory
execute_new = '''
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
'''
pi_py = re.sub(r'def execute_trajectory\(pan_spline, tilt_spline\):.*?\)\n', execute_new, pi_py, flags=re.DOTALL)

# Update the main loop call
pi_py = pi_py.replace('pan_spline, tilt_spline = receive_trajectory(conn)', 'a_fwd, a_times, b_fwd, b_times = receive_trajectory(conn)')
pi_py = pi_py.replace('print(f"[SERVER] Received Splines: Pan {len(pan_spline)} pts, Tilt {len(tilt_spline)} pts.")', 'print(f"[SERVER] Received Payload.")')
pi_py = pi_py.replace('execute_trajectory(pan_spline, tilt_spline)', 'execute_trajectory(a_fwd, a_times, b_fwd, b_times)')

with open(pi_path, 'w') as f:
    f.write(pi_py)


# 3. Update mac/spline_server.py
mac_server_path = '/Users/adamzhang/Projects/AXI6 Cinema Robotics/mac/spline_server.py'
with open(mac_server_path, 'r') as f:
    mac_server_py = f.read()

if 'from core.motion.spline import' not in mac_server_py:
    imports_add = '''
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from core.motion.spline import generate_step_times
'''
    mac_server_py = mac_server_py.replace('import webbrowser', 'import webbrowser' + imports_add)

mac_post_logic = '''
    def do_POST(self):
        if self.path == '/trajectory':
            content_length = int(self.headers['Content-Length'])
            body = self.rfile.read(content_length)
            trajectory_dict = json.loads(body.decode('utf-8'))
            
            pan_spline = trajectory_dict.get('pan', [])
            tilt_spline = trajectory_dict.get('tilt', [])
            print(f"[HTTP] Received {len(pan_spline)} pan / {len(tilt_spline)} tilt pts from browser.")

            # Differential Math on the Mac -> Motor A and B Arrays
            a_spline = []
            b_spline = []
            for i in range(len(pan_spline)):
                t = pan_spline[i][0]
                pan_p = pan_spline[i][1]
                tilt_p = tilt_spline[i][1]
                a_spline.append((t, pan_p + tilt_p))
                b_spline.append((t, pan_p - tilt_p))
                
            a_fwd, a_times = generate_step_times(a_spline)
            b_fwd, b_times = generate_step_times(b_spline)

            payload_dict = {
                "a_fwd": a_fwd,
                "a_times": a_times,
                "b_fwd": b_fwd,
                "b_times": b_times
            }

            # Forward to Pi
            try:
                ack = send_trajectory_to_pi(payload_dict)
'''
mac_server_py = re.sub(r'    def do_POST\(self\):.*?                ack = send_trajectory_to_pi\(trajectory_dict\)', mac_post_logic, mac_server_py, flags=re.DOTALL)
mac_server_py = mac_server_py.replace('pan_len = len(trajectory_dict.get(\'pan\', []))', '')
mac_server_py = mac_server_py.replace('tilt_len = len(trajectory_dict.get(\'tilt\', []))', '')
mac_server_py = mac_server_py.replace('print(f"[NET] Sent {pan_len} pan points & {tilt_len} tilt points ({len(payload)} bytes) to Pi.")', 'print(f"[NET] Sent highly precise step times ({len(payload)} bytes) to Pi.")')
mac_server_py = mac_server_py.replace('"points": pan_len + tilt_len,', '"points": len(a_times) + len(b_times),')

with open(mac_server_path, 'w') as f:
    f.write(mac_server_py)

# 4. Update mac/tracking/yolo.py
yolo_path = '/Users/adamzhang/Projects/AXI6 Cinema Robotics/mac/tracking/yolo.py'
with open(yolo_path, 'r') as f:
    yolo_py = f.read()

if 'from core.motion.spline import' not in yolo_py:
    yolo_imports = '''
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
from core.motion.spline import generate_step_times
'''
    yolo_py = yolo_py.replace('from ultralytics import YOLO', 'from ultralytics import YOLO' + yolo_imports)

yolo_payload = '''
    s_dp = get_dp("slide") # Slide ignored for now on Pi side, but sent anyway
    p_dp = get_dp("tilt")  # Camera Tilt = Mount Pan
    t_dp = get_dp("pan")   # Camera Pan = Mount Tilt

    # Local differential math to offload the Pi
    # Motor A = Pan + Tilt
    # Motor B = Pan - Tilt
    # Since yolo is a 0.1s delta update relative to 0
    a_spline = [[0.0, 0.0], [dt, p_dp + t_dp]]
    b_spline = [[0.0, 0.0], [dt, p_dp - t_dp]]
    
    a_fwd, a_times = generate_step_times(a_spline)
    b_fwd, b_times = generate_step_times(b_spline)

    payload_dict = {
        "a_fwd": a_fwd,
        "a_times": a_times,
        "b_fwd": b_fwd,
        "b_times": b_times
    }
'''
yolo_py = re.sub(r'    s_dp = get_dp\("slide"\).*?        "tilt": \[\[dt, t_dp\]\]\n    \}', yolo_payload, yolo_py, flags=re.DOTALL)

yolo_shutdown = '''
# Safe Shutdown
stop_payload = {
    "a_fwd": True, "a_times": [],
    "b_fwd": True, "b_times": []
}
'''
yolo_py = re.sub(r'# Safe Shutdown.*?stop_payload = \{.*?\}', yolo_shutdown, yolo_py, flags=re.DOTALL)

with open(yolo_path, 'w') as f:
    f.write(yolo_py)

print("Migration complete. Mac now computes all timings.")
