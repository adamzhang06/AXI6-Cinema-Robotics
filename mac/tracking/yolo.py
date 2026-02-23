import cv2
import time
import os
import threading
import math
import socket
import json
import struct
import numpy as np
from ultralytics import YOLO
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
from core.motion.spline import generate_step_times



# --- NETWORK SETUP ---
PI_IP = "100.69.176.89"  
PI_PORT = 5010 # Changed to TCP port

# Global state for network thread
net_thread = None

def send_trajectory_to_pi(trajectory_dict):
    """Connects to the Pi and sends the trajectory dict as length-prefixed JSON."""
    global net_thread
    
    # If the thread is still alive, the Pi is busy executing the previous trajectory
    # We simply drop the frame and wait for the Pi to catch up
    if net_thread is not None and net_thread.is_alive():
        return

    def worker():
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2.0) # Larger timeout for execution wait
            
            # Disable Nagle's algorithm for fast transmission
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            
            sock.connect((PI_IP, PI_PORT))
            
            payload = json.dumps(trajectory_dict).encode('utf-8')
            header = struct.pack('>I', len(payload))
            sock.sendall(header + payload)
            
            # Wait for ACK. Since server.py sends ACK *after* execution completes,
            # this correctly ties up the thread until the Pi is ready for a new array!
            _ = sock.recv(1024)
            
            sock.close()
        except Exception:
            pass # Ignore network errors silently to avoid console spam

    net_thread = threading.Thread(target=worker, daemon=True)
    net_thread.start()


# --- CONFIGURABLE BASE SETTINGS ---
WIDTH, HEIGHT = 1920, 1080
BASE_DZ_SLIDE = 60
BASE_DZ_PAN = 150
BASE_DZ_TILT = 40 # Tighter deadzone since faces are smaller targets

# The dynamic deadzone factor (0.5 = shrinks to 50% of its size when recentering)
RECENTER_RATIO = 0.5

SMOOTHING_FACTOR = 0.25
MAX_TRACK_DIST = 250 
GLOBAL_SPEED_SCALE = 0.1

# --- 1. SHARED SYSTEM STATE ---
system_state = {
    "running": True,
    "slide": {"speed": 0, "dir": 0, "locked": True,  "recentering": False},
    "pan":   {"speed": 0, "dir": 0, "locked": True,  "recentering": False},
    "tilt":  {"speed": 0, "dir": 0, "locked": True,  "recentering": False}
}

# --- 2. VISION THREADS (YOLOv8 Dual Models) ---
latest_body_boxes = []
latest_face_boxes = []
frame_for_body = None
frame_for_face = None

def body_vision_worker():
    global latest_body_boxes, frame_for_body
    print("[VISION] Loading Body YOLO Model...")
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(script_dir, '.', 'models', 'yolov8n_float16.tflite')
    body_model = YOLO(model_path) 
    print("[VISION] Body Model Loaded. Thread Active.")
    
    while system_state["running"]:
        if frame_for_body is not None:
            # Predict 'person' class only (0)
            results = body_model.predict(frame_for_body, classes=[0], verbose=False)
            if len(results) > 0:
                latest_body_boxes = results[0].boxes.xyxy.cpu().numpy()
            else:
                latest_body_boxes = []
            frame_for_body = None 
        else:
            time.sleep(0.01)
    print("[VISION] Body Thread Stopped.")

def face_vision_worker():
    global latest_face_boxes, frame_for_face
    print("[VISION] Loading Face YOLO Model...")
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(script_dir, '.', 'models', 'face_yolov8n_float16.tflite')
    face_model = YOLO(model_path) 
    print("[VISION] Face Model Loaded. Thread Active.")
    
    while system_state["running"]:
        if frame_for_face is not None:
            # Face model usually has face mapped to class 0 (or is the only class)
            results = face_model.predict(frame_for_face, verbose=False)
            if len(results) > 0:
                latest_face_boxes = results[0].boxes.xyxy.cpu().numpy()
            else:
                latest_face_boxes = []
            frame_for_face = None 
        else:
            time.sleep(0.01)
    print("[VISION] Face Thread Stopped.")


# --- 3. PID & EMA CLASSES ---
class StepperPID:
    def __init__(self, kp, ki, kd, max_speed):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_speed = max_speed
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()

    def compute(self, error):
        current_time = time.time()
        dt = max(current_time - self.last_time, 0.001)
        self.last_time = current_time
        
        self.integral += error * dt
        self.integral = max(min(self.integral, 1000), -1000) 
        
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        raw_velocity = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        # Non-linear curve: apply an exponent to make small errors move very slowly
        # while keeping large errors fast.
        sign = 1 if raw_velocity >= 0 else -1
        # The 0.05 factor keeps the math manageable before exponentiating
        curved_velocity = sign * (abs(raw_velocity * 0.05) ** 1.5)
        
        scaled_velocity = abs(curved_velocity) * GLOBAL_SPEED_SCALE
        
        speed = min(scaled_velocity, self.max_speed)
        direction = 10 if error > 0 else -10 
        
        return float(speed), int(direction)
        
    def reset(self):
        self.integral = 0
        self.prev_error = 0

class EMASmoother:
    def __init__(self, alpha):
        self.alpha, self.value = alpha, None
    def update(self, new_value):
        if self.value is None: self.value = new_value
        else: self.value += self.alpha * (new_value - self.value)
        return self.value

# --- 4. START BACKGROUND THREADS ---
body_thread = threading.Thread(target=body_vision_worker, daemon=True)
face_thread = threading.Thread(target=face_vision_worker, daemon=True)
body_thread.start()
face_thread.start()

# --- 5. SETUP UTILS & PID ---
# max_speed here represents maximum steps per second (Hz). 800 Hz = 4 revs/sec at 200 steps/rev
slide_pid = StepperPID(kp=0.5, ki=0.01, kd=3.5, max_speed=1200)
pan_pid   = StepperPID(kp=1.75, ki=0.01, kd=0.001, max_speed=1200)
tilt_pid  = StepperPID(kp=1.85, ki=0.01, kd=0.0008, max_speed=1200)

smoothers_body = {k: EMASmoother(SMOOTHING_FACTOR) for k in ["left", "right", "top", "bottom"]}
smoothers_face = {k: EMASmoother(SMOOTHING_FACTOR) for k in ["left", "right", "top", "bottom"]}

def draw_outlined_text(img, text, pos, font_scale, color, thickness):
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), thickness + 3)
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, thickness)

def draw_crosshair(img, cx, cy, color, size=8, thickness=1):
    cv2.line(img, (int(cx) - size, int(cy)), (int(cx) + size, int(cy)), color, thickness)
    cv2.line(img, (int(cx), int(cy) - size), (int(cx), int(cy) + size), color, thickness)

def center_of_box(box):
    return (box[0] + box[2]) / 2, (box[1] + box[3]) / 2

# --- 6. MAIN VISION LOOP ---
target_person_index = 0
dz_scale = 1.0  
STANDBY_COLOR = (255, 200, 50) 
RECENTER_COLOR = (0, 0, 255) # Red for active snapping
FACE_COLOR = (200, 50, 255)

is_frozen = False
freeze_start_time = 0
flicker_grace_sec = 0.5 
target_lost = True

force_target_switch = True
last_target_cx, last_target_cy = 0, 0

cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) 
cap.set(cv2.CAP_PROP_EXPOSURE, 300)

print("\n--- AXI6 MAC VISION SYSTEM ONLINE ---")

while cap.isOpened() and system_state["running"]:
    success, frame = cap.read()
    if not success: break
    frame = cv2.flip(frame, 1)
    current_time = time.time()

    if frame_for_body is None:
        frame_for_body = frame.copy()
    if frame_for_face is None:
        frame_for_face = frame.copy()

    frame_h, frame_w, _ = frame.shape
    frame_center_x, frame_center_y = int(frame_w / 2), int(frame_h / 2)
    
    # Calculate base deadzones for this frame
    base_dz_slide = int(BASE_DZ_SLIDE * dz_scale)
    base_dz_pan = int(BASE_DZ_PAN * dz_scale)
    base_dz_tilt = int(BASE_DZ_TILT * dz_scale)
    
    system_engaged = not all(system_state[a]["locked"] for a in ["slide", "pan", "tilt"])

    current_body_boxes = list(latest_body_boxes)
    current_face_boxes = list(latest_face_boxes)
    current_num_people = len(current_body_boxes)
    
    if current_num_people > 0:
        current_body_boxes.sort(key=lambda b: b[0])
        target_person_index = target_person_index % current_num_people

    target_body_found = False
    best_body_box = None
    best_face_box = None

    # --- TRACKING LOGIC ---
    if current_num_people > 0:
        if force_target_switch:
            best_body_box = current_body_boxes[target_person_index]
            last_target_cx, last_target_cy = center_of_box(best_body_box)
            force_target_switch = False
            target_body_found = True
        else:
            min_dist = float('inf')
            for box in current_body_boxes:
                cx, cy = center_of_box(box)
                dist = math.hypot(cx - last_target_cx, cy - last_target_cy)
                if dist < min_dist:
                    min_dist = dist
                    best_body_box = box
            
            if best_body_box is not None and min_dist < MAX_TRACK_DIST:
                target_body_found = True
                last_target_cx, last_target_cy = center_of_box(best_body_box)
                target_person_index = next(i for i, b in enumerate(current_body_boxes) if np.array_equal(b, best_body_box))

    # If we found a body, find its corresponding face
    if target_body_found:
        body_x1, body_y1, body_x2, body_y2 = best_body_box[:4]
        
        # Find the face that is mostly inside the target body box
        for f_box in current_face_boxes:
            fx1, fy1, fx2, fy2 = f_box[:4]
            fcx, fcy = center_of_box(f_box)
            
            # Simple check: is face center inside the upper half of the body box?
            upper_body_y = body_y1 + ((body_y2 - body_y1) * 0.6)
            if body_x1 <= fcx <= body_x2 and body_y1 <= fcy <= upper_body_y:
                best_face_box = f_box
                break


    # --- THE STATE MACHINE ---
    if target_body_found:
        is_frozen, target_lost = False, False
        mode_text = f"Composite Tracking (Person {target_person_index + 1}/{current_num_people})"
        b_raw_left, b_raw_top, b_raw_right, b_raw_bottom = best_body_box[:4]
        
        if best_face_box is not None:
            f_raw_left, f_raw_top, f_raw_right, f_raw_bottom = best_face_box[:4]
        else:
            # Fallback to approximating face if we lose the face model temporarily
            approx_fw = (b_raw_right - b_raw_left) * 0.2
            approx_cx = (b_raw_left + b_raw_right) / 2
            f_raw_top = b_raw_top
            f_raw_bottom = b_raw_top + approx_fw
            f_raw_left = approx_cx - (approx_fw / 2)
            f_raw_right = approx_cx + (approx_fw / 2)
    else:
        if not is_frozen and not target_lost:
            is_frozen, freeze_start_time = True, current_time

        if is_frozen:
            time_left = max(0.0, flicker_grace_sec - (current_time - freeze_start_time))
            if time_left > 0:
                mode_text = f"LOSS GRACE HOLD ({time_left:.1f}s)"
            else:
                is_frozen = False
                if current_num_people == 0:
                    target_lost = True

    # Draw all bodies
    for box in current_body_boxes:
        x1, y1, x2, y2 = int(box[0]), int(box[1]), int(box[2]), int(box[3])
        cx, cy = center_of_box(box)
        is_target = target_body_found and best_body_box is not None and np.array_equal(box, best_body_box)
        box_color = STANDBY_COLOR if is_target else (0, 0, 255)
        cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)
        draw_crosshair(frame, cx, cy, box_color, size=8, thickness=1)
        
    # Draw all faces
    for box in current_face_boxes:
        x1, y1, x2, y2 = int(box[0]), int(box[1]), int(box[2]), int(box[3])
        is_target_face = target_body_found and best_face_box is not None and np.array_equal(box, best_face_box)
        box_color = FACE_COLOR if is_target_face else (100, 100, 100)
        cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)

    # Variables for UI drawing so they sync perfectly with the logic
    active_dz_s = base_dz_slide
    active_dz_p = base_dz_pan
    active_dz_t = base_dz_tilt

    # --- DYNAMIC DEADZONE & MOTOR LOGIC ---
    if target_lost:
        for axis in ["slide", "pan", "tilt"]: 
            system_state[axis]["speed"] = 0
            system_state[axis]["dir"] = 0
            system_state[axis]["recentering"] = False
            
        slide_pid.reset(); pan_pid.reset(); tilt_pid.reset()
        actively_sending_input = False
        mode_text = "TARGET LOST"
    else:
        if not is_frozen:
            smoothers_body["left"].update(b_raw_left)
            smoothers_body["right"].update(b_raw_right)
            smoothers_body["top"].update(b_raw_top)
            smoothers_body["bottom"].update(b_raw_bottom)
            
            smoothers_face["left"].update(f_raw_left)
            smoothers_face["right"].update(f_raw_right)
            smoothers_face["top"].update(f_raw_top)
            smoothers_face["bottom"].update(f_raw_bottom)

        # Slide & Pan track BODY
        b_s_left, b_s_right = smoothers_body["left"].value, smoothers_body["right"].value
        b_s_top, b_s_bottom = smoothers_body["top"].value, smoothers_body["bottom"].value
        body_cx, body_cy = (b_s_left + b_s_right) / 2, (b_s_top + b_s_bottom) / 2
        body_offset_x = body_cx - frame_center_x
        
        # Tilt tracks FACE
        f_s_left, f_s_right = smoothers_face["left"].value, smoothers_face["right"].value
        f_s_top, f_s_bottom = smoothers_face["top"].value, smoothers_face["bottom"].value
        face_cx, face_cy = (f_s_left + f_s_right) / 2, (f_s_top + f_s_bottom) / 2
        face_offset_y = face_cy - frame_center_y
        
        # --- SLIDE (X-Axis, Body) ---
        if not system_state["slide"]["locked"]:
            if not system_state["slide"]["recentering"] and abs(body_offset_x) > base_dz_slide:
                system_state["slide"]["recentering"] = True
            
            active_dz_s = int(base_dz_slide * RECENTER_RATIO) if system_state["slide"]["recentering"] else base_dz_slide
            
            if system_state["slide"]["recentering"] and abs(body_offset_x) <= active_dz_s:
                system_state["slide"]["recentering"] = False
                active_dz_s = base_dz_slide

            if abs(body_offset_x) > active_dz_s:
                system_state["slide"]["speed"], system_state["slide"]["dir"] = slide_pid.compute(body_offset_x)
            else: 
                system_state["slide"]["speed"], system_state["slide"]["dir"] = 0, 0
                slide_pid.reset()

        # --- PAN (X-Axis, Body) ---
        if not system_state["pan"]["locked"]:
            if not system_state["pan"]["recentering"] and abs(body_offset_x) > base_dz_pan:
                system_state["pan"]["recentering"] = True
            
            active_dz_p = int(base_dz_pan * RECENTER_RATIO) if system_state["pan"]["recentering"] else base_dz_pan
            
            if system_state["pan"]["recentering"] and abs(body_offset_x) <= active_dz_p:
                system_state["pan"]["recentering"] = False
                active_dz_p = base_dz_pan

            if abs(body_offset_x) > active_dz_p:
                system_state["pan"]["speed"], system_state["pan"]["dir"] = pan_pid.compute(body_offset_x)
            else: 
                system_state["pan"]["speed"], system_state["pan"]["dir"] = 0, 0
                pan_pid.reset()

        # --- TILT (Y-Axis, Face) ---
        if not system_state["tilt"]["locked"]:
            if not system_state["tilt"]["recentering"] and abs(face_offset_y) > base_dz_tilt:
                system_state["tilt"]["recentering"] = True
            
            active_dz_t = int(base_dz_tilt * RECENTER_RATIO) if system_state["tilt"]["recentering"] else base_dz_tilt
            
            if system_state["tilt"]["recentering"] and abs(face_offset_y) <= active_dz_t:
                system_state["tilt"]["recentering"] = False
                active_dz_t = base_dz_tilt

            if abs(face_offset_y) > active_dz_t:
                system_state["tilt"]["speed"], system_state["tilt"]["dir"] = tilt_pid.compute(face_offset_y)
            else: 
                system_state["tilt"]["speed"], system_state["tilt"]["dir"] = 0, 0
                tilt_pid.reset()

        actively_sending_input = any(not system_state[a]["locked"] and system_state[a]["speed"] > 0 for a in ["slide", "pan", "tilt"])
        target_color = (0, 255, 0) if actively_sending_input else STANDBY_COLOR

        # Draw Smoothed Target Boxes
        cv2.rectangle(frame, (int(b_s_left), int(b_s_top)), (int(b_s_right), int(b_s_bottom)), target_color, 2)
        cv2.rectangle(frame, (int(f_s_left), int(f_s_top)), (int(f_s_right), int(f_s_bottom)), FACE_COLOR, 2)
        draw_crosshair(frame, body_cx, body_cy, target_color, size=8, thickness=2)
        draw_crosshair(frame, face_cx, face_cy, FACE_COLOR, size=8, thickness=2)


    # --- TRANSMIT DATA TO PI ---
    # Convert speeds to purely relative changes (dp) over a fixed dt interval (e.g., 0.1s frame assuming 10hz update rate)
    dt = 0.1 
    
    # We swap pan/tilt at network level like before
    # old: "pan":   {"speed": system_state["tilt"]["speed"],  "dir": system_state["tilt"]["dir"]}
    # Hz = speed * sign(dir)
    # dp = Hz * dt
    
    def get_dp(axis_name):
        speed = system_state[axis_name]["speed"]
        direction = system_state[axis_name]["dir"]
        hz = speed if direction > 0 else -speed if direction < 0 else 0
        return hz * dt
        

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

    
    # Only transmit if actively engaged (save TCP overhead when locked)
    if actively_sending_input:
        send_trajectory_to_pi(payload_dict)


    # --- DRAW ACTIVE DYNAMIC DEADZONE BOUNDARIES ---
    if not system_state["slide"]["locked"]:
        c_slide = RECENTER_COLOR if system_state["slide"]["recentering"] else STANDBY_COLOR
        xl, xr = int(frame_center_x - active_dz_s), int(frame_center_x + active_dz_s)
        cv2.line(frame, (xl, 0), (xl, frame_h), c_slide, 1)
        cv2.line(frame, (xr, 0), (xr, frame_h), c_slide, 1)

    if not system_state["pan"]["locked"]:
        c_pan = RECENTER_COLOR if system_state["pan"]["recentering"] else STANDBY_COLOR
        xl, xr = int(frame_center_x - active_dz_p), int(frame_center_x + active_dz_p)
        yt, yb = frame_center_y - 120, frame_center_y + 120
        cv2.line(frame, (xl, yt), (xl, yb), c_pan, 2)
        cv2.line(frame, (xr, yt), (xr, yb), c_pan, 2)
        if xr < frame_w + 50: cv2.putText(frame, "PAN DZ", (xr + 5, yt - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, c_pan, 1)

    if not system_state["tilt"]["locked"]:
        c_tilt = RECENTER_COLOR if system_state["tilt"]["recentering"] else FACE_COLOR
        yt, yb = int(frame_center_y - active_dz_t), int(frame_center_y + active_dz_t)
        xl, xr = frame_center_x - 150, frame_center_x + 150
        cv2.line(frame, (xl, yt), (xr, yt), c_tilt, 2)
        cv2.line(frame, (xl, yb), (xr, yb), c_tilt, 2)

    # --- DRAW HUD ---
    mode_color = (0, 165, 255) if is_frozen else (0, 0, 255) if target_lost else (255, 255, 0)
    draw_outlined_text(frame, f"MODE: {mode_text}", (10, 35), 0.8, mode_color, 2)
    
    for i, axis in enumerate(["slide", "pan", "tilt"]):
        is_lck = system_state[axis]['locked']
        col = (0, 0, 255) if is_lck else (0, 255, 0)
        stat = "LOCKED" if is_lck else f"SPD: {system_state[axis]['speed']} | DIR: {'+' if system_state[axis]['dir']>0 else '-'}"
        draw_outlined_text(frame, f"{axis.capitalize()}: {stat}", (10, 75 + (i*35)), 0.7, col, 2)
    
    eng_txt = "ENGAGED" if system_engaged else "NOT ENGAGED"
    eng_col = (0, 255, 0) if system_engaged else (0, 0, 255)
    (ew, eh), _ = cv2.getTextSize(eng_txt, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 3)
    draw_outlined_text(frame, eng_txt, (frame_w - ew - 20, 40), 1.0, eng_col, 3)

    draw_crosshair(frame, frame_center_x, frame_center_y, (200, 200, 200), size=12, thickness=1)
    
    view_frame = cv2.resize(frame, (1280, 720)) if frame_w == 1920 else frame
    cv2.imshow('AXI6 Master Brain', view_frame)
    
    # --- KEYBOARD CONTROLS ---
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'): system_state["running"] = False; break
    elif key == ord('s'): system_state["slide"]["locked"] = not system_state["slide"]["locked"]
    elif key == ord('p'): system_state["pan"]["locked"] = not system_state["pan"]["locked"]
    elif key == ord('t'): system_state["tilt"]["locked"] = not system_state["tilt"]["locked"]
    elif key == ord(' '): system_state["slide"]["locked"] = system_state["pan"]["locked"] = system_state["tilt"]["locked"] = True
        
    elif key == ord('1'): 
        force_target_switch, target_lost = True, False
    elif key == ord('2'): 
        target_person_index = target_person_index + 1
        force_target_switch, target_lost = True, False
    
    elif key == ord('i'): dz_scale = max(1.0, dz_scale - 0.5) 
    elif key == ord('o'): dz_scale = min(8.0, dz_scale + 0.5) 
    elif key == ord('['): flicker_grace_sec = max(0.0, flicker_grace_sec - 0.1) 
    elif key == ord(']'): flicker_grace_sec = min(5.0, flicker_grace_sec + 0.1) 



# Safe Shutdown
stop_payload = {
    "a_fwd": True, "a_times": [],
    "b_fwd": True, "b_times": []
}

send_trajectory_to_pi(stop_payload)


cap.release()
cv2.destroyAllWindows()
body_thread.join()
face_thread.join()
print("System Successfully Offline.")