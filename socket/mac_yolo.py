import cv2
import time
import os
import threading
import math
import socket
import json
import numpy as np
from ultralytics import YOLO

# --- NETWORK SETUP ---
PI_IP = "100.69.176.89"  
PI_PORT = 5005
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# --- CONFIGURABLE BASE SETTINGS ---
WIDTH, HEIGHT = 1920, 1080
BASE_DZ_SLIDE = 60
BASE_DZ_PAN = 150
BASE_DZ_TILT = 40

RECENTER_RATIO = 0.5
SMOOTHING_FACTOR = 0.25 
MAX_TRACK_DIST = 250 
GLOBAL_SPEED_SCALE = 1.0

# --- 1. SHARED SYSTEM STATE ---
system_state = {
    "running": True,
    "slide": {"speed": 0, "dir": 0, "locked": True,  "recentering": False},
    "pan":   {"speed": 0, "dir": 0, "locked": False, "recentering": False},
    "tilt":  {"speed": 0, "dir": 0, "locked": True,  "recentering": False}
}

# --- 2. VISION THREAD (YOLOv8) ---
latest_boxes = []
frame_for_yolo = None

def vision_worker():
    global latest_boxes, frame_for_yolo
    print("[VISION] Loading YOLO Model...")
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(script_dir, '..', 'models', 'yolov8n_float16.tflite')
    
    model = YOLO(model_path) 
    print("[VISION] Model Loaded. Thread Active.")
    
    while system_state["running"]:
        if frame_for_yolo is not None:
            results = model.predict(frame_for_yolo, classes=[0], verbose=False)
            if len(results) > 0:
                latest_boxes = results[0].boxes.xyxy.cpu().numpy()
            else:
                latest_boxes = []
            frame_for_yolo = None 
        else:
            time.sleep(0.01)
    print("[VISION] Thread Stopped.")

# --- 3. PID & EMA CLASSES ---
class StepperPID:
    # ADDED 'accel' PARAMETER HERE
    def __init__(self, kp, ki, kd, max_speed, accel):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_speed = max_speed
        self.accel = accel 
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
        scaled_velocity = abs(raw_velocity) * GLOBAL_SPEED_SCALE
        
        speed = min(int(scaled_velocity), self.max_speed)
        
        # --- REVERTED THIS LINE: Back to 10-step chunks ---
        direction = 1 if error > 0 else -1
        
        return speed, direction
        
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
vision_thread = threading.Thread(target=vision_worker, daemon=True)
vision_thread.start()

# --- 5. SETUP UTILS & PID ---
# Dropped max_speed to 300 to stop it from violently whipping around
slide_pid = StepperPID(kp=0.075, ki=0.01, kd=3.5, max_speed=300, accel=400)
pan_pid   = StepperPID(kp=0.075, ki=0.01, kd=3.5, max_speed=300, accel=2000) # 200 accel makes it start slow
tilt_pid  = StepperPID(kp=0.075, ki=0.01, kd=3.5, max_speed=300, accel=400)

smoothers = {k: EMASmoother(SMOOTHING_FACTOR) for k in ["left", "right", "top", "bottom"]}

def draw_outlined_text(img, text, pos, font_scale, color, thickness):
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), thickness + 3)
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, thickness)

def draw_crosshair(img, cx, cy, color, size=8, thickness=1):
    cv2.line(img, (int(cx) - size, int(cy)), (int(cx) + size, int(cy)), color, thickness)
    cv2.line(img, (int(cx), int(cy) - size), (int(cx), int(cy) + size), color, thickness)

# --- 6. MAIN VISION LOOP ---
tracking_mode = "single"
target_person_index = 0
dz_scale = 1.0  
STANDBY_COLOR = (255, 200, 50) 
RECENTER_COLOR = (0, 0, 255) 

expected_num_people = 0
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

    if frame_for_yolo is None:
        frame_for_yolo = frame.copy()

    frame_h, frame_w, _ = frame.shape
    frame_center_x, frame_center_y = int(frame_w / 2), int(frame_h / 2)
    
    base_dz_slide = int(BASE_DZ_SLIDE * dz_scale)
    base_dz_pan = int(BASE_DZ_PAN * dz_scale)
    base_dz_tilt = int(BASE_DZ_TILT * dz_scale)
    
    system_engaged = not all(system_state[a]["locked"] for a in ["slide", "pan", "tilt"])

    current_boxes = list(latest_boxes)
    current_num_people = len(current_boxes)
    
    if current_num_people > 0:
        current_boxes.sort(key=lambda b: b[0])
        target_person_index = target_person_index % current_num_people

    target_found_this_frame = False
    best_box = None

    # --- TRACKING LOGIC ---
    if tracking_mode == "single" and current_num_people > 0:
        if force_target_switch:
            best_box = current_boxes[target_person_index]
            last_target_cx = (best_box[0] + best_box[2]) / 2
            last_target_cy = (best_box[1] + best_box[3]) / 2
            force_target_switch = False
            target_found_this_frame = True
        else:
            min_dist = float('inf')
            for box in current_boxes:
                cx, cy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
                dist = math.hypot(cx - last_target_cx, cy - last_target_cy)
                if dist < min_dist:
                    min_dist = dist
                    best_box = box
            
            if best_box is not None and min_dist < MAX_TRACK_DIST:
                target_found_this_frame = True
                last_target_cx = (best_box[0] + best_box[2]) / 2
                last_target_cy = (best_box[1] + best_box[3]) / 2
                target_person_index = next(i for i, b in enumerate(current_boxes) if np.array_equal(b, best_box))

    elif tracking_mode == "group":
        if current_num_people >= expected_num_people and current_num_people > 0:
            expected_num_people = current_num_people
            target_found_this_frame = True

    # --- THE STATE MACHINE ---
    if target_found_this_frame:
        is_frozen, target_lost = False, False
        if tracking_mode == "single":
            mode_text = f"Single Tracking (Person {target_person_index + 1}/{current_num_people})"
            raw_left, raw_top, raw_right, raw_bottom = best_box[0], best_box[1], best_box[2], best_box[3]
        elif tracking_mode == "group":
            mode_text = f"Group Tracking ({current_num_people} people)"
            raw_left = min(b[0] for b in current_boxes)
            raw_right = max(b[2] for b in current_boxes)
            raw_top = min(b[1] for b in current_boxes)
            raw_bottom = max(b[3] for b in current_boxes)
    else:
        if not is_frozen and not target_lost:
            is_frozen, freeze_start_time = True, current_time

        if is_frozen:
            time_left = max(0.0, flicker_grace_sec - (current_time - freeze_start_time))
            if time_left > 0:
                mode_text = f"LOSS GRACE HOLD ({time_left:.1f}s)"
            else:
                is_frozen = False
                if tracking_mode == "single" or current_num_people == 0:
                    target_lost = True
                elif tracking_mode == "group":
                    expected_num_people = current_num_people
                    target_lost = False 

    for box in current_boxes:
        x1, y1, x2, y2 = int(box[0]), int(box[1]), int(box[2]), int(box[3])
        raw_cx, raw_cy = (x1 + x2) / 2, (y1 + y2) / 2
        is_target = target_found_this_frame and best_box is not None and np.array_equal(box, best_box)
        box_color = STANDBY_COLOR if is_target else (0, 0, 255)
        cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)
        draw_crosshair(frame, raw_cx, raw_cy, box_color, size=8, thickness=1)

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
            smoothers["left"].update(raw_left)
            smoothers["right"].update(raw_right)
            smoothers["top"].update(raw_top)
            smoothers["bottom"].update(raw_bottom)

        s_left, s_right = smoothers["left"].value, smoothers["right"].value
        s_top, s_bottom = smoothers["top"].value, smoothers["bottom"].value
        target_cx, target_cy = (s_left + s_right) / 2, (s_top + s_bottom) / 2
        offset_x, offset_y = target_cx - frame_center_x, target_cy - frame_center_y
        
        # --- SLIDE ---
        if not system_state["slide"]["locked"]:
            if not system_state["slide"]["recentering"] and abs(offset_x) > base_dz_slide:
                system_state["slide"]["recentering"] = True
            active_dz_s = int(base_dz_slide * RECENTER_RATIO) if system_state["slide"]["recentering"] else base_dz_slide
            if system_state["slide"]["recentering"] and abs(offset_x) <= active_dz_s:
                system_state["slide"]["recentering"] = False
                active_dz_s = base_dz_slide

            if abs(offset_x) > active_dz_s:
                system_state["slide"]["speed"], system_state["slide"]["dir"] = slide_pid.compute(offset_x)
            else: 
                system_state["slide"]["speed"], system_state["slide"]["dir"] = 0, 0
                slide_pid.reset()

        # --- PAN ---
        if not system_state["pan"]["locked"]:
            if not system_state["pan"]["recentering"] and abs(offset_x) > base_dz_pan:
                system_state["pan"]["recentering"] = True
            active_dz_p = int(base_dz_pan * RECENTER_RATIO) if system_state["pan"]["recentering"] else base_dz_pan
            if system_state["pan"]["recentering"] and abs(offset_x) <= active_dz_p:
                system_state["pan"]["recentering"] = False
                active_dz_p = base_dz_pan

            if abs(offset_x) > active_dz_p:
                system_state["pan"]["speed"], system_state["pan"]["dir"] = pan_pid.compute(offset_x)
            else: 
                system_state["pan"]["speed"], system_state["pan"]["dir"] = 0, 0
                pan_pid.reset()

        # --- TILT ---
        if not system_state["tilt"]["locked"]:
            if not system_state["tilt"]["recentering"] and abs(offset_y) > base_dz_tilt:
                system_state["tilt"]["recentering"] = True
            active_dz_t = int(base_dz_tilt * RECENTER_RATIO) if system_state["tilt"]["recentering"] else base_dz_tilt
            if system_state["tilt"]["recentering"] and abs(offset_y) <= active_dz_t:
                system_state["tilt"]["recentering"] = False
                active_dz_t = base_dz_tilt

            if abs(offset_y) > active_dz_t:
                system_state["tilt"]["speed"], system_state["tilt"]["dir"] = tilt_pid.compute(offset_y)
            else: 
                system_state["tilt"]["speed"], system_state["tilt"]["dir"] = 0, 0
                tilt_pid.reset()

        actively_sending_input = any(not system_state[a]["locked"] and system_state[a]["speed"] > 0 for a in ["slide", "pan", "tilt"])
        target_color = (0, 255, 0) if actively_sending_input else STANDBY_COLOR

        if tracking_mode == "single":
            cv2.rectangle(frame, (int(s_left), int(s_top)), (int(s_right), int(s_bottom)), target_color, 2)
            draw_crosshair(frame, target_cx, target_cy, target_color, size=8, thickness=2)
        elif tracking_mode == "group":
            cv2.circle(frame, (int(target_cx), int(target_cy)), 25, target_color, 3)
            cv2.circle(frame, (int(target_cx), int(target_cy)), 4, target_color, -1)

    # --- TRANSMIT DATA TO PI ---
    # ADDED ACCEL TO THE PAYLOAD
    payload = json.dumps({
        "slide": {"speed": system_state["slide"]["speed"], "dir": system_state["slide"]["dir"], "accel": slide_pid.accel},
        "pan":   {"speed": system_state["pan"]["speed"],   "dir": system_state["pan"]["dir"],   "accel": pan_pid.accel},
        "tilt":  {"speed": system_state["tilt"]["speed"],  "dir": system_state["tilt"]["dir"],  "accel": tilt_pid.accel}
    })
    udp_sock.sendto(payload.encode('utf-8'), (PI_IP, PI_PORT))

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
        c_tilt = RECENTER_COLOR if system_state["tilt"]["recentering"] else STANDBY_COLOR
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
        tracking_mode, target_person_index = "single", target_person_index - 1
        force_target_switch, target_lost = True, False
    elif key == ord('2'): 
        tracking_mode, target_person_index = "single", target_person_index + 1
        force_target_switch, target_lost = True, False
    elif key == ord('3'): 
        tracking_mode, expected_num_people, target_lost = "group", 0, False
    
    elif key == ord('i'): dz_scale = max(1.0, dz_scale - 0.5) 
    elif key == ord('o'): dz_scale = min(8.0, dz_scale + 0.5) 
    elif key == ord('['): flicker_grace_sec = max(0.0, flicker_grace_sec - 0.1) 
    elif key == ord(']'): flicker_grace_sec = min(5.0, flicker_grace_sec + 0.1) 

# Safe Shutdown
stop_payload = json.dumps({"slide": {"speed":0,"dir":0,"accel":0}, "pan": {"speed":0,"dir":0,"accel":0}, "tilt": {"speed":0,"dir":0,"accel":0}})
udp_sock.sendto(stop_payload.encode('utf-8'), (PI_IP, PI_PORT))

cap.release()
cv2.destroyAllWindows()
vision_thread.join()
print("System Successfully Offline.")