import cv2
import time
import os
import threading
import math
import socket
import json
import numpy as np
from ultralytics import YOLO

# --- AUTO-RECOVERY CONFIG ---
MAX_RECOVERY_ATTEMPTS = 30
RECOVERY_SEARCH_RADIUS = 250

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

# --- LOOP TIMING ---
TARGET_FPS = 60
FRAME_TIME = 1.0 / TARGET_FPS

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
        self.integral = max(min(self.integral, 2000), -2000)
        
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        raw_velocity = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        scaled_velocity = abs(raw_velocity) * GLOBAL_SPEED_SCALE
        
        speed = min(int(scaled_velocity), self.max_speed)
        direction = 1 if error > 0 else -1
        
        return speed, direction
        
    def reset(self):
        self.integral = 0
        self.prev_error = 0
        self.last_time = time.time()

class EMASmoother:
    def __init__(self, alpha):
        self.alpha, self.value = alpha, None
    def update(self, new_value):
        if self.value is None: self.value = new_value
        else: self.value += self.alpha * (new_value - self.value)
        return self.value

class MotorOutputSmoother:
    """Rate-limited EMA smoother for motor speed output (from orbit controller)."""
    def __init__(self, alpha, max_delta):
        self.alpha = alpha
        self.max_delta = max_delta
        self.value = 0.0
    def update(self, target):
        delta = target - self.value
        delta = max(-self.max_delta, min(self.max_delta, delta))
        self.value += delta
        self.value += self.alpha * (target - self.value)
        return max(0, int(self.value))
    def reset(self):
        self.value = 0.0

# --- 4. START BACKGROUND THREADS ---
vision_thread = threading.Thread(target=vision_worker, daemon=True)
vision_thread.start()

# --- 5. SETUP UTILS & PID ---
slide_pid = StepperPID(kp=0.075, ki=0.01, kd=3.5, max_speed=300, accel=400)
pan_pid   = StepperPID(kp=0.3, ki=0.01, kd=0.15, max_speed=200, accel=2000)  # Orbit-style P-dominant tuning
tilt_pid  = StepperPID(kp=0.075, ki=0.01, kd=3.5, max_speed=300, accel=400)

pan_smoother = MotorOutputSmoother(alpha=0.12, max_delta=10)  # Rate-limited motor output

smoothers = {k: EMASmoother(SMOOTHING_FACTOR) for k in ["left", "right", "top", "bottom"]}

# --- KALMAN FILTER (from orbit controller — tuned for steady drift prediction) ---
kalman = cv2.KalmanFilter(4, 2)
kalman.measurementMatrix = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0]], np.float32)
kalman.transitionMatrix = np.array([
    [1, 0, 1, 0],
    [0, 1, 0, 1],
    [0, 0, 1, 0],
    [0, 0, 0, 1]], np.float32)
kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 5e-3
kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 5e-1
kalman_initialized = False

def reset_kalman(cx, cy):
    global kalman_initialized
    kalman.statePre = np.array([[cx], [cy], [0], [0]], np.float32)
    kalman.statePost = np.array([[cx], [cy], [0], [0]], np.float32)
    kalman.errorCovPre = np.eye(4, dtype=np.float32)
    kalman.errorCovPost = np.eye(4, dtype=np.float32)
    kalman_initialized = True

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
target_lost = True

# Auto-recovery state (replaces flicker grace)
recovery_active = False
recovery_attempts = 0

force_target_switch = True
last_target_cx, last_target_cy = 0, 0

cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) 
cap.set(cv2.CAP_PROP_EXPOSURE, 300)

print("\n--- AXI6 MAC VISION SYSTEM ONLINE ---")

while cap.isOpened() and system_state["running"]:
    loop_start = time.time()
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
        # Also check recovery — if in recovery and YOLO finds a person near Kalman prediction, re-acquire
        elif recovery_active and current_num_people > 0 and kalman_initialized:
            prediction = kalman.predict()
            pred_cx = float(prediction[0].item())
            pred_cy = float(prediction[1].item())
            min_dist_to_pred = float('inf')
            for box in current_boxes:
                cx, cy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
                dist = math.hypot(cx - pred_cx, cy - pred_cy)
                if dist < min_dist_to_pred:
                    min_dist_to_pred = dist
                    best_box = box
            if best_box is not None and min_dist_to_pred < RECOVERY_SEARCH_RADIUS:
                target_found_this_frame = True
                last_target_cx = (best_box[0] + best_box[2]) / 2
                last_target_cy = (best_box[1] + best_box[3]) / 2
                target_person_index = next(i for i, b in enumerate(current_boxes) if np.array_equal(b, best_box))
                recovery_active = False
                recovery_attempts = 0
                print(f"[RECOVERY] Re-acquired target!")

    elif tracking_mode == "group":
        if current_num_people >= expected_num_people and current_num_people > 0:
            expected_num_people = current_num_people
            target_found_this_frame = True

    # --- THE STATE MACHINE (Kalman auto-recovery replaces flicker grace) ---
    if target_found_this_frame:
        target_lost = False
        if tracking_mode == "single":
            mode_text = f"Single Tracking (Person {target_person_index + 1}/{current_num_people})"
            raw_left, raw_top, raw_right, raw_bottom = best_box[0], best_box[1], best_box[2], best_box[3]
        elif tracking_mode == "group":
            mode_text = f"Group Tracking ({current_num_people} people)"
            raw_left = min(b[0] for b in current_boxes)
            raw_right = max(b[2] for b in current_boxes)
            raw_top = min(b[1] for b in current_boxes)
            raw_bottom = max(b[3] for b in current_boxes)
        
        # Update Kalman with measurement
        target_cx_raw = (raw_left + raw_right) / 2
        target_cy_raw = (raw_top + raw_bottom) / 2
        if not kalman_initialized:
            reset_kalman(target_cx_raw, target_cy_raw)
        kalman.correct(np.array([[np.float32(target_cx_raw)], [np.float32(target_cy_raw)]]))
    else:
        # --- KALMAN AUTO-RECOVERY (replaces flicker grace) ---
        if not recovery_active and not target_lost and kalman_initialized:
            recovery_active = True
            recovery_attempts = 0
            prediction = kalman.predict()
            pred_cx = float(prediction[0].item())
            pred_cy = float(prediction[1].item())
            print(f"[RECOVERY] Target lost — searching near ({int(pred_cx)}, {int(pred_cy)})...")

        if recovery_active and recovery_attempts < MAX_RECOVERY_ATTEMPTS:
            recovery_attempts += 1
            # Kalman coasts forward using velocity model
            prediction = kalman.predict()
            pred_cx = float(prediction[0].item())
            pred_cy = float(prediction[1].item())
            last_target_cx, last_target_cy = pred_cx, pred_cy

            # Draw ghost crosshair at predicted position
            ghost_color = (0, 0, 255)
            draw_crosshair(frame, pred_cx, pred_cy, ghost_color, size=20, thickness=3)
            cv2.circle(frame, (int(pred_cx), int(pred_cy)), RECOVERY_SEARCH_RADIUS, ghost_color, 1)
            mode_text = f"RECOVERING ({recovery_attempts}/{MAX_RECOVERY_ATTEMPTS})"

            # Ramp motors down smoothly during recovery
            if not system_state["pan"]["locked"]:
                system_state["pan"]["speed"] = pan_smoother.update(0)
            for axis in ["slide", "tilt"]:
                if not system_state[axis]["locked"]:
                    system_state[axis]["speed"] = 0
        elif recovery_active:
            recovery_active = False
            target_lost = True
            pan_smoother.reset()
            print("[RECOVERY] Failed — target fully lost.")

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

        # --- PAN (with orbit-style rate-limited output) ---
        if not system_state["pan"]["locked"]:
            if not system_state["pan"]["recentering"] and abs(offset_x) > base_dz_pan:
                system_state["pan"]["recentering"] = True
            active_dz_p = int(base_dz_pan * RECENTER_RATIO) if system_state["pan"]["recentering"] else base_dz_pan
            if system_state["pan"]["recentering"] and abs(offset_x) <= active_dz_p:
                system_state["pan"]["recentering"] = False
                active_dz_p = base_dz_pan

            if abs(offset_x) > active_dz_p:
                raw_speed, raw_dir = pan_pid.compute(offset_x)
                system_state["pan"]["speed"] = pan_smoother.update(raw_speed)
                system_state["pan"]["dir"] = raw_dir
            else: 
                system_state["pan"]["speed"] = pan_smoother.update(0)
                system_state["pan"]["dir"] = 0
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
    mode_color = (0, 165, 255) if recovery_active else (0, 0, 255) if target_lost else (255, 255, 0)
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

    # --- FRAME RATE CAP (60Hz) ---
    elapsed = time.time() - loop_start
    sleep_time = FRAME_TIME - elapsed
    if sleep_time > 0:
        time.sleep(sleep_time)

# Safe Shutdown
stop_payload = json.dumps({"slide": {"speed":0,"dir":0,"accel":0}, "pan": {"speed":0,"dir":0,"accel":0}, "tilt": {"speed":0,"dir":0,"accel":0}})
udp_sock.sendto(stop_payload.encode('utf-8'), (PI_IP, PI_PORT))

cap.release()
cv2.destroyAllWindows()
vision_thread.join()
print("System Successfully Offline.")