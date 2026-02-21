import cv2
import time
import os
import threading
import math
import numpy as np
from ultralytics import YOLO

# --- CONFIGURABLE BASE SETTINGS ---
BASE_DZ_SLIDE = 60
BASE_DZ_PAN = 30
BASE_DZ_TILT = 40
SMOOTHING_FACTOR = 0.05 
MAX_TRACK_DIST = 250 

# --- 1. SHARED SYSTEM STATE ---
system_state = {
    "running": True,
    "slide": {"delay": 0.0, "dir": "RIGHT", "locked": True},
    "pan":   {"delay": 0.0, "dir": "RIGHT", "locked": True},
    "tilt":  {"delay": 0.0, "dir": "UP",    "locked": True}
}

# --- 2. MOTOR WORKER THREADS ---
def motor_worker(axis_name):
    print(f"[{axis_name.upper()}] Motor Thread Started.")
    while system_state["running"]:
        delay = system_state[axis_name]["delay"]
        is_locked = system_state[axis_name]["locked"]
        if delay > 0 and not is_locked:
            time.sleep(delay / 2)
            time.sleep(delay / 2)
        else:
            time.sleep(0.01)
    print(f"[{axis_name.upper()}] Motor Thread Stopped.")

# --- 3. VISION THREAD (YOLOv8) ---
latest_boxes = []
frame_for_yolo = None

def vision_worker():
    global latest_boxes, frame_for_yolo
    print("[VISION] Loading YOLO Nano TFLite Model...")
    
    # 1. Get the absolute path of the directory containing THIS script (tests/)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 2. Build the path dynamically: go up one folder ('..'), then into 'models'
    model_path = os.path.join(script_dir, '..', 'models', 'yolov8n_float16.tflite')
    
    # Optional: Print it out so you can verify it built the path correctly
    print(f"[VISION] Resolved model path: {model_path}")
    
    # Load the TFLite model using the dynamic path
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

# --- 4. PID & EMA CLASSES ---
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
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        speed = max(min(abs(output), self.max_speed), 0)
        return output, (1.0 / speed if speed > 0.1 else 0.0)

class EMASmoother:
    def __init__(self, alpha):
        self.alpha, self.value = alpha, None
    def update(self, new_value):
        if self.value is None: self.value = new_value
        else: self.value += self.alpha * (new_value - self.value)
        return self.value

# --- 5. START BACKGROUND THREADS ---
threads = []
for axis in ["slide", "pan", "tilt"]:
    t = threading.Thread(target=motor_worker, args=(axis,), daemon=True)
    t.start()
    threads.append(t)

vision_thread = threading.Thread(target=vision_worker, daemon=True)
vision_thread.start()

# --- 6. SETUP UTILS & SMOOTHERS ---
slide_pid = StepperPID(kp=1.5, ki=0.01, kd=0.5, max_speed=800)
pan_pid = StepperPID(kp=3.0, ki=0.05, kd=0.8, max_speed=2000)
tilt_pid = StepperPID(kp=3.0, ki=0.05, kd=0.8, max_speed=2000)

smoothers = {k: EMASmoother(SMOOTHING_FACTOR) for k in ["left", "right", "top", "bottom"]}

def draw_outlined_text(img, text, pos, font_scale, color, thickness):
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), thickness + 3)
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, thickness)

def draw_crosshair(img, cx, cy, color, size=8, thickness=1):
    cv2.line(img, (int(cx) - size, int(cy)), (int(cx) + size, int(cy)), color, thickness)
    cv2.line(img, (int(cx), int(cy) - size), (int(cx), int(cy) + size), color, thickness)

# --- 7. MAIN VISION LOOP ---
tracking_mode = "single"
target_person_index = 0
dz_scale = 1.0  
STANDBY_COLOR = (255, 200, 50) 

expected_num_people = 0
is_frozen = False
freeze_start_time = 0
flicker_grace_sec = 0.5 
target_lost = True

force_target_switch = True
last_target_cx, last_target_cy = 0, 0

cap = cv2.VideoCapture(0)
print("\n--- AXI6 VISION SYSTEM ONLINE ---")

while cap.isOpened():
    success, frame = cap.read()
    if not success: break
    frame = cv2.flip(frame, 1)
    current_time = time.time()

    # Feed the Vision Thread
    if frame_for_yolo is None:
        frame_for_yolo = frame.copy()

    frame_h, frame_w, _ = frame.shape
    frame_center_x, frame_center_y = int(frame_w / 2), int(frame_h / 2)
    dz_slide, dz_pan, dz_tilt = int(BASE_DZ_SLIDE * dz_scale), int(BASE_DZ_PAN * dz_scale), int(BASE_DZ_TILT * dz_scale)
    system_engaged = not all(system_state[a]["locked"] for a in ["slide", "pan", "tilt"])

    # Grab a safe copy of the latest boxes from the Vision Thread
    current_boxes = list(latest_boxes)
    current_num_people = len(current_boxes)
    
    # Sort boxes left to right based on x1 coordinate
    if current_num_people > 0:
        current_boxes.sort(key=lambda b: b[0])
        target_person_index = target_person_index % current_num_people

    target_found_this_frame = False
    best_box = None

    # --- CENTROID & GROUP TRACKING LOGIC ---
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
                # Update index to match the tracked person's new sorted position
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

    # --- DRAW RAW DETECTIONS ---
    for box in current_boxes:
        x1, y1, x2, y2 = int(box[0]), int(box[1]), int(box[2]), int(box[3])
        raw_cx, raw_cy = (x1 + x2) / 2, (y1 + y2) / 2
        
        # Check if this box is our specific target to highlight it
        is_target = target_found_this_frame and best_box is not None and np.array_equal(box, best_box)
        box_color = STANDBY_COLOR if is_target else (0, 0, 255)
        
        cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)
        draw_crosshair(frame, raw_cx, raw_cy, box_color, size=8, thickness=1)

    # --- MOTOR & GHOST BOX LOGIC ---
    if target_lost:
        for axis in ["slide", "pan", "tilt"]: system_state[axis]["delay"] = 0.0
        slide_pid.integral, pan_pid.integral, tilt_pid.integral = 0, 0, 0
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
        
        if not system_state["slide"]["locked"] and abs(offset_x) > dz_slide:
            out_s, del_s = slide_pid.compute(offset_x)
            system_state["slide"]["dir"], system_state["slide"]["delay"] = ("RIGHT" if out_s > 0 else "LEFT"), del_s
        else: system_state["slide"]["delay"], slide_pid.integral = 0.0, 0

        if not system_state["pan"]["locked"] and abs(offset_x) > dz_pan:
            out_p, del_p = pan_pid.compute(offset_x)
            system_state["pan"]["dir"], system_state["pan"]["delay"] = ("RIGHT" if out_p > 0 else "LEFT"), del_p
        else: system_state["pan"]["delay"], pan_pid.integral = 0.0, 0

        if not system_state["tilt"]["locked"] and abs(offset_y) > dz_tilt:
            out_t, del_t = tilt_pid.compute(offset_y)
            system_state["tilt"]["dir"], system_state["tilt"]["delay"] = ("DOWN" if out_t > 0 else "UP"), del_t
        else: system_state["tilt"]["delay"], tilt_pid.integral = 0.0, 0

        actively_sending_input = any(not system_state[a]["locked"] and system_state[a]["delay"] > 0 for a in ["slide", "pan", "tilt"])
        target_color = (0, 255, 0) if actively_sending_input else STANDBY_COLOR

        if tracking_mode == "single":
            cv2.rectangle(frame, (int(s_left), int(s_top)), (int(s_right), int(s_bottom)), target_color, 2)
            draw_crosshair(frame, target_cx, target_cy, target_color, size=8, thickness=2)
        elif tracking_mode == "group":
            cv2.circle(frame, (int(target_cx), int(target_cy)), 25, target_color, 3)
            cv2.circle(frame, (int(target_cx), int(target_cy)), 4, target_color, -1)

    # --- DRAW ACTIVE DEADZONE BOUNDARIES ---
    if not system_state["slide"]["locked"]:
        xl, xr = int(frame_center_x - dz_slide), int(frame_center_x + dz_slide)
        cv2.line(frame, (xl, 0), (xl, frame_h), STANDBY_COLOR, 1)
        cv2.line(frame, (xr, 0), (xr, frame_h), STANDBY_COLOR, 1)
        if xl > -50: cv2.putText(frame, "SLIDE DZ", (xl + 5, frame_h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, STANDBY_COLOR, 1)

    if not system_state["pan"]["locked"]:
        xl, xr = int(frame_center_x - dz_pan), int(frame_center_x + dz_pan)
        yt, yb = frame_center_y - 120, frame_center_y + 120
        cv2.line(frame, (xl, yt), (xl, yb), STANDBY_COLOR, 2)
        cv2.line(frame, (xr, yt), (xr, yb), STANDBY_COLOR, 2)
        if xr < frame_w + 50: cv2.putText(frame, "PAN DZ", (xr + 5, yt - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, STANDBY_COLOR, 1)

    if not system_state["tilt"]["locked"]:
        yt, yb = int(frame_center_y - dz_tilt), int(frame_center_y + dz_tilt)
        xl, xr = frame_center_x - 150, frame_center_x + 150
        cv2.line(frame, (xl, yt), (xr, yt), STANDBY_COLOR, 2)
        cv2.line(frame, (xl, yb), (xr, yb), STANDBY_COLOR, 2)
        if yt > -50: cv2.putText(frame, "TILT DZ", (xr + 5, yt - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, STANDBY_COLOR, 1)

    # --- DRAW THE HIGH-VISIBILITY HUD ---
    mode_color = (0, 165, 255) if is_frozen else (0, 0, 255) if target_lost else (255, 255, 0)
    draw_outlined_text(frame, f"MODE: {mode_text}", (10, 35), 0.8, mode_color, 2)
    
    for i, axis in enumerate(["slide", "pan", "tilt"]):
        is_lck = system_state[axis]['locked']
        col = (0, 0, 255) if is_lck else (0, 255, 0)
        stat = "LOCKED" if is_lck else f"{system_state[axis]['dir']} ({system_state[axis]['delay']:.5f}s)"
        draw_outlined_text(frame, f"{axis.capitalize()}: {stat}", (10, 75 + (i*35)), 0.7, col, 2)
    
    eng_txt = "ENGAGED" if system_engaged else "NOT ENGAGED"
    eng_col = (0, 255, 0) if system_engaged else (0, 0, 255)
    (ew, eh), _ = cv2.getTextSize(eng_txt, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 3)
    draw_outlined_text(frame, eng_txt, (frame_w - ew - 20, 40), 1.0, eng_col, 3)
    
    dz_txt = f"DZ SCALE: {dz_scale:.1f}x"
    (dw, dh), _ = cv2.getTextSize(dz_txt, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
    draw_outlined_text(frame, dz_txt, (frame_w - dw - 20, 75), 0.6, STANDBY_COLOR, 2)

    grc_txt = f"GRACE: {flicker_grace_sec:.1f}s"
    (gw, gh), _ = cv2.getTextSize(grc_txt, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
    draw_outlined_text(frame, grc_txt, (frame_w - gw - 20, 105), 0.6, (0, 165, 255), 2)

    draw_crosshair(frame, frame_center_x, frame_center_y, (200, 200, 200), size=12, thickness=1)
    cv2.imshow('AXI6 Multi-Threaded Tracker', frame)
    
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

cap.release()
cv2.destroyAllWindows()
for t in threads: t.join()
vision_thread.join()
print("System Successfully Offline.")