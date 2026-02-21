import cv2
import time
import os
import threading
import math
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe.tasks.python.vision import FaceDetectorResult

# --- CONFIGURABLE BASE SETTINGS ---
BASE_DZ_SLIDE = 60
BASE_DZ_PAN = 30
BASE_DZ_TILT = 40
SMOOTHING_FACTOR = 0.05 
MAX_TRACK_DIST = 250 # Max pixels a face can move between frames and still be considered the "same" person

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
        dt = current_time - self.last_time
        if dt <= 0.0: dt = 0.001 
        self.last_time = current_time

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        speed = max(min(abs(output), self.max_speed), 0)
        step_delay = 1.0 / speed if speed > 0.1 else 0.0 
        return output, step_delay

class EMASmoother:
    def __init__(self, alpha):
        self.alpha = alpha
        self.value = None

    def update(self, new_value):
        if self.value is None:
            self.value = new_value
        else:
            self.value += self.alpha * (new_value - self.value)
        return self.value

# --- 4. START BACKGROUND THREADS ---
threads = []
for axis in ["slide", "pan", "tilt"]:
    t = threading.Thread(target=motor_worker, args=(axis,), daemon=True)
    t.start()
    threads.append(t)

# --- 5. MEDIAPIPE SETUP ---
model_path = os.path.expanduser('./models/blaze_face_short_range.tflite')
latest_result = None

def result_callback(result: FaceDetectorResult, output_image: mp.Image, timestamp_ms: int):
    global latest_result
    latest_result = result

options = vision.FaceDetectorOptions(
    base_options=python.BaseOptions(model_asset_path=model_path),
    running_mode=vision.RunningMode.LIVE_STREAM,
    result_callback=result_callback
)

slide_pid = StepperPID(kp=1.5, ki=0.01, kd=0.5, max_speed=800)
pan_pid = StepperPID(kp=3.0, ki=0.05, kd=0.8, max_speed=2000)
tilt_pid = StepperPID(kp=3.0, ki=0.05, kd=0.8, max_speed=2000)

smoothers = {
    "left": EMASmoother(SMOOTHING_FACTOR),
    "right": EMASmoother(SMOOTHING_FACTOR),
    "top": EMASmoother(SMOOTHING_FACTOR),
    "bottom": EMASmoother(SMOOTHING_FACTOR)
}

# --- HELPER GRAPHICS FUNCTIONS ---
def draw_outlined_text(img, text, pos, font_scale, color, thickness):
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), thickness + 3)
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, thickness)

def draw_crosshair(img, cx, cy, color, size=8, thickness=1):
    cv2.line(img, (int(cx) - size, int(cy)), (int(cx) + size, int(cy)), color, thickness)
    cv2.line(img, (int(cx), int(cy) - size), (int(cx), int(cy) + size), color, thickness)

# --- 6. MAIN VISION LOOP ---
tracking_mode = "single"
target_face_index = 0
dz_scale = 1.0  
STANDBY_COLOR = (255, 200, 50) 

# --- STATE MACHINE VARIABLES ---
expected_num_faces = 0
is_frozen = False
freeze_start_time = 0
flicker_grace_sec = 1.0 
target_lost = True

# Centroid Tracking Variables
force_target_switch = True
last_target_cx, last_target_cy = 0, 0

with vision.FaceDetector.create_from_options(options) as detector:
    cap = cv2.VideoCapture(0)
    print("\n--- AXI6 VISION SYSTEM ONLINE ---")

    while cap.isOpened():
        success, frame = cap.read()
        if not success: break
        frame = cv2.flip(frame, 1)
        current_time = time.time()

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
        detector.detect_async(mp_image, int(current_time * 1000))

        frame_h, frame_w, _ = frame.shape
        frame_center_x, frame_center_y = int(frame_w / 2), int(frame_h / 2)
        
        dz_slide = int(BASE_DZ_SLIDE * dz_scale)
        dz_pan = int(BASE_DZ_PAN * dz_scale)
        dz_tilt = int(BASE_DZ_TILT * dz_scale)
        system_engaged = not (system_state["slide"]["locked"] and system_state["pan"]["locked"] and system_state["tilt"]["locked"])

        current_num_faces = 0
        sorted_faces = []
        if latest_result and latest_result.detections:
            sorted_faces = sorted(latest_result.detections, key=lambda d: d.bounding_box.origin_x)
            current_num_faces = len(sorted_faces)

        target_found_this_frame = False
        best_face_d = None # Stores the specific MediaPipe object we lock onto

        # --- CENTROID & GROUP TRACKING LOGIC ---
        if tracking_mode == "single":
            if current_num_faces > 0:
                if force_target_switch:
                    # User manually picked a new face index
                    target_face_index = target_face_index % current_num_faces
                    best_face_d = sorted_faces[target_face_index]
                    b = best_face_d.bounding_box
                    last_target_cx, last_target_cy = b.origin_x + b.width/2, b.origin_y + b.height/2
                    force_target_switch = False
                    target_found_this_frame = True
                else:
                    # Find the face closest to where our target was last seen
                    min_dist = float('inf')
                    for d in sorted_faces:
                        b = d.bounding_box
                        cx, cy = b.origin_x + b.width/2, b.origin_y + b.height/2
                        dist = math.hypot(cx - last_target_cx, cy - last_target_cy)
                        if dist < min_dist:
                            min_dist = dist
                            best_face_d = d
                    
                    # Only accept it if it hasn't teleported across the room
                    if best_face_d and min_dist < MAX_TRACK_DIST:
                        target_found_this_frame = True
                        b = best_face_d.bounding_box
                        last_target_cx, last_target_cy = b.origin_x + b.width/2, b.origin_y + b.height/2
                        target_face_index = sorted_faces.index(best_face_d)

        elif tracking_mode == "group":
            if current_num_faces >= expected_num_faces and current_num_faces > 0:
                expected_num_faces = current_num_faces
                target_found_this_frame = True

        # --- THE STATE MACHINE ---
        if target_found_this_frame:
            is_frozen = False
            target_lost = False
            
            if tracking_mode == "single":
                mode_text = f"Single Tracking (Face {target_face_index + 1}/{current_num_faces})"
                raw_left, raw_top = best_face_d.bounding_box.origin_x, best_face_d.bounding_box.origin_y
                raw_right, raw_bottom = raw_left + best_face_d.bounding_box.width, raw_top + best_face_d.bounding_box.height
            elif tracking_mode == "group":
                mode_text = f"Group Tracking ({current_num_faces} faces)"
                raw_left = min([d.bounding_box.origin_x for d in sorted_faces])
                raw_right = max([d.bounding_box.origin_x + d.bounding_box.width for d in sorted_faces])
                raw_top = min([d.bounding_box.origin_y for d in sorted_faces])
                raw_bottom = max([d.bounding_box.origin_y + d.bounding_box.height for d in sorted_faces])
                
        else:
            if not is_frozen and not target_lost:
                is_frozen = True
                freeze_start_time = current_time

            if is_frozen:
                time_left = max(0.0, flicker_grace_sec - (current_time - freeze_start_time))
                if time_left > 0:
                    mode_text = f"LOSS GRACE HOLD ({time_left:.1f}s)"
                else:
                    is_frozen = False
                    if tracking_mode == "single" or current_num_faces == 0:
                        target_lost = True
                    elif tracking_mode == "group":
                        # Grace expired, accept the smaller group size
                        expected_num_faces = current_num_faces
                        target_lost = False 

        # --- DRAW RAW FACES ---
        for d in sorted_faces:
            b = d.bounding_box
            raw_cx, raw_cy = b.origin_x + b.width/2, b.origin_y + b.height/2
            # Highlight specifically tracked raw face in Standby Color, others in Red
            box_color = STANDBY_COLOR if (target_found_this_frame and d == best_face_d) else (0, 0, 255)
            cv2.rectangle(frame, (int(b.origin_x), int(b.origin_y)), (int(b.origin_x + b.width), int(b.origin_y + b.height)), box_color, 2)
            draw_crosshair(frame, raw_cx, raw_cy, box_color, size=8, thickness=1)

        # --- MOTOR & GHOST BOX LOGIC ---
        if target_lost:
            for axis in ["slide", "pan", "tilt"]:
                system_state[axis]["delay"] = 0.0
            slide_pid.integral, pan_pid.integral, tilt_pid.integral = 0, 0, 0
            actively_sending_input = False
            mode_text = "TARGET LOST"
            
        else:
            # ONLY update the Ghost Box EMA if we are NOT frozen
            if not is_frozen:
                smoothers["left"].update(raw_left)
                smoothers["right"].update(raw_right)
                smoothers["top"].update(raw_top)
                smoothers["bottom"].update(raw_bottom)

            s_left, s_right = smoothers["left"].value, smoothers["right"].value
            s_top, s_bottom = smoothers["top"].value, smoothers["bottom"].value
            face_center_x, face_center_y = (s_left + s_right) / 2, (s_top + s_bottom) / 2

            offset_x, offset_y = face_center_x - frame_center_x, face_center_y - frame_center_y
            
            if not system_state["slide"]["locked"] and abs(offset_x) > dz_slide:
                output_s, delay_s = slide_pid.compute(offset_x)
                system_state["slide"]["dir"], system_state["slide"]["delay"] = ("RIGHT" if output_s > 0 else "LEFT"), delay_s
            else: system_state["slide"]["delay"], slide_pid.integral = 0.0, 0
            
            if not system_state["pan"]["locked"] and abs(offset_x) > dz_pan:
                output_p, delay_p = pan_pid.compute(offset_x)
                system_state["pan"]["dir"], system_state["pan"]["delay"] = ("RIGHT" if output_p > 0 else "LEFT"), delay_p
            else: system_state["pan"]["delay"], pan_pid.integral = 0.0, 0

            if not system_state["tilt"]["locked"] and abs(offset_y) > dz_tilt:
                output_t, delay_t = tilt_pid.compute(offset_y)
                system_state["tilt"]["dir"], system_state["tilt"]["delay"] = ("DOWN" if output_t > 0 else "UP"), delay_t
            else: system_state["tilt"]["delay"], tilt_pid.integral = 0.0, 0

            actively_sending_input = any(not system_state[axis]["locked"] and system_state[axis]["delay"] > 0 for axis in ["slide", "pan", "tilt"])
            target_color = (0, 255, 0) if actively_sending_input else STANDBY_COLOR

            # Draw the Smooth Ghost Box
            if tracking_mode == "single":
                cv2.rectangle(frame, (int(s_left), int(s_top)), (int(s_right), int(s_bottom)), target_color, 2)
                draw_crosshair(frame, face_center_x, face_center_y, target_color, size=8, thickness=2)
            elif tracking_mode == "group":
                cv2.circle(frame, (int(face_center_x), int(face_center_y)), 25, target_color, 3)
                cv2.circle(frame, (int(face_center_x), int(face_center_y)), 4, target_color, -1)

        # --- DRAW ACTIVE DEADZONE BOUNDARIES ---
        if not system_state["slide"]["locked"]:
            x_left, x_right = int(frame_center_x - dz_slide), int(frame_center_x + dz_slide)
            cv2.line(frame, (x_left, 0), (x_left, frame_h), STANDBY_COLOR, 1)
            cv2.line(frame, (x_right, 0), (x_right, frame_h), STANDBY_COLOR, 1)
            if x_left > -50: cv2.putText(frame, "SLIDE DZ", (x_left + 5, frame_h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, STANDBY_COLOR, 1)

        if not system_state["pan"]["locked"]:
            x_left, x_right = int(frame_center_x - dz_pan), int(frame_center_x + dz_pan)
            y_top, y_bottom = frame_center_y - 120, frame_center_y + 120
            cv2.line(frame, (x_left, y_top), (x_left, y_bottom), STANDBY_COLOR, 2)
            cv2.line(frame, (x_right, y_top), (x_right, y_bottom), STANDBY_COLOR, 2)
            if x_right < frame_w + 50: cv2.putText(frame, "PAN DZ", (x_right + 5, y_top - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, STANDBY_COLOR, 1)

        if not system_state["tilt"]["locked"]:
            y_top, y_bottom = int(frame_center_y - dz_tilt), int(frame_center_y + dz_tilt)
            x_left, x_right = frame_center_x - 150, frame_center_x + 150
            cv2.line(frame, (x_left, y_top), (x_right, y_top), STANDBY_COLOR, 2)
            cv2.line(frame, (x_left, y_bottom), (x_right, y_bottom), STANDBY_COLOR, 2)
            if y_top > -50: cv2.putText(frame, "TILT DZ", (x_right + 5, y_top - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, STANDBY_COLOR, 1)

        # --- DRAW THE HIGH-VISIBILITY HUD ---
        mode_color = (0, 165, 255) if is_frozen else (0, 0, 255) if target_lost else (255, 255, 0)
        draw_outlined_text(frame, f"MODE: {mode_text}", (10, 35), 0.8, mode_color, 2)
        
        for i, axis in enumerate(["slide", "pan", "tilt"]):
            is_locked = system_state[axis]['locked']
            color = (0, 0, 255) if is_locked else (0, 255, 0)
            status_text = "LOCKED" if is_locked else f"{system_state[axis]['dir']} ({system_state[axis]['delay']:.5f}s)"
            draw_outlined_text(frame, f"{axis.capitalize()}: {status_text}", (10, 75 + (i*35)), 0.7, color, 2)
        
        engaged_text = "ENGAGED" if system_engaged else "NOT ENGAGED"
        engaged_color = (0, 255, 0) if system_engaged else (0, 0, 255)
        (eng_w, eng_h), _ = cv2.getTextSize(engaged_text, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 3)
        draw_outlined_text(frame, engaged_text, (frame_w - eng_w - 20, 40), 1.0, engaged_color, 3)
        
        dz_text = f"DZ SCALE: {dz_scale:.1f}x"
        (dz_w, dz_h), _ = cv2.getTextSize(dz_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        draw_outlined_text(frame, dz_text, (frame_w - dz_w - 20, 75), 0.6, STANDBY_COLOR, 2)

        grace_text = f"GRACE: {flicker_grace_sec:.1f}s"
        (grc_w, grc_h), _ = cv2.getTextSize(grace_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        draw_outlined_text(frame, grace_text, (frame_w - grc_w - 20, 105), 0.6, (0, 165, 255), 2)

        draw_crosshair(frame, frame_center_x, frame_center_y, (200, 200, 200), size=12, thickness=1)
        cv2.imshow('AXI6 Multi-Threaded Tracker', frame)
        
        # --- KEYBOARD CONTROLS ---
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'): 
            system_state["running"] = False
            break
        elif key == ord('s'): system_state["slide"]["locked"] = not system_state["slide"]["locked"]
        elif key == ord('p'): system_state["pan"]["locked"] = not system_state["pan"]["locked"]
        elif key == ord('t'): system_state["tilt"]["locked"] = not system_state["tilt"]["locked"]
        elif key == ord(' '): 
            system_state["slide"]["locked"], system_state["pan"]["locked"], system_state["tilt"]["locked"] = True, True, True
            
        elif key == ord('1'): 
            tracking_mode, target_face_index = "single", target_face_index - 1
            force_target_switch, target_lost = True, False
        elif key == ord('2'): 
            tracking_mode, target_face_index = "single", target_face_index + 1
            force_target_switch, target_lost = True, False
        elif key == ord('3'): 
            tracking_mode, expected_num_faces = "group", 0 
            target_lost = False
        
        elif key == ord('i'): dz_scale = max(1.0, dz_scale - 0.5) 
        elif key == ord('o'): dz_scale = min(8.0, dz_scale + 0.5) 
        elif key == ord('['): flicker_grace_sec = max(0.0, flicker_grace_sec - 0.1) 
        elif key == ord(']'): flicker_grace_sec = min(5.0, flicker_grace_sec + 0.1) 

cap.release()
cv2.destroyAllWindows()
for t in threads: t.join()
print("System Successfully Offline.")