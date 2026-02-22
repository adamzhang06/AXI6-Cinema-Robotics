import cv2
import time
import os
import threading
import math
import socket
import json
import numpy as np
from ultralytics import YOLO

# ============================================================
#  AXI6 YOLO + KALMAN FILTER CONTROLLER
#  mac_yolo_KF.py — Clean rewrite with Kalman prediction.
#
#  Same YOLO features as mac_yolo.py (single/group tracking,
#  deadzone hysteresis, UDP motor commands to Pi) but with:
#   - Kalman filter for position smoothing & prediction
#   - Orbit-style P-dominant PID for pan
#   - Rate-limited motor output smoother for pan
#   - Grace period (1s) + Kalman recovery on target loss
#   - 60Hz frame rate cap for stable PID timing
#
#  CONTROLS:
#    S/P/T     - Toggle axis lock (slide/pan/tilt)
#    SPACE     - Emergency stop (lock all axes)
#    1/2       - Switch single-track target (prev/next)
#    3         - Switch to group tracking mode
#    I/O       - Shrink/Grow deadzone scale
#    [/]       - Adjust grace period duration
#    K         - Toggle Kalman filter on/off
#    Q         - Quit (sends stop command to Pi)
# ============================================================

# --- NETWORK SETUP (auto-discovery) ---
BEACON_PORT = 5006
PI_PORT = 5005

def discover_pi(timeout=10):
    """Listen for the Pi's beacon broadcast to auto-discover its IP."""
    print(f"[NETWORK] Searching for Pi motor server...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('', BEACON_PORT))
    sock.settimeout(timeout)
    try:
        data, addr = sock.recvfrom(1024)
        msg = json.loads(data.decode('utf-8'))
        if msg.get('service') == 'axi6_motor_server':
            pi_ip = addr[0]
            pi_port = msg.get('port', PI_PORT)
            print(f"[NETWORK] ✅ Found Pi at {pi_ip}:{pi_port}")
            sock.close()
            return pi_ip, pi_port
    except socket.timeout:
        print(f"[NETWORK] ❌ No Pi found after {timeout}s.")
        manual_ip = input("Enter Pi IP manually (or 'q' to quit): ").strip()
        sock.close()
        if manual_ip.lower() == 'q':
            exit(0)
        return manual_ip, PI_PORT
    sock.close()
    return None, PI_PORT

PI_IP, PI_PORT = discover_pi()
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

# --- GRACE PERIOD + RECOVERY ---
GRACE_PERIOD_SEC = 1.0          # Hold position this long before Kalman recovery
MAX_RECOVERY_FRAMES = 60        # ~1s at 60Hz — how long Kalman coasts
RECOVERY_SEARCH_RADIUS = 300    # px — how far from prediction to re-acquire

# ===================== CLASSES =====================

class StepperPID:
    """PID controller for stepper motor output."""
    def __init__(self, kp, ki, kd, max_speed, accel):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_speed = max_speed
        self.accel = accel
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()

    def compute(self, error):
        now = time.time()
        dt = max(now - self.last_time, 0.001)
        self.last_time = now
        self.integral += error * dt
        self.integral = max(min(self.integral, 2000), -2000)
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        raw = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        speed = min(int(abs(raw) * GLOBAL_SPEED_SCALE), self.max_speed)
        direction = 1 if error > 0 else -1
        return speed, direction

    def reset(self):
        self.integral = 0
        self.prev_error = 0
        self.last_time = time.time()


class EMASmoother:
    """Exponential moving average smoother for bounding box coords."""
    def __init__(self, alpha):
        self.alpha, self.value = alpha, None
    def update(self, new_value):
        if self.value is None: self.value = new_value
        else: self.value += self.alpha * (new_value - self.value)
        return self.value


class MotorOutputSmoother:
    """Rate-limited EMA smoother — prevents sudden motor speed jumps."""
    def __init__(self, alpha, max_delta):
        self.alpha = alpha
        self.max_delta = max_delta
        self.value = 0.0

    def update(self, target):
        delta = max(-self.max_delta, min(self.max_delta, target - self.value))
        self.value += delta
        self.value += self.alpha * (target - self.value)
        return max(0, int(self.value))

    def reset(self):
        self.value = 0.0


# ===================== SYSTEM STATE =====================

system_state = {
    "running": True,
    "slide": {"speed": 0, "dir": 0, "locked": True,  "recentering": False},
    "pan":   {"speed": 0, "dir": 0, "locked": False, "recentering": False},
    "tilt":  {"speed": 0, "dir": 0, "locked": True,  "recentering": False}
}

# ===================== YOLO VISION THREAD =====================

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

vision_thread = threading.Thread(target=vision_worker, daemon=True)
vision_thread.start()

# ===================== PID + SMOOTHERS =====================

slide_pid = StepperPID(kp=0.075, ki=0.01, kd=3.5,  max_speed=300, accel=400)
pan_pid   = StepperPID(kp=0.3,   ki=0.01, kd=0.15, max_speed=200, accel=2000)  # Orbit-style P-dominant
tilt_pid  = StepperPID(kp=0.075, ki=0.01, kd=3.5,  max_speed=300, accel=400)

pan_smoother = MotorOutputSmoother(alpha=0.12, max_delta=10)

smoothers = {k: EMASmoother(SMOOTHING_FACTOR) for k in ["left", "right", "top", "bottom"]}

# ===================== KALMAN FILTER =====================

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

kalman_enabled = True
kalman_initialized = False

def reset_kalman(cx, cy):
    global kalman_initialized
    kalman.statePre  = np.array([[cx], [cy], [0], [0]], np.float32)
    kalman.statePost = np.array([[cx], [cy], [0], [0]], np.float32)
    kalman.errorCovPre  = np.eye(4, dtype=np.float32)
    kalman.errorCovPost = np.eye(4, dtype=np.float32)
    kalman_initialized = True

def kalman_predict():
    """Get Kalman's current predicted position."""
    prediction = kalman.predict()
    return float(prediction[0].item()), float(prediction[1].item())

def kalman_correct(cx, cy):
    """Feed a measurement to the Kalman filter."""
    if not kalman_initialized:
        reset_kalman(cx, cy)
    kalman.correct(np.array([[np.float32(cx)], [np.float32(cy)]]))


# ===================== DRAWING HELPERS =====================

STANDBY_COLOR  = (255, 200, 50)
RECENTER_COLOR = (0, 0, 255)
GRACE_COLOR    = (0, 165, 255)
RECOVERY_COLOR = (0, 100, 255)
LOST_COLOR     = (0, 0, 255)
ACTIVE_COLOR   = (0, 255, 0)

def draw_outlined_text(img, text, pos, font_scale, color, thickness):
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), thickness + 3)
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, thickness)

def draw_crosshair(img, cx, cy, color, size=8, thickness=1):
    cv2.line(img, (int(cx) - size, int(cy)), (int(cx) + size, int(cy)), color, thickness)
    cv2.line(img, (int(cx), int(cy) - size), (int(cx), int(cy) + size), color, thickness)


# ===================== TRACKING STATE =====================

tracking_mode = "single"
target_person_index = 0
dz_scale = 1.0
expected_num_people = 0

force_target_switch = True
last_target_cx, last_target_cy = 0.0, 0.0

# Three-phase loss handling: TRACKING → GRACE → RECOVERY → LOST
# (no "is_frozen" / "ghost box" — clean state machine)
class TrackState:
    TRACKING  = "tracking"
    GRACE     = "grace"       # holding position, waiting for YOLO re-detect
    RECOVERY  = "recovery"    # Kalman coasting, actively searching
    LOST      = "lost"

track_state = TrackState.LOST
grace_start_time = 0.0
recovery_frame_count = 0

# ===================== CAMERA SETUP =====================

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
cap.set(cv2.CAP_PROP_EXPOSURE, 300)

print("\n--- AXI6 YOLO+KF CONTROLLER ONLINE ---")
print(f"Pan PID: Kp={pan_pid.kp} Ki={pan_pid.ki} Kd={pan_pid.kd} | Max={pan_pid.max_speed}")
print(f"Grace: {GRACE_PERIOD_SEC}s | Recovery: {MAX_RECOVERY_FRAMES} frames")
print("Press 'K' to toggle Kalman, 'Q' to quit.\n")

# ===================== MAIN LOOP =====================

while cap.isOpened() and system_state["running"]:
    loop_start = time.time()
    success, frame = cap.read()
    if not success:
        break
    frame = cv2.flip(frame, 1)
    current_time = time.time()

    if frame_for_yolo is None:
        frame_for_yolo = frame.copy()

    frame_h, frame_w, _ = frame.shape
    frame_cx, frame_cy = frame_w // 2, frame_h // 2

    base_dz_slide = int(BASE_DZ_SLIDE * dz_scale)
    base_dz_pan   = int(BASE_DZ_PAN * dz_scale)
    base_dz_tilt  = int(BASE_DZ_TILT * dz_scale)

    system_engaged = not all(system_state[a]["locked"] for a in ["slide", "pan", "tilt"])

    current_boxes = list(latest_boxes)
    current_num_people = len(current_boxes)

    if current_num_people > 0:
        current_boxes.sort(key=lambda b: b[0])
        target_person_index = target_person_index % current_num_people

    # ===================================================
    #  PHASE 1: TRY TO FIND TARGET IN CURRENT DETECTIONS
    # ===================================================
    target_found = False
    best_box = None

    if tracking_mode == "single" and current_num_people > 0:
        if force_target_switch:
            best_box = current_boxes[target_person_index]
            last_target_cx = (best_box[0] + best_box[2]) / 2
            last_target_cy = (best_box[1] + best_box[3]) / 2
            force_target_switch = False
            target_found = True
        else:
            # Find detection closest to last known position
            search_cx, search_cy = last_target_cx, last_target_cy

            # During recovery, use Kalman prediction as the search center
            if track_state == TrackState.RECOVERY and kalman_enabled and kalman_initialized:
                search_cx, search_cy = kalman_predict()

            min_dist = float('inf')
            for box in current_boxes:
                cx = (box[0] + box[2]) / 2
                cy = (box[1] + box[3]) / 2
                dist = math.hypot(cx - search_cx, cy - search_cy)
                if dist < min_dist:
                    min_dist = dist
                    best_box = box

            search_radius = RECOVERY_SEARCH_RADIUS if track_state == TrackState.RECOVERY else MAX_TRACK_DIST
            if best_box is not None and min_dist < search_radius:
                target_found = True
                last_target_cx = (best_box[0] + best_box[2]) / 2
                last_target_cy = (best_box[1] + best_box[3]) / 2
                target_person_index = next(i for i, b in enumerate(current_boxes) if np.array_equal(b, best_box))

    elif tracking_mode == "group":
        if current_num_people > 0:
            if current_num_people >= expected_num_people:
                expected_num_people = current_num_people
            target_found = True

    # ===================================================
    #  PHASE 2: STATE MACHINE
    #  TRACKING → GRACE (1s hold) → RECOVERY (Kalman) → LOST
    # ===================================================
    mode_text = ""
    raw_left = raw_top = raw_right = raw_bottom = 0

    if target_found:
        # --- TARGET FOUND: back to tracking ---
        if track_state in (TrackState.GRACE, TrackState.RECOVERY):
            print(f"[RECOVERY] Re-acquired target!")
        track_state = TrackState.TRACKING
        recovery_frame_count = 0

        if tracking_mode == "single":
            mode_text = f"Single Tracking (Person {target_person_index + 1}/{current_num_people})"
            raw_left, raw_top, raw_right, raw_bottom = best_box[0], best_box[1], best_box[2], best_box[3]
        elif tracking_mode == "group":
            mode_text = f"Group Tracking ({current_num_people} people)"
            raw_left   = min(b[0] for b in current_boxes)
            raw_right  = max(b[2] for b in current_boxes)
            raw_top    = min(b[1] for b in current_boxes)
            raw_bottom = max(b[3] for b in current_boxes)

        # Feed Kalman with the actual measurement
        meas_cx = (raw_left + raw_right) / 2
        meas_cy = (raw_top + raw_bottom) / 2
        if kalman_enabled:
            kalman_correct(meas_cx, meas_cy)

    else:
        # --- TARGET NOT FOUND ---
        if track_state == TrackState.TRACKING:
            # Just lost — enter grace period
            track_state = TrackState.GRACE
            grace_start_time = current_time

        if track_state == TrackState.GRACE:
            grace_remaining = GRACE_PERIOD_SEC - (current_time - grace_start_time)
            if grace_remaining > 0:
                # Hold position, keep using last known smoothed values
                mode_text = f"HOLD ({grace_remaining:.1f}s)"
            else:
                # Grace expired — start Kalman recovery
                if kalman_enabled and kalman_initialized:
                    track_state = TrackState.RECOVERY
                    recovery_frame_count = 0
                    pred_cx, pred_cy = kalman_predict()
                    print(f"[RECOVERY] Grace expired — Kalman searching near ({int(pred_cx)}, {int(pred_cy)})...")
                else:
                    track_state = TrackState.LOST

        if track_state == TrackState.RECOVERY:
            recovery_frame_count += 1
            if recovery_frame_count <= MAX_RECOVERY_FRAMES:
                # Kalman coasts using velocity model
                pred_cx, pred_cy = kalman_predict()
                last_target_cx, last_target_cy = pred_cx, pred_cy

                # Draw ghost crosshair at Kalman prediction
                draw_crosshair(frame, pred_cx, pred_cy, RECOVERY_COLOR, size=20, thickness=3)
                cv2.circle(frame, (int(pred_cx), int(pred_cy)), RECOVERY_SEARCH_RADIUS, RECOVERY_COLOR, 1)
                mode_text = f"RECOVERING ({recovery_frame_count}/{MAX_RECOVERY_FRAMES})"

                # Ramp motors down during recovery
                if not system_state["pan"]["locked"]:
                    system_state["pan"]["speed"] = pan_smoother.update(0)
                for axis in ["slide", "tilt"]:
                    if not system_state[axis]["locked"]:
                        system_state[axis]["speed"] = 0
            else:
                track_state = TrackState.LOST
                pan_smoother.reset()
                print("[RECOVERY] Failed — target fully lost.")

        if track_state == TrackState.LOST:
            mode_text = "TARGET LOST"

    # ===================================================
    #  PHASE 3: DRAW DETECTIONS
    # ===================================================
    for box in current_boxes:
        x1, y1, x2, y2 = int(box[0]), int(box[1]), int(box[2]), int(box[3])
        raw_cx, raw_cy = (x1 + x2) / 2, (y1 + y2) / 2
        is_target = target_found and best_box is not None and np.array_equal(box, best_box)
        box_color = STANDBY_COLOR if is_target else (0, 0, 255)
        cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)
        draw_crosshair(frame, raw_cx, raw_cy, box_color, size=8, thickness=1)

    # ===================================================
    #  PHASE 4: DEADZONE + MOTOR LOGIC
    # ===================================================
    active_dz_s = base_dz_slide
    active_dz_p = base_dz_pan
    active_dz_t = base_dz_tilt

    if track_state == TrackState.LOST:
        for axis in ["slide", "pan", "tilt"]:
            system_state[axis]["speed"] = 0
            system_state[axis]["dir"] = 0
            system_state[axis]["recentering"] = False
        slide_pid.reset(); pan_pid.reset(); tilt_pid.reset()
        pan_smoother.reset()

    elif track_state in (TrackState.TRACKING, TrackState.GRACE):
        # Update EMA smoothers only when we have fresh detections
        if track_state == TrackState.TRACKING:
            smoothers["left"].update(raw_left)
            smoothers["right"].update(raw_right)
            smoothers["top"].update(raw_top)
            smoothers["bottom"].update(raw_bottom)
        # During grace, keep using last smoothed values (don't update)

        s_left  = smoothers["left"].value
        s_right = smoothers["right"].value
        s_top   = smoothers["top"].value
        s_bottom = smoothers["bottom"].value

        if s_left is None:
            # Smoothers not initialized yet
            pass
        else:
            target_cx = (s_left + s_right) / 2
            target_cy = (s_top + s_bottom) / 2
            offset_x = target_cx - frame_cx
            offset_y = target_cy - frame_cy

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

            # --- PAN (orbit-style: PID → rate-limited smoother) ---
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

            # Draw tracking visuals
            actively_sending = any(not system_state[a]["locked"] and system_state[a]["speed"] > 0 for a in ["slide", "pan", "tilt"])
            target_color = ACTIVE_COLOR if actively_sending else STANDBY_COLOR

            if tracking_mode == "single":
                cv2.rectangle(frame, (int(s_left), int(s_top)), (int(s_right), int(s_bottom)), target_color, 2)
                draw_crosshair(frame, target_cx, target_cy, target_color, size=8, thickness=2)
            elif tracking_mode == "group":
                cv2.circle(frame, (int(target_cx), int(target_cy)), 25, target_color, 3)
                cv2.circle(frame, (int(target_cx), int(target_cy)), 4, target_color, -1)

    # --- RECOVERY state motor handling is done in Phase 2 above ---

    # ===================================================
    #  PHASE 5: TRANSMIT TO PI
    # ===================================================
    payload = json.dumps({
        "slide": {"speed": system_state["slide"]["speed"], "dir": system_state["slide"]["dir"], "accel": slide_pid.accel},
        "pan":   {"speed": system_state["pan"]["speed"],   "dir": system_state["pan"]["dir"],   "accel": pan_pid.accel},
        "tilt":  {"speed": system_state["tilt"]["speed"],  "dir": system_state["tilt"]["dir"],  "accel": tilt_pid.accel}
    })
    udp_sock.sendto(payload.encode('utf-8'), (PI_IP, PI_PORT))

    # ===================================================
    #  PHASE 6: DRAW DEADZONE BOUNDARIES
    # ===================================================
    if not system_state["slide"]["locked"]:
        c_slide = RECENTER_COLOR if system_state["slide"]["recentering"] else STANDBY_COLOR
        xl, xr = int(frame_cx - active_dz_s), int(frame_cx + active_dz_s)
        cv2.line(frame, (xl, 0), (xl, frame_h), c_slide, 1)
        cv2.line(frame, (xr, 0), (xr, frame_h), c_slide, 1)

    if not system_state["pan"]["locked"]:
        c_pan = RECENTER_COLOR if system_state["pan"]["recentering"] else STANDBY_COLOR
        xl, xr = int(frame_cx - active_dz_p), int(frame_cx + active_dz_p)
        yt, yb = frame_cy - 120, frame_cy + 120
        cv2.line(frame, (xl, yt), (xl, yb), c_pan, 2)
        cv2.line(frame, (xr, yt), (xr, yb), c_pan, 2)
        if xr < frame_w + 50:
            cv2.putText(frame, "PAN DZ", (xr + 5, yt - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, c_pan, 1)

    if not system_state["tilt"]["locked"]:
        c_tilt = RECENTER_COLOR if system_state["tilt"]["recentering"] else STANDBY_COLOR
        yt, yb = int(frame_cy - active_dz_t), int(frame_cy + active_dz_t)
        xl, xr = frame_cx - 150, frame_cx + 150
        cv2.line(frame, (xl, yt), (xr, yt), c_tilt, 2)
        cv2.line(frame, (xl, yb), (xr, yb), c_tilt, 2)

    # ===================================================
    #  PHASE 7: HUD
    # ===================================================
    if track_state == TrackState.TRACKING:
        mode_color = (255, 255, 0)
    elif track_state == TrackState.GRACE:
        mode_color = GRACE_COLOR
    elif track_state == TrackState.RECOVERY:
        mode_color = RECOVERY_COLOR
    else:
        mode_color = LOST_COLOR

    draw_outlined_text(frame, f"MODE: {mode_text}", (10, 35), 0.8, mode_color, 2)

    for i, axis in enumerate(["slide", "pan", "tilt"]):
        is_lck = system_state[axis]["locked"]
        col = (0, 0, 255) if is_lck else (0, 255, 0)
        stat = "LOCKED" if is_lck else f"SPD: {system_state[axis]['speed']} | DIR: {'+' if system_state[axis]['dir'] > 0 else '-'}"
        draw_outlined_text(frame, f"{axis.capitalize()}: {stat}", (10, 75 + (i * 35)), 0.7, col, 2)

    eng_txt = "ENGAGED" if system_engaged else "NOT ENGAGED"
    eng_col = (0, 255, 0) if system_engaged else (0, 0, 255)
    (ew, _), _ = cv2.getTextSize(eng_txt, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 3)
    draw_outlined_text(frame, eng_txt, (frame_w - ew - 20, 40), 1.0, eng_col, 3)

    kf_txt = f"KF: {'ON' if kalman_enabled else 'OFF'}"
    kf_col = (0, 255, 0) if kalman_enabled else (0, 0, 255)
    draw_outlined_text(frame, kf_txt, (frame_w - 120, 75), 0.6, kf_col, 2)

    draw_crosshair(frame, frame_cx, frame_cy, (200, 200, 200), size=12, thickness=1)

    view_frame = cv2.resize(frame, (1280, 720)) if frame_w == 1920 else frame
    cv2.imshow('AXI6 YOLO+KF', view_frame)

    # ===================================================
    #  PHASE 8: KEYBOARD CONTROLS
    # ===================================================
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        system_state["running"] = False
        break
    elif key == ord('s'):
        system_state["slide"]["locked"] = not system_state["slide"]["locked"]
    elif key == ord('p'):
        system_state["pan"]["locked"] = not system_state["pan"]["locked"]
    elif key == ord('t'):
        system_state["tilt"]["locked"] = not system_state["tilt"]["locked"]
    elif key == ord(' '):
        for a in ["slide", "pan", "tilt"]:
            system_state[a]["locked"] = True
            system_state[a]["speed"] = 0
            system_state[a]["dir"] = 0
        pan_smoother.reset()
        print("[EMERGENCY] All axes locked!")

    elif key == ord('1'):
        tracking_mode, target_person_index = "single", target_person_index - 1
        force_target_switch = True
        track_state = TrackState.TRACKING
    elif key == ord('2'):
        tracking_mode, target_person_index = "single", target_person_index + 1
        force_target_switch = True
        track_state = TrackState.TRACKING
    elif key == ord('3'):
        tracking_mode = "group"
        expected_num_people = 0
        track_state = TrackState.TRACKING

    elif key == ord('i'):
        dz_scale = max(1.0, dz_scale - 0.5)
    elif key == ord('o'):
        dz_scale = min(8.0, dz_scale + 0.5)
    elif key == ord('['):
        GRACE_PERIOD_SEC = max(0.0, GRACE_PERIOD_SEC - 0.1)
        print(f"[GRACE] {GRACE_PERIOD_SEC:.1f}s")
    elif key == ord(']'):
        GRACE_PERIOD_SEC = min(5.0, GRACE_PERIOD_SEC + 0.1)
        print(f"[GRACE] {GRACE_PERIOD_SEC:.1f}s")
    elif key == ord('k'):
        kalman_enabled = not kalman_enabled
        print(f"[KALMAN] {'ON' if kalman_enabled else 'OFF'}")

    # --- FRAME RATE CAP ---
    elapsed = time.time() - loop_start
    sleep_time = FRAME_TIME - elapsed
    if sleep_time > 0:
        time.sleep(sleep_time)

# ===================== SAFE SHUTDOWN =====================
stop_payload = json.dumps({
    "slide": {"speed": 0, "dir": 0, "accel": 0},
    "pan":   {"speed": 0, "dir": 0, "accel": 0},
    "tilt":  {"speed": 0, "dir": 0, "accel": 0}
})
udp_sock.sendto(stop_payload.encode('utf-8'), (PI_IP, PI_PORT))

cap.release()
cv2.destroyAllWindows()
vision_thread.join()
print("YOLO+KF Controller Offline.")
