import cv2
import time
import numpy as np

# ============================================================
#  AXI6 ORBIT TRACKER TEST (v5 — Orbit Compensation)
#  Simulates the pan/tilt compensation layer for orbit shots.
#
#  The slide moves on a trapezoidal velocity curve (pre-planned).
#  Pan & tilt REACT to keep the tracked subject centered.
#  This script tests that reactive tracking layer.
#
#  CONTROLS:
#    R         - Draw a new ROI (click & drag)
#    C         - Cancel tracking
#    K         - Toggle Kalman filter
#    A         - Toggle scale adaptation
#    D         - Toggle deadzone visualization
#    G         - START/STOP simulated slide drift (test mode)
#    +/-       - Adjust deadzone size
#    1/2/3     - Switch tracker: 1=KCF, 2=CSRT, 3=MOSSE
#    Q         - Quit
# ============================================================

# --- DISPLAY SETTINGS ---
CAPTURE_W, CAPTURE_H = 1920, 1080
DISPLAY_W, DISPLAY_H = 1280, 720
DISPLAY_SCALE_X = CAPTURE_W / DISPLAY_W
DISPLAY_SCALE_Y = CAPTURE_H / DISPLAY_H

# --- LOOP TIMING ---
TARGET_FPS = 60
FRAME_TIME = 1.0 / TARGET_FPS

# --- TIGHT DEADZONE FOR ORBIT SHOTS ---
# Small deadzone = the pan/tilt reacts early, keeping subject locked center.
# The hysteresis (inner DZ is smaller) prevents chatter at the boundary.
DEADZONE_X = 30            # Outer DZ: start correcting when offset > this
DEADZONE_Y = 20
INNER_DZ_RATIO = 0.5       # Inner DZ: stop correcting when offset < DZ * this

# --- TRACKING AT HALF RES ---
TRACK_SCALE = 0.5

# --- SCALE ADAPTATION ---
SCALE_CHECK_INTERVAL = 8
SCALE_CANDIDATES = [0.90, 0.95, 1.0, 1.05, 1.10]
SCALE_SMOOTHING = 0.3
MIN_BOX_SIZE = 30
MAX_BOX_RATIO = 0.8

# --- PID TUNING FOR SMOOTH ORBIT COMPENSATION ---
# P is king: motor speed is proportional to distance from deadzone.
# Ki slowly ramps for persistent offsets that P alone can't zero out.
# Kd is intentionally tiny — just enough to slightly dampen overshoot.
#   At 60Hz, dt=0.016s, so even Kd=0.05 produces meaningful damping
#   without overwhelming the proportional term.
# Rule of thumb: 100px offset ≈ 60 motor speed, 200px ≈ 120.
PAN_KP = 0.6               # Proportional: THE primary driver
PAN_KI = 0.015             # Integral: gentle ramp for persistent error
PAN_KD = 0.25              # Derivative: whisper of damping, NOT dominant
TILT_KP = 0.6
TILT_KI = 0.015
TILT_KD = 0.25

# --- MOTOR OUTPUT SMOOTHING ---
# Rate limiting prevents sudden speed jumps (jerky motion).
# Max speed change per frame — acts like motor acceleration.
MAX_SPEED_CHANGE = 15       # Max speed delta per frame (lower = smoother starts/stops)
MAX_MOTOR_SPEED = 300
MOTOR_SMOOTHING = 0.15      # EMA factor for motor output (lower = more cinematic lag)

# --- SIMULATED SLIDE DRIFT ---
# Simulates a trapezoidal slide move to test compensation without hardware.
# When active, the "frame center" shifts left-to-right to mimic how the
# subject drifts in-frame as the slide moves.
SIM_SLIDE_DISTANCE = 500    # Total pixels of simulated drift
SIM_SLIDE_MAX_SPEED = 3.0   # Pixels per frame at cruise
SIM_SLIDE_ACCEL = 0.05      # Pixels per frame^2

# ===================== CLASSES =====================

class StepperPID:
    """PID controller tuned for smooth stepper motor output."""
    def __init__(self, kp, ki, kd, max_speed):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_speed = max_speed
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
        speed = min(int(abs(raw)), self.max_speed)
        direction = 1 if error > 0 else -1
        return speed, direction

    def reset(self):
        self.integral = 0
        self.prev_error = 0
        self.last_time = time.time()


class MotorOutputSmoother:
    """Rate-limited EMA smoother for motor speed output.
    Prevents sudden speed jumps for cinematic smoothness."""
    def __init__(self, alpha, max_delta):
        self.alpha = alpha
        self.max_delta = max_delta
        self.value = 0.0

    def update(self, target):
        # Rate limit: cap how fast speed can change per frame
        delta = target - self.value
        delta = max(-self.max_delta, min(self.max_delta, delta))
        self.value += delta
        # EMA on top for extra smoothness
        self.value += self.alpha * (target - self.value)
        return max(0, int(self.value))

    def reset(self):
        self.value = 0.0


class TrapezoidalSlideSimulator:
    """Simulates a trapezoidal velocity profile slide move.
    Returns the current "drift offset" that would appear in frame."""
    def __init__(self, distance, max_speed, accel):
        self.distance = distance
        self.max_speed = max_speed
        self.accel = accel
        self.reset()

    def reset(self):
        self.active = False
        self.position = 0.0
        self.velocity = 0.0
        self.phase = "idle"  # idle, accel, cruise, decel, done

    def start(self):
        self.active = True
        self.position = 0.0
        self.velocity = 0.0
        self.phase = "accel"
        # Calculate decel start point (symmetric accel/decel)
        accel_dist = (self.max_speed ** 2) / (2 * self.accel)
        if accel_dist * 2 > self.distance:
            # Not enough room to reach max speed — triangle profile
            self.decel_start = self.distance / 2
        else:
            self.decel_start = self.distance - accel_dist

    def update(self):
        if not self.active:
            return 0.0

        if self.phase == "accel":
            self.velocity = min(self.velocity + self.accel, self.max_speed)
            if self.position >= self.decel_start:
                self.phase = "decel"
            elif self.velocity >= self.max_speed:
                self.phase = "cruise"

        elif self.phase == "cruise":
            if self.position >= self.decel_start:
                self.phase = "decel"

        elif self.phase == "decel":
            self.velocity = max(self.velocity - self.accel, 0)
            if self.velocity <= 0 or self.position >= self.distance:
                self.phase = "done"
                self.velocity = 0
                self.active = False

        self.position += self.velocity
        return self.position


# ===================== TRACKER UTILS =====================

TRACKER_TYPES = ["KCF", "CSRT", "MOSSE"]
current_tracker_idx = 0

tracker = None
tracking_active = False
track_box = None
kalman_enabled = True
scale_adapt_enabled = False
show_deadzones = True
tracker_fps = 0.0
overall_fps = 0.0
frame_counter = 0
initial_template = None
current_scale = 1.0
original_box_w = 0
original_box_h = 0

# Hysteresis state
pan_recentering = False
tilt_recentering = False

def create_tracker(tracker_type):
    if tracker_type == "CSRT":
        return cv2.TrackerCSRT_create()
    elif tracker_type == "KCF":
        return cv2.TrackerKCF_create()
    elif tracker_type == "MOSSE":
        return cv2.legacy.TrackerMOSSE_create()
    return cv2.TrackerKCF_create()

# --- KALMAN FILTER (tuned for steady drift prediction) ---
# Lower process noise = trusts velocity prediction more (good for smooth slide drift)
# Higher measurement noise = less reactive to tracker jitter
kalman = cv2.KalmanFilter(4, 2)
kalman.measurementMatrix = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0]], np.float32)
kalman.transitionMatrix = np.array([
    [1, 0, 1, 0],
    [0, 1, 0, 1],
    [0, 0, 1, 0],
    [0, 0, 0, 1]], np.float32)
kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 5e-3      # Low = trust prediction
kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 5e-1   # High = smooth out jitter

def reset_kalman(cx, cy):
    kalman.statePre = np.array([[cx], [cy], [0], [0]], np.float32)
    kalman.statePost = np.array([[cx], [cy], [0], [0]], np.float32)
    kalman.errorCovPre = np.eye(4, dtype=np.float32)
    kalman.errorCovPost = np.eye(4, dtype=np.float32)

# --- SCALE ESTIMATOR ---
def estimate_best_scale(frame_gray, cx, cy, base_w, base_h, template):
    fh, fw = frame_gray.shape[:2]
    best_score, best_scale = -1, 1.0
    compare_size = (64, 64)
    tmpl_resized = cv2.resize(template, compare_size)

    for s in SCALE_CANDIDATES:
        test_w, test_h = int(base_w * s), int(base_h * s)
        if test_w < MIN_BOX_SIZE or test_h < MIN_BOX_SIZE:
            continue
        if test_w > fw * MAX_BOX_RATIO or test_h > fh * MAX_BOX_RATIO:
            continue
        x1, y1 = max(0, int(cx - test_w / 2)), max(0, int(cy - test_h / 2))
        x2, y2 = min(fw, x1 + test_w), min(fh, y1 + test_h)
        if (x2 - x1) < MIN_BOX_SIZE or (y2 - y1) < MIN_BOX_SIZE:
            continue
        roi = cv2.resize(frame_gray[y1:y2, x1:x2], compare_size)
        score = cv2.matchTemplate(roi, tmpl_resized, cv2.TM_CCOEFF_NORMED)[0][0]
        if score > best_score:
            best_score, best_scale = score, s
    return best_scale, best_score


# --- MOUSE CALLBACK ---
drawing = False
roi_start = (0, 0)
roi_end = (0, 0)
roi_ready = False
selecting_roi = False

def mouse_callback(event, x, y, flags, param):
    global drawing, roi_start, roi_end, roi_ready, selecting_roi
    if not selecting_roi:
        return
    full_x, full_y = int(x * DISPLAY_SCALE_X), int(y * DISPLAY_SCALE_Y)
    if event == cv2.EVENT_LBUTTONDOWN:
        drawing, roi_ready = True, False
        roi_start = roi_end = (full_x, full_y)
    elif event == cv2.EVENT_MOUSEMOVE and drawing:
        roi_end = (full_x, full_y)
    elif event == cv2.EVENT_LBUTTONUP:
        drawing, roi_ready = False, True
        roi_end = (full_x, full_y)


# --- DRAWING HELPERS ---
HUD_FONT = cv2.FONT_HERSHEY_SIMPLEX
HUD_COLOR = (255, 255, 255)
HUD_ACCENT = (0, 255, 255)
HUD_GREEN = (50, 255, 50)
HUD_RED = (80, 80, 255)
HUD_WARN = (0, 165, 255)
HUD_LOCKED = (255, 200, 50)

def draw_outlined_text(img, text, pos, font_scale, color, thickness):
    cv2.putText(img, text, pos, HUD_FONT, font_scale, (0, 0, 0), thickness + 4)
    cv2.putText(img, text, pos, HUD_FONT, font_scale, color, thickness)

def draw_crosshair(img, cx, cy, color, size=8, thickness=1):
    cx, cy = int(cx), int(cy)
    cv2.line(img, (cx - size, cy), (cx + size, cy), color, thickness)
    cv2.line(img, (cx, cy - size), (cx, cy + size), color, thickness)

def draw_corner_brackets(img, x, y, w, h, color, length=25, thickness=3):
    for p1, corner, p2 in [
        ((x, y + length), (x, y), (x + length, y)),
        ((x + w - length, y), (x + w, y), (x + w, y + length)),
        ((x, y + h - length), (x, y + h), (x + length, y + h)),
        ((x + w - length, y + h), (x + w, y + h), (x + w, y + h - length)),
    ]:
        cv2.line(img, p1, corner, color, thickness)
        cv2.line(img, corner, p2, color, thickness)

def draw_hud_panel(img, x, y, w, h, alpha=0.6):
    overlay = img.copy()
    cv2.rectangle(overlay, (x, y), (x + w, y + h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)


# ===================== MAIN =====================

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURE_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_H)

window_name = "AXI6 Orbit Tracker Test"
cv2.namedWindow(window_name)
cv2.setMouseCallback(window_name, mouse_callback)

# --- INITIALIZE CONTROLLERS ---
pan_pid = StepperPID(PAN_KP, PAN_KI, PAN_KD, MAX_MOTOR_SPEED)
tilt_pid = StepperPID(TILT_KP, TILT_KI, TILT_KD, MAX_MOTOR_SPEED)
pan_smoother = MotorOutputSmoother(MOTOR_SMOOTHING, MAX_SPEED_CHANGE)
tilt_smoother = MotorOutputSmoother(MOTOR_SMOOTHING, MAX_SPEED_CHANGE)
slide_sim = TrapezoidalSlideSimulator(SIM_SLIDE_DISTANCE, SIM_SLIDE_MAX_SPEED, SIM_SLIDE_ACCEL)

tracker_name = TRACKER_TYPES[current_tracker_idx]
print("\n--- AXI6 ORBIT TRACKER TEST (v5) ---")
print(f"Tracker: {tracker_name} | Deadzone: {DEADZONE_X}x{DEADZONE_Y}px (tight)")
print("Press 'R' to select target, 'G' to simulate slide drift, 'Q' to quit.\n")

# Motor output state (what would be sent to Pi)
motor_pan_speed, motor_pan_dir = 0, 0
motor_tilt_speed, motor_tilt_dir = 0, 0

while cap.isOpened():
    loop_start = time.time()
    success, frame = cap.read()
    if not success:
        break
    frame = cv2.flip(frame, 1)
    frame_h, frame_w = frame.shape[:2]
    frame_cx, frame_cy = frame_w // 2, frame_h // 2

    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    small_w, small_h = int(frame_w * TRACK_SCALE), int(frame_h * TRACK_SCALE)
    frame_small = cv2.resize(frame, (small_w, small_h))
    inv_scale = 1.0 / TRACK_SCALE
    frame_counter += 1

    # --- SIMULATED SLIDE DRIFT ---
    # Shifts the "target center" that tracking aims for, simulating
    # how the subject appears to drift as the slide moves.
    sim_offset = slide_sim.update() if slide_sim.active else 0
    effective_center_x = frame_cx + int(sim_offset)

    # --- ROI SELECTION MODE ---
    if selecting_roi:
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (frame_w, frame_h), (0, 0, 0), -1)
        frame = cv2.addWeighted(overlay, 0.5, frame, 0.5, 0)
        draw_outlined_text(frame, "DRAW A BOX AROUND YOUR TARGET",
                           (frame_cx - 450, 80), 1.4, HUD_ACCENT, 3)
        draw_outlined_text(frame, "Click & drag, then release. Press 'C' to cancel.",
                           (frame_cx - 400, 130), 0.8, HUD_COLOR, 2)
        if drawing:
            cv2.rectangle(frame, roi_start, roi_end, HUD_ACCENT, 3)

        if roi_ready:
            x1, y1 = roi_start
            x2, y2 = roi_end
            rx, ry = min(x1, x2), min(y1, y2)
            rw, rh = abs(x2 - x1), abs(y2 - y1)
            if rw > 10 and rh > 10:
                gy1, gy2 = max(0, ry), min(frame_h, ry + rh)
                gx1, gx2 = max(0, rx), min(frame_w, rx + rw)
                initial_template = frame_gray[gy1:gy2, gx1:gx2].copy()
                original_box_w, original_box_h = rw, rh
                current_scale = 1.0

                tracker_name = TRACKER_TYPES[current_tracker_idx]
                tracker = create_tracker(tracker_name)
                tracker.init(frame_small, (int(rx * TRACK_SCALE), int(ry * TRACK_SCALE),
                                           int(rw * TRACK_SCALE), int(rh * TRACK_SCALE)))
                tracking_active = True
                track_box = (rx, ry, rw, rh)
                reset_kalman(rx + rw / 2, ry + rh / 2)
                pan_pid.reset(); tilt_pid.reset()
                pan_smoother.reset(); tilt_smoother.reset()
                pan_recentering = False; tilt_recentering = False
                print(f"[TRACKER] {tracker_name} initialized on ROI: {rw}x{rh}")
            else:
                print("[TRACKER] ROI too small.")
            selecting_roi = False
            roi_ready = False

        view = cv2.resize(frame, (DISPLAY_W, DISPLAY_H))
        cv2.imshow(window_name, view)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('c'):
            selecting_roi, roi_ready = False, False
        elif key == ord('q'):
            break
        continue

    # --- TRACKING UPDATE ---
    raw_cx, raw_cy = 0.0, 0.0
    track_ok = False

    if tracking_active and tracker is not None:
        t0 = time.time()
        track_ok, bbox = tracker.update(frame_small)
        dt = time.time() - t0
        tracker_fps = 1.0 / dt if dt > 0 else 0

        if track_ok:
            sbx, sby, sbw, sbh = bbox
            bx, by = int(sbx * inv_scale), int(sby * inv_scale)
            bw, bh = int(sbw * inv_scale), int(sbh * inv_scale)
            raw_cx, raw_cy = bx + bw / 2, by + bh / 2

            # --- SCALE ADAPTATION ---
            if (scale_adapt_enabled and initial_template is not None
                    and frame_counter % SCALE_CHECK_INTERVAL == 0):
                best_s, best_score = estimate_best_scale(
                    frame_gray, raw_cx, raw_cy, bw, bh, initial_template)
                if best_score > 0.3 and best_s != 1.0:
                    current_scale += SCALE_SMOOTHING * (best_s - 1.0)
                    current_scale = max(0.4, min(current_scale, 3.0))
                    new_w = int(original_box_w * current_scale)
                    new_h = int(original_box_h * current_scale)
                    new_x = max(0, min(int(raw_cx - new_w / 2), frame_w - new_w))
                    new_y = max(0, min(int(raw_cy - new_h / 2), frame_h - new_h))
                    if new_w > MIN_BOX_SIZE and new_h > MIN_BOX_SIZE:
                        tracker_name = TRACKER_TYPES[current_tracker_idx]
                        tracker = create_tracker(tracker_name)
                        tracker.init(frame_small, (int(new_x * TRACK_SCALE), int(new_y * TRACK_SCALE),
                                                   int(new_w * TRACK_SCALE), int(new_h * TRACK_SCALE)))
                        bx, by, bw, bh = new_x, new_y, new_w, new_h
                        raw_cx, raw_cy = bx + bw / 2, by + bh / 2

            track_box = (bx, by, bw, bh)

            # --- KALMAN FILTER ---
            if kalman_enabled:
                kalman.correct(np.array([[np.float32(raw_cx)], [np.float32(raw_cy)]]))
                prediction = kalman.predict()
                smooth_cx = float(prediction[0].item())
                smooth_cy = float(prediction[1].item())
            else:
                smooth_cx, smooth_cy = raw_cx, raw_cy

            # Offset from effective center (accounts for sim slide drift)
            offset_x = smooth_cx - effective_center_x
            offset_y = smooth_cy - frame_cy

            # --- DEADZONE WITH HYSTERESIS ---
            # Outer DZ: start correcting. Inner DZ: stop correcting.
            inner_dz_x = int(DEADZONE_X * INNER_DZ_RATIO)
            inner_dz_y = int(DEADZONE_Y * INNER_DZ_RATIO)

            if not pan_recentering and abs(offset_x) > DEADZONE_X:
                pan_recentering = True
            if pan_recentering and abs(offset_x) <= inner_dz_x:
                pan_recentering = False

            if not tilt_recentering and abs(offset_y) > DEADZONE_Y:
                tilt_recentering = True
            if tilt_recentering and abs(offset_y) <= inner_dz_y:
                tilt_recentering = False

            active_dz_x = inner_dz_x if pan_recentering else DEADZONE_X
            active_dz_y = inner_dz_y if tilt_recentering else DEADZONE_Y

            # --- PID → RATE-LIMITED MOTOR OUTPUT ---
            if abs(offset_x) > active_dz_x:
                raw_speed, raw_dir = pan_pid.compute(offset_x)
                motor_pan_speed = pan_smoother.update(raw_speed)
                motor_pan_dir = raw_dir
            else:
                motor_pan_speed = pan_smoother.update(0)
                motor_pan_dir = 0
                pan_pid.reset()

            if abs(offset_y) > active_dz_y:
                raw_speed, raw_dir = tilt_pid.compute(offset_y)
                motor_tilt_speed = tilt_smoother.update(raw_speed)
                motor_tilt_dir = raw_dir
            else:
                motor_tilt_speed = tilt_smoother.update(0)
                motor_tilt_dir = 0
                tilt_pid.reset()

            in_deadzone = not pan_recentering and not tilt_recentering
            box_color = HUD_LOCKED if in_deadzone else HUD_GREEN

            # --- DRAW TARGET ---
            draw_corner_brackets(frame, bx, by, bw, bh, box_color, length=25, thickness=3)
            draw_crosshair(frame, raw_cx, raw_cy, HUD_RED, size=6, thickness=1)
            draw_crosshair(frame, smooth_cx, smooth_cy, box_color, size=14, thickness=3)
            cv2.line(frame, (effective_center_x, frame_cy),
                     (int(smooth_cx), int(smooth_cy)), box_color, 2)

            if scale_adapt_enabled:
                draw_outlined_text(frame, f"{int(current_scale * 100)}%",
                                   (bx, by - 10), 0.6, HUD_ACCENT, 2)

            # --- MOTOR OUTPUT BARS (visual speed meters) ---
            bar_x = 30
            bar_w = 12
            max_bar_h = 150

            # Pan speed bar (horizontal concept shown vertically on left)
            pan_bar_h = int((motor_pan_speed / MAX_MOTOR_SPEED) * max_bar_h)
            pan_bar_col = HUD_GREEN if motor_pan_speed > 0 else (80, 80, 80)
            bar_y_base = frame_h // 2 + max_bar_h // 2
            cv2.rectangle(frame, (bar_x, bar_y_base - pan_bar_h), (bar_x + bar_w, bar_y_base), pan_bar_col, -1)
            cv2.rectangle(frame, (bar_x, bar_y_base - max_bar_h), (bar_x + bar_w, bar_y_base), (80, 80, 80), 1)
            dir_sym = ">" if motor_pan_dir > 0 else "<" if motor_pan_dir < 0 else "-"
            draw_outlined_text(frame, f"PAN {dir_sym} {motor_pan_speed}",
                               (bar_x + bar_w + 8, bar_y_base), 0.65, pan_bar_col, 2)

            # Tilt speed bar
            tilt_bar_h = int((motor_tilt_speed / MAX_MOTOR_SPEED) * max_bar_h)
            tilt_bar_col = HUD_WARN if motor_tilt_speed > 0 else (80, 80, 80)
            bar_x2 = bar_x + bar_w + 200
            cv2.rectangle(frame, (bar_x2, bar_y_base - tilt_bar_h), (bar_x2 + bar_w, bar_y_base), tilt_bar_col, -1)
            cv2.rectangle(frame, (bar_x2, bar_y_base - max_bar_h), (bar_x2 + bar_w, bar_y_base), (80, 80, 80), 1)
            dir_sym_t = "v" if motor_tilt_dir > 0 else "^" if motor_tilt_dir < 0 else "-"
            draw_outlined_text(frame, f"TILT {dir_sym_t} {motor_tilt_speed}",
                               (bar_x2 + bar_w + 8, bar_y_base), 0.65, tilt_bar_col, 2)

        else:
            tracking_active = False
            tracker = None
            initial_template = None
            motor_pan_speed = motor_tilt_speed = 0
            motor_pan_dir = motor_tilt_dir = 0
            pan_smoother.reset(); tilt_smoother.reset()
            print("[TRACKER] Target lost!")

    # --- DRAW DEADZONES ---
    if show_deadzones and tracking_active:
        # Outer deadzone (start correcting)
        outer_col = (100, 100, 100)
        cv2.line(frame, (effective_center_x - DEADZONE_X, 0),
                 (effective_center_x - DEADZONE_X, frame_h), outer_col, 1)
        cv2.line(frame, (effective_center_x + DEADZONE_X, 0),
                 (effective_center_x + DEADZONE_X, frame_h), outer_col, 1)
        cv2.line(frame, (0, frame_cy - DEADZONE_Y),
                 (frame_w, frame_cy - DEADZONE_Y), outer_col, 1)
        cv2.line(frame, (0, frame_cy + DEADZONE_Y),
                 (frame_w, frame_cy + DEADZONE_Y), outer_col, 1)

        # Inner deadzone (stop correcting) — brighter when recentering
        if pan_recentering or tilt_recentering:
            inner_col = (0, 0, 180)
            inner_dz_x = int(DEADZONE_X * INNER_DZ_RATIO)
            inner_dz_y = int(DEADZONE_Y * INNER_DZ_RATIO)
            cv2.line(frame, (effective_center_x - inner_dz_x, frame_cy - 80),
                     (effective_center_x - inner_dz_x, frame_cy + 80), inner_col, 1)
            cv2.line(frame, (effective_center_x + inner_dz_x, frame_cy - 80),
                     (effective_center_x + inner_dz_x, frame_cy + 80), inner_col, 1)
            cv2.line(frame, (effective_center_x - 80, frame_cy - inner_dz_y),
                     (effective_center_x + 80, frame_cy - inner_dz_y), inner_col, 1)
            cv2.line(frame, (effective_center_x - 80, frame_cy + inner_dz_y),
                     (effective_center_x + 80, frame_cy + inner_dz_y), inner_col, 1)

    # --- CENTER CROSSHAIR (moves with simulated slide) ---
    draw_crosshair(frame, effective_center_x, frame_cy, (220, 220, 220), size=18, thickness=2)

    # --- SIMULATED SLIDE PROGRESS BAR ---
    if slide_sim.active or slide_sim.phase == "done":
        progress = min(slide_sim.position / slide_sim.distance, 1.0)
        bar_total_w = 400
        bar_h = 10
        bx_start = frame_cx - bar_total_w // 2
        by_start = frame_h - 100
        cv2.rectangle(frame, (bx_start, by_start), (bx_start + bar_total_w, by_start + bar_h), (80, 80, 80), -1)
        cv2.rectangle(frame, (bx_start, by_start), (bx_start + int(bar_total_w * progress), by_start + bar_h), HUD_ACCENT, -1)
        draw_outlined_text(frame, f"SLIDE: {slide_sim.phase.upper()} ({int(progress * 100)}%)",
                           (bx_start, by_start - 10), 0.65, HUD_ACCENT, 2)

    # --- HUD ---
    loop_dt = time.time() - loop_start
    overall_fps = 0.9 * overall_fps + 0.1 * (1.0 / loop_dt if loop_dt > 0 else 0)
    tracker_name = TRACKER_TYPES[current_tracker_idx]

    if tracking_active and track_ok:
        mode_text = "ORBIT TRACKING"
        mode_color = HUD_GREEN
    elif not tracking_active:
        mode_text = "IDLE  [R] SELECT TARGET"
        mode_color = HUD_ACCENT
    else:
        mode_text = "TARGET LOST"
        mode_color = HUD_RED

    draw_hud_panel(frame, 0, 0, 800, 200, alpha=0.55)
    draw_outlined_text(frame, mode_text, (20, 55), 1.3, mode_color, 3)
    draw_outlined_text(frame, f"{tracker_name}  |  Tracker: {tracker_fps:.0f}  |  Loop: {overall_fps:.0f} FPS",
                       (20, 100), 0.8, HUD_COLOR, 2)
    draw_outlined_text(frame, f"Kalman: {'ON' if kalman_enabled else 'OFF'}  |  Scale: {'ON' if scale_adapt_enabled else 'OFF'} ({int(current_scale * 100)}%)"
                       f"  |  DZ: {DEADZONE_X}x{DEADZONE_Y}",
                       (20, 140), 0.65, HUD_GREEN if kalman_enabled else HUD_RED, 2)
    slide_status = f"SIM: {slide_sim.phase.upper()}" if slide_sim.active else "SIM: OFF [G] to start"
    draw_outlined_text(frame, slide_status, (20, 175), 0.65, HUD_ACCENT if slide_sim.active else (150, 150, 150), 2)

    # Controls bar
    draw_hud_panel(frame, 0, frame_h - 55, frame_w, 55, alpha=0.55)
    draw_outlined_text(frame,
        "R: Select  C: Cancel  G: Sim Slide  K: Kalman  A: Scale  D: DZ  1/2/3: Tracker  Q: Quit",
        (20, frame_h - 20), 0.55, HUD_COLOR, 2)

    # --- DISPLAY ---
    view = cv2.resize(frame, (DISPLAY_W, DISPLAY_H))
    cv2.imshow(window_name, view)

    # --- KEYBOARD CONTROLS ---
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('r'):
        selecting_roi, drawing, roi_ready = True, False, False
        tracking_active, tracker, initial_template = False, None, None
        current_scale = 1.0
        slide_sim.reset()
    elif key == ord('c'):
        tracking_active, tracker, initial_template = False, None, None
        current_scale = 1.0
        pan_smoother.reset(); tilt_smoother.reset()
        slide_sim.reset()
        print("[TRACKER] Cancelled.")
    elif key == ord('k'):
        kalman_enabled = not kalman_enabled
        print(f"[KALMAN] {'ON' if kalman_enabled else 'OFF'}")
    elif key == ord('a'):
        scale_adapt_enabled = not scale_adapt_enabled
        if not scale_adapt_enabled:
            current_scale = 1.0
        print(f"[SCALE] {'ON' if scale_adapt_enabled else 'OFF'}")
    elif key == ord('d'):
        show_deadzones = not show_deadzones
    elif key == ord('g'):
        if slide_sim.active:
            slide_sim.reset()
            print("[SIM] Slide stopped.")
        else:
            slide_sim.start()
            print("[SIM] Trapezoidal slide drift started!")
    elif key == ord('=') or key == ord('+'):
        DEADZONE_X = min(DEADZONE_X + 5, frame_w // 2)
        DEADZONE_Y = min(DEADZONE_Y + 5, frame_h // 2)
    elif key == ord('-'):
        DEADZONE_X = max(DEADZONE_X - 5, 5)
        DEADZONE_Y = max(DEADZONE_Y - 5, 5)
    elif key in [ord('1'), ord('2'), ord('3')]:
        new_idx = key - ord('1')
        if new_idx != current_tracker_idx:
            current_tracker_idx = new_idx
            new_name = TRACKER_TYPES[current_tracker_idx]
            print(f"[TRACKER] Switched to {new_name}.")
            if tracking_active and track_box is not None:
                bx, by, bw, bh = track_box
                tracker = create_tracker(new_name)
                tracker.init(frame_small, (int(bx * TRACK_SCALE), int(by * TRACK_SCALE),
                                           int(bw * TRACK_SCALE), int(bh * TRACK_SCALE)))
                reset_kalman(bx + bw / 2, by + bh / 2)
                print(f"[TRACKER] {new_name} re-initialized.")

    # --- FRAME RATE CAP (60Hz) ---
    elapsed = time.time() - loop_start
    sleep_time = FRAME_TIME - elapsed
    if sleep_time > 0:
        time.sleep(sleep_time)

cap.release()
cv2.destroyAllWindows()
print("Orbit Tracker Test Offline.")
