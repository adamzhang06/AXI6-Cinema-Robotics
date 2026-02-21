import cv2
import time
import threading
from flask import Flask, Response
from tmc_driver import (
    Tmc2209, Loglevel, MovementAbsRel, TmcEnableControlPin, TmcMotionControlStepDir,
)

# --- 1. GLOBAL STATE ---
WIDTH, HEIGHT = 320, 240
CENTER_X = WIDTH // 2
DEADZONE = 25 # Tightened for more professional framing

class RobotState:
    def __init__(self):
        self.target_cx = CENTER_X 
        self.ghost_cx = CENTER_X   
        self.target_visible = False
        self.running = True

state = RobotState()

# --- 2. MOTOR THREAD (Cinematic Glide Logic) ---
def motor_loop():
    tmc = Tmc2209(TmcEnableControlPin(21), TmcMotionControlStepDir(16, 20), loglevel=Loglevel.INFO)
    
    # Low acceleration creates the "Slow Start" cinematic look
    tmc.acceleration_fullstep = 800 
    tmc.set_motor_enabled(True)
    
    # PID Tuning for "Smoothness"
    # kp: Low value for slow, graceful reaction
    # kd: High value to 'brake' and prevent overshoot
    kp, ki, kd = 3.5, 0.01, 0.6
    
    prev_error = 0
    integral = 0
    
    print("[THREAD] Cinematic Glide Loop Active")
    
    while state.running:
        start_time = time.time()
        
        if state.target_visible:
            # 1. Update Ghost (The smoothed target)
            error = state.target_cx - state.ghost_cx
            state.ghost_cx += error * 0.15 # Slow follow for "weighty" feel
            
            # 2. Calculate PID for Velocity
            motor_error = state.ghost_cx - CENTER_X
            
            if abs(motor_error) > DEADZONE:
                # Basic PID math for velocity
                dt = 0.01 
                integral += motor_error * dt
                derivative = (motor_error - prev_error) / dt
                
                velocity = (kp * motor_error) + (ki * integral) + (kd * derivative)
                
                # 3. Apply Speed & Constant Step Size
                # Max speed capped at 500 for cinematic safety
                tmc.max_speed_fullstep = min(int(abs(velocity)), 500)
                
                # CONSTANT STEP SIZE: 10 steps
                direction = 10 if motor_error > 0 else -10
                tmc.run_to_position_steps(direction, MovementAbsRel.RELATIVE)
                
                prev_error = motor_error
            else:
                tmc.max_speed_fullstep = 0
                integral = 0 # Reset integral to prevent windup in deadzone
        else:
            # FREEZE: Stop immediately if target is lost
            tmc.max_speed_fullstep = 0
            
        # 100Hz frequency
        sleep_time = 0.01 - (time.time() - start_time)
        if sleep_time > 0:
            time.sleep(sleep_time)
            
    tmc.set_motor_enabled(False)

# --- 3. VISION & SERVER ---
app = Flask(__name__)
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

def generate_frames():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    try:
        while state.running:
            success, frame = cap.read()
            if not success: break
            frame = cv2.resize(frame, (WIDTH, HEIGHT))
            frame = cv2.flip(frame, 1)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            faces = face_cascade.detectMultiScale(gray, 1.3, 5)
            
            if len(faces) > 0:
                (x, y, w, h) = faces[0]
                state.target_cx = x + (w // 2)
                state.target_visible = True
                
                # Visuals: Target (Green), Ghost (Red), Deadzone (Yellow)
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.circle(frame, (int(state.ghost_cx), y + (h // 2)), 6, (0, 0, 255), 2)
            else:
                state.target_visible = False

            # HUD
            cv2.line(frame, (CENTER_X - DEADZONE, 0), (CENTER_X - DEADZONE, HEIGHT), (0, 255, 255), 1)
            cv2.line(frame, (CENTER_X + DEADZONE, 0), (CENTER_X + DEADZONE, HEIGHT), (0, 255, 255), 1)

            ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
            yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
    finally:
        cap.release()

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return '<body style="margin:0; background:#000;"><img src="/video_feed" style="width:100vw; height:100vh; object-fit:contain;"></body>'

if __name__ == "__main__":
    m_thread = threading.Thread(target=motor_loop, daemon=True)
    m_thread.start()
    try:
        app.run(host='0.0.0.0', port=5000, threaded=True, debug=False)
    except KeyboardInterrupt:
        state.running = False