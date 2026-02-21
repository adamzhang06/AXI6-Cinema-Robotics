import cv2
import time
import threading
from flask import Flask, Response
from tmc_driver import (
    Tmc2209, Loglevel, MovementAbsRel, TmcEnableControlPin, TmcMotionControlStepDir,
)

# --- 1. GLOBAL STATE & CONSTANTS ---
WIDTH, HEIGHT = 320, 240
CENTER_X = WIDTH // 2
DEADZONE = 30 

class RobotState:
    def __init__(self):
        self.target_cx = CENTER_X 
        self.ghost_cx = CENTER_X   
        self.running = True

state = RobotState()

# --- 2. MOTOR THREAD (Dynamic Velocity) ---
def motor_loop():
    tmc = Tmc2209(TmcEnableControlPin(21), TmcMotionControlStepDir(16, 20), loglevel=Loglevel.INFO)
    # High acceleration allows for the fast "snaps"
    tmc.acceleration_fullstep = 5000 
    tmc.set_motor_enabled(True)
    
    print("[THREAD] Non-Linear Velocity Loop Active")
    
    while state.running:
        start_time = time.time()
        
        # 1. Update Ghost (The "Leash")
        # Higher multiplier here (0.25) makes the ghost follow the target more tightly
        error = state.target_cx - state.ghost_cx
        state.ghost_cx += error * 0.25 
        
        # 2. Distance from Center
        motor_error = state.ghost_cx - CENTER_X
        abs_error = abs(motor_error)
        
        if abs_error > DEADZONE:
            # --- NON-LINEAR VELOCITY CALCULATION ---
            # We normalize the error (0.0 to 1.0)
            normalized_error = (abs_error - DEADZONE) / (CENTER_X - DEADZONE)
            
            # Use a power function (Exponent of 2.0 or 2.5) to create the curve
            # This makes it crawl when close, and lunge when far.
            velocity_multiplier = pow(normalized_error, 2.2) 
            
            # Map to actual speed (Max 1400, Min 100)
            target_speed = 100 + (velocity_multiplier * 1300)
            
            # 3. Apply to Motor
            tmc.max_speed_fullstep = int(target_speed)
            direction = 15 if motor_error > 0 else -15
            tmc.run_to_position_steps(direction, MovementAbsRel.RELATIVE)
            
            # Debugging speed in terminal
            # print(f"Error: {int(abs_error)} | Speed: {int(target_speed)}")
        else:
            tmc.max_speed_fullstep = 0
            
        # 100Hz frequency
        sleep_time = 0.01 - (time.time() - start_time)
        if sleep_time > 0:
            time.sleep(sleep_time)
            
    tmc.set_motor_enabled(False)

# --- 3. VISION & WEB SERVER (Standard) ---
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
            
            # HUD
            cv2.line(frame, (CENTER_X - DEADZONE, 0), (CENTER_X - DEADZONE, HEIGHT), (0, 255, 255), 1)
            cv2.line(frame, (CENTER_X + DEADZONE, 0), (CENTER_X + DEADZONE, HEIGHT), (0, 255, 255), 1)

            faces = face_cascade.detectMultiScale(gray, 1.2, 5)
            if len(faces) > 0:
                (x, y, w, h) = faces[0]
                state.target_cx = x + (w // 2)
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.circle(frame, (int(state.ghost_cx), y + (h // 2)), 6, (0, 0, 255), 2)

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