import cv2
import time
import threading
from flask import Flask, Response
from tmc_driver import (
    Tmc2209, Loglevel, MovementAbsRel, TmcEnableControlPin, TmcMotionControlStepDir,
)

# --- 1. GLOBAL STATE & FILTER SETUP ---
WIDTH, HEIGHT = 320, 240
CENTER_X = WIDTH // 2
DEADZONE = 25 

class RobotState:
    def __init__(self):
        self.target_cx = CENTER_X 
        self.ghost_cx = CENTER_X   
        self.target_visible = False
        self.running = True
        
        # Moving Average Filter variables
        self.history_cx = []
        self.filter_size = 4  # Averages the last 4 frames. Increase to 6 for even more smoothing.

state = RobotState()

# --- 2. MOTOR THREAD (500Hz Direct Input) ---
def motor_loop():
    tmc = Tmc2209(TmcEnableControlPin(21), TmcMotionControlStepDir(16, 20), loglevel=Loglevel.INFO)
    tmc.acceleration_fullstep = 400 
    tmc.set_motor_enabled(True)
    
    # PID Tuning
    kp, ki, kd = 0.75, 0.01, 1.5
    
    # Master Speed Dial (0.25 = 25% of the raw PID velocity)
    GLOBAL_SPEED_SCALE = 1
    
    prev_error = 0
    integral = 0
    last_time = time.time() 
    print_counter = 0
    
    print("\n[THREAD] 500Hz Cinematic Direct-Drive Active")
    print("-" * 50)
    
    while state.running:
        current_time = time.time()
        dt = current_time - last_time
        if dt <= 0: dt = 0.001 
        
        if state.target_visible:
            # Calculate PID directly on the perfectly synced Ghost input
            motor_error = state.ghost_cx - CENTER_X
            
            if abs(motor_error) > DEADZONE:
                integral += motor_error * dt
                integral = max(min(integral, 1000), -1000) # Anti-windup
                
                derivative = (motor_error - prev_error) / dt
                
                raw_velocity = (kp * motor_error) + (ki * integral) + (kd * derivative)
                
                # Proportional Slowdown
                scaled_velocity = abs(raw_velocity) * GLOBAL_SPEED_SCALE
                
                # Cap speed for safety and cinematic crawl
                final_speed = min(int(scaled_velocity), 200)
                tmc.max_speed_fullstep = final_speed
                
                # Constant micro-steps for fluid motion
                direction = 10 if motor_error > 0 else -10
                tmc.run_to_position_steps(direction, MovementAbsRel.RELATIVE)
                
                prev_error = motor_error
                
                if print_counter % 50 == 0:  
                    print(f"🏃 MOVE | Raw Vel: {raw_velocity:6.1f} | Scaled Spd: {final_speed} | Dir: {direction}")
            else:
                tmc.max_speed_fullstep = 0
                integral = 0 
                
                if print_counter % 100 == 0:
                    print(f"🎯 DEADZONE | Target Locked.")
        else:
            tmc.max_speed_fullstep = 0
            
            if print_counter % 100 == 0:
                print(f"🙈 LOST | Waiting for face detection...")
            
        print_counter += 1
        last_time = current_time
            
        # 500Hz Loop
        sleep_time = 0.002 - (time.time() - current_time)
        if sleep_time > 0:
            time.sleep(sleep_time)
            
    tmc.set_motor_enabled(False)
    print("\n[SHUTDOWN] Motor thread safely disabled.")

# --- 3. VISION & SERVER ---
app = Flask(__name__)
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

def generate_frames():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    # Manual exposure (Adjust if too dark/bright)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) 
    cap.set(cv2.CAP_PROP_EXPOSURE, 400)
    
    try:
        while state.running:
            success, frame = cap.read()
            if not success or frame is None: 
                time.sleep(0.01)
                continue
                
            frame = cv2.resize(frame, (WIDTH, HEIGHT))
            frame = cv2.flip(frame, 1)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            faces = face_cascade.detectMultiScale(gray, 1.3, 5)
            
            if len(faces) > 0:
                (x, y, w, h) = faces[0]
                raw_cx = x + (w // 2)
                
                # --- APPLY MOVING AVERAGE FILTER ---
                state.history_cx.append(raw_cx)
                if len(state.history_cx) > state.filter_size:
                    state.history_cx.pop(0)
                
                smooth_cx = sum(state.history_cx) / len(state.history_cx)
                
                # --- THE FIX: INSTANT SYNC ---
                # Update Target and Ghost identically before drawing
                state.target_cx = smooth_cx
                state.ghost_cx = smooth_cx
                state.target_visible = True
                
                # Visuals: Box and dot are now perfectly locked together
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.circle(frame, (int(state.ghost_cx), y + (h // 2)), 6, (0, 0, 255), -1) 
            else:
                state.target_visible = False
                state.history_cx.clear() # Dump the filter history so we don't start with old data next time

            # Draw HUD
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