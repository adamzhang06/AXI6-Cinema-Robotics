import cv2
import time
from flask import Flask, Response
from tmc_driver import (
    Tmc2209,
    Loglevel,
    MovementAbsRel,
    TmcEnableControlPin,
    TmcMotionControlStepDir,
)

# --- 1. PID CONTROLLER LOGIC ---
class PID:
    def __init__(self, kp, ki, kd, center):
        self.kp = kp  # Proportional: The "Strength" of the move
        self.ki = ki  # Integral: Fixes long-term drift
        self.kd = kd  # Derivative: The "Brake" to prevent overshoot
        self.center = center
        
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()

    def compute(self, current_pos):
        now = time.time()
        dt = now - self.last_time
        if dt <= 0: dt = 0.001 # Prevent division by zero
        
        error = current_pos - self.center
        
        # Proportional term
        P = self.kp * error
        
        # Integral term
        self.integral += error * dt
        I = self.ki * self.integral
        
        # Derivative term (The "Brake")
        derivative = (error - self.prev_error) / dt
        D = self.kd * derivative
        
        self.prev_error = error
        self.last_time = now
        
        return int(P + I + D)

# --- 2. MOTOR & SYSTEM MANAGER ---
class AXI6System:
    def __init__(self):
        # Motor Hardware Setup
        self.tmc = Tmc2209(
            TmcEnableControlPin(21),
            TmcMotionControlStepDir(16, 20),
            loglevel=Loglevel.INFO,
        )
        self.tmc.acceleration_fullstep = 600
        self.tmc.max_speed_fullstep = 500
        self.tmc.set_motor_enabled(True)

        # PID Tuning (Adjust these for the "Cinema" feel)
        # Kp: How fast it reacts.
        # Kd: Increase this if the motor "vibrates" or overshoots.
        self.pid = PID(kp=0.4, ki=0.01, kd=0.05, center=160)
        
        # Smoothing Filter (Moving Average)
        self.coord_history = []
        self.filter_size = 3 

        print("[SYSTEM] AXI6 PID Cinema Controller Online")

    def process_frame(self, raw_cx):
        # 1. Filter the "flicker" out of the detection
        self.coord_history.append(raw_cx)
        if len(self.coord_history) > self.filter_size:
            self.coord_history.pop(0)
        smooth_cx = sum(self.coord_history) / len(self.coord_history)

        # 2. Compute PID Output
        steps_to_move = self.pid.compute(smooth_cx)

        # 3. Apply Movement (Deadzone check included in P-term naturally)
        if abs(steps_to_move) > 2: # Ignore tiny 1-2 step jitters
            print(f"PID OUTPUT: {steps_to_move} steps")
            self.tmc.run_to_position_steps(steps_to_move, MovementAbsRel.RELATIVE)

    def shutdown(self):
        print("\n[SHUTDOWN] Safely disabling motors.")
        self.tmc.set_motor_enabled(False)
        del self.tmc

# --- 3. VISION & WEB SERVER ---
app = Flask(__name__)
axi6 = AXI6System()
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

def generate_frames():
    cap = cv2.VideoCapture(0)
    # Use MJPG for Brio speed on Pi 3B
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    try:
        while True:
            success, frame = cap.read()
            if not success: break
                
            frame = cv2.resize(frame, (320, 240))
            frame = cv2.flip(frame, 1)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Draw HUD
            cv2.line(frame, (160, 0), (160, 240), (50, 50, 50), 1) # Center line

            faces = face_cascade.detectMultiScale(gray, 1.2, 5)
            
            if len(faces) > 0:
                (x, y, w, h) = faces[0]
                cx = x + (w // 2)
                
                # Send to PID System
                axi6.process_frame(cx)

                # Draw Face Box
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (cx, y + (h // 2)), 4, (0, 0, 255), -1)

            ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
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
    try:
        app.run(host='0.0.0.0', port=5000, threaded=True, debug=False)
    except KeyboardInterrupt:
        pass
    finally:
        axi6.shutdown()