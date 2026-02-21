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

# --- 1. PID VELOCITY CONTROLLER ---
class PIDVelocity:
    def __init__(self, kp, ki, kd, center):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.center = center
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()

    def compute(self, current_pos):
        now = time.time()
        dt = now - self.last_time
        if dt <= 0: dt = 0.001
        
        error = current_pos - self.center
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        self.prev_error = error
        self.last_time = now
        return output # This will be used as our Velocity/Speed

# --- 2. SYSTEM MANAGER ---
class AXI6System:
    def __init__(self):
        self.tmc = Tmc2209(
            TmcEnableControlPin(21),
            TmcMotionControlStepDir(16, 20),
            loglevel=Loglevel.INFO,
        )
        # High acceleration for responsiveness, speed will be dynamic
        self.tmc.acceleration_fullstep = 1200 
        self.tmc.set_motor_enabled(True)

        # PID Tuning for Velocity
        # Kp here determines the "Top Speed" based on distance
        self.pid = PIDVelocity(kp=4.5, ki=0.05, kd=0.1, center=160)
        
        self.coord_history = []
        self.filter_size = 5 # More smoothing for velocity control

        print("[SYSTEM] AXI6 Dynamic Velocity Control Online")

    def process_frame(self, raw_cx):
        # Filtering
        self.coord_history.append(raw_cx)
        if len(self.coord_history) > self.filter_size:
            self.coord_history.pop(0)
        smooth_cx = sum(self.coord_history) / len(self.coord_history)

        # Calculate target velocity
        velocity_output = self.pid.compute(smooth_cx)
        
        # Determine Direction and Speed
        speed = abs(velocity_output)
        
        if speed > 10: # Minimum speed threshold to avoid motor stall humming
            # 1. Update the Speed
            # Cap it at 800 for safety on a Pi 3B
            self.tmc.max_speed_fullstep = min(int(speed), 800)
            
            # 2. Command a fixed "Chunk" of steps
            # Moving 5 steps at a time makes the motion fluid as we update speed constantly
            direction = 5 if velocity_output > 0 else -5
            
            print(f"SPEED: {int(speed)} | DIR: {direction}")
            self.tmc.run_to_position_steps(direction, MovementAbsRel.RELATIVE)
        else:
            # Optionally slow down to 0 if centered
            self.tmc.max_speed_fullstep = 0

    def shutdown(self):
        self.tmc.set_motor_enabled(False)
        del self.tmc

# --- 3. VISION & SERVER (Standard) ---
app = Flask(__name__)
axi6 = AXI6System()
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

def generate_frames():
    cap = cv2.VideoCapture(0)
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
            faces = face_cascade.detectMultiScale(gray, 1.2, 5)
            
            if len(faces) > 0:
                (x, y, w, h) = faces[0]
                cx = x + (w // 2)
                axi6.process_frame(cx)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (cx, y + (h // 2)), 4, (0, 0, 255), -1)

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
    try:
        app.run(host='0.0.0.0', port=5000, threaded=True, debug=False)
    except KeyboardInterrupt:
        pass
    finally:
        axi6.shutdown()