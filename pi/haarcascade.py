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

# --- 1. MOTOR CONTROLLER CLASS ---
class AXI6MotorController:
    def __init__(self):
        # Initiate the Tmc2209 (Pins: EN=21, STEP=16, DIR=20)
        self.tmc = Tmc2209(
            TmcEnableControlPin(21),
            TmcMotionControlStepDir(16, 20),
            loglevel=Loglevel.INFO,
        )
        self.tmc.acceleration_fullstep = 1000
        self.tmc.max_speed_fullstep = 250
        self.tmc.set_motor_enabled(True)

        self.center_x = 160 # 320 / 2
        self.deadzone = 30  # Yellow line boundaries
        self.last_move_time = 0
        self.move_delay = 0.5 # Your requested 0.5s delay
        
        print("[INIT] AXI6 Cinema Robotics Motor System Active")

    def update_tracking(self, face_cx):
        current_time = time.time()
        
        # Check if enough time has passed since the last move
        if current_time - self.last_move_time < self.move_delay:
            return

        error = face_cx - self.center_x

        if abs(error) > self.deadzone:
            # MAP PIXELS TO DEGREES
            # Max error is 160px. Let's map that to a 15-degree "jump" to start.
            # (error / 160) * 15 degrees
            degrees_to_move = (error / 160.0) * 15.0
            
            # Inverse the direction if the motor moves away from you
            # (Remove the minus sign if it moves the wrong way)
            self.rotate_degrees(degrees_to_move) 
            
            self.last_move_time = current_time

    def rotate_degrees(self, degrees):
        # 1600 steps per 360-degree revolution (1.8 deg motor @ 1/8 microstepping?)
        steps = int((degrees / 360.0) * 1600)
        print(f"TRACKING: {degrees:.2f}° | Steps: {steps}")
        self.tmc.run_to_position_steps(steps, MovementAbsRel.RELATIVE)

    def shutdown(self):
        print("\n[SAFE SHUTDOWN] Disabling Motor current...")
        self.tmc.set_motor_enabled(False)
        del self.tmc

# --- 2. VISION & WEB SETUP ---
app = Flask(__name__)
motor_ctrl = AXI6MotorController()
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

def generate_frames():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    try:
        while True:
            success, frame = cap.read()
            if not success: break
                
            frame = cv2.resize(frame, (320, 240))
            frame = cv2.flip(frame, 1)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Draw Deadzone HUD (Yellow lines)
            cv2.line(frame, (160 - 30, 0), (160 - 30, 240), (0, 255, 255), 1)
            cv2.line(frame, (160 + 30, 0), (160 + 30, 240), (0, 255, 255), 1)

            faces = face_cascade.detectMultiScale(gray, 1.3, 5)
            
            if len(faces) > 0:
                (x, y, w, h) = faces[0]
                cx = x + (w // 2)
                
                # EXECUTE MOTOR MOVEMENT
                motor_ctrl.update_tracking(cx)

                # Draw Visuals
                color = (0, 255, 0) if abs(cx - 160) > 30 else (255, 255, 0)
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
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
        print("Server live at http://100.69.176.89:5000")
        app.run(host='0.0.0.0', port=5000, threaded=True, debug=False)
    except KeyboardInterrupt:
        pass
    finally:
        motor_ctrl.shutdown()