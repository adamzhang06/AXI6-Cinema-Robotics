import cv2
from flask import Flask, Response

app = Flask(__name__)

# --- TRACKING CONSTANTS ---
WIDTH = 320
HEIGHT = 240
CENTER_X = WIDTH // 2
DEADZONE = 30  # Adjust this to make the "Safe Zone" wider or narrower

class MotorController:
    def __init__(self):
        print("[INIT] Motor Logic Ready")

    def update_position(self, face_cx):
        error = face_cx - CENTER_X
        if abs(error) > DEADZONE:
            direction = "RIGHT (CW)" if error > 0 else "LEFT (CCW)"
            # This is where your motor.step() logic will eventually go
            print(f"MOTOR COMMAND: Move {direction} | Error: {error}")
        else:
            # Target is inside the yellow lines
            pass

controller = MotorController()
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

def generate_frames():
    cap = cv2.VideoCapture(0)
    
    # Standard Brio 4:3 aspect ratio settings
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    while True:
        success, frame = cap.read()
        if not success: break
            
        # Downsample to your requested 320x240
        frame = cv2.resize(frame, (WIDTH, HEIGHT))
        frame = cv2.flip(frame, 1)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # --- DRAW THE DEADZONE OVERLAY ---
        # Left boundary of deadzone
        cv2.line(frame, (CENTER_X - DEADZONE, 0), (CENTER_X - DEADZONE, HEIGHT), (0, 255, 255), 1)
        # Right boundary of deadzone
        cv2.line(frame, (CENTER_X + DEADZONE, 0), (CENTER_X + DEADZONE, HEIGHT), (0, 255, 255), 1)
        # Center target line
        cv2.line(frame, (CENTER_X, 0), (CENTER_X, HEIGHT), (100, 100, 100), 1)

        # Detect Faces
        faces = face_cascade.detectMultiScale(gray, 1.3, 5)
        
        if len(faces) > 0:
            (x, y, w, h) = faces[0]
            cx = x + (w // 2)
            
            # Send to motor logic
            controller.update_position(cx)

            # Draw Face HUD
            # If outside deadzone, box is Green. If inside, box is Blue.
            color = (0, 255, 0) if abs(cx - CENTER_X) > DEADZONE else (255, 255, 0)
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            cv2.circle(frame, (cx, y + (h // 2)), 4, (0, 0, 255), -1)
            
            status = "TRACKING" if color == (0, 255, 0) else "CENTERED"
            cv2.putText(frame, status, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        # Encode for Browser
        ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

    cap.release()

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return '''
    <body style="margin:0; background:#000; display:flex; justify-content:center; align-items:center; height:100vh; overflow:hidden;">
        <img src="/video_feed" style="width:100vw; height:100vh; object-fit:contain;">
    </body>
    '''

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000, threaded=True)