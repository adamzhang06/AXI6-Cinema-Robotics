import cv2
import mediapipe as mp
from flask import Flask, Response

app = Flask(__name__)

# --- 1. INITIALIZE MEDIAPIPE ---
# The rpi3 version uses the same API structure
mp_face_detection = mp.solutions.face_detection
# model_selection=0 is optimized for faces within 2 meters
face_detector = mp_face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.5)

def generate_frames():
    print("[INFO] Starting Brio 101 on Pi 3B (32-bit)...")
    cap = cv2.VideoCapture(0)
    
    # Standard resolution for Pi 3B processing speed
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    while True:
        success, frame = cap.read()
        if not success:
            break
            
        frame = cv2.flip(frame, 1)
        ih, iw, _ = frame.shape

        # --- 2. AI INFERENCE ---
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = face_detector.process(frame_rgb)
        
        # --- 3. DRAW TARGETING HUD ---
        if results.detections:
            for detection in results.detections:
                bboxC = detection.location_data.relative_bounding_box
                x, y, w, h = int(bboxC.xmin * iw), int(bboxC.ymin * ih), \
                             int(bboxC.width * iw), int(bboxC.height * ih)
                
                # Calculate Center of Target
                cx, cy = x + (w // 2), y + (h // 2)

                # Draw Bounding Box (Green)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                # Draw Center Crosshair (Red)
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                
                cv2.putText(frame, "TARGET ACQUIRED", (x, y - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # --- 4. STREAM TO WEB ---
        ret, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

    cap.release()

@app.route('/')
def index():
    # Updated CSS: "object-fit: cover" and 100vw/vh makes it truly full screen
    return '''
    <html>
        <head>
            <title>AXI6 AI Tracker</title>
            <style>
                body { margin: 0; padding: 0; background-color: #000; overflow: hidden; }
                .stream { 
                    width: 100vw; 
                    height: 100vh; 
                    object-fit: cover; 
                }
            </style>
        </head>
        <body>
            <img src="/video_feed" class="stream">
        </body>
    </html>
    '''

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    # Use the IP you found earlier: 100.69.176.89
    app.run(host='0.0.0.0', port=5000, debug=False)