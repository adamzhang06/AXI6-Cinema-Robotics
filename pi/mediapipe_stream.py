import cv2
import mediapipe as mp
from flask import Flask, Response

app = Flask(__name__)

# --- 1. INITIALIZE MEDIAPIPE ---
mp_face_detection = mp.solutions.face_detection
# model_selection=0 is for short-range (within 2 meters), ideal for the Brio
face_detector = mp_face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.5)

def generate_frames():
    cap = cv2.VideoCapture(0)
    
    # 320x240 is the "Sweet Spot" for the Pi 3B
    # It's high enough for AI to see you, but low enough to keep FPS steady
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    while True:
        success, frame = cap.read()
        if not success:
            break
            
        frame = cv2.flip(frame, 1)
        ih, iw, _ = frame.shape

        # --- 2. RUN AI INFERENCE ---
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = face_detector.process(frame_rgb)
        
        # --- 3. DRAW FACE BOXES & CROSSHAIRS ---
        if results.detections:
            for detection in results.detections:
                bboxC = detection.location_data.relative_bounding_box
                x, y, w, h = int(bboxC.xmin * iw), int(bboxC.ymin * ih), \
                             int(bboxC.width * iw), int(bboxC.height * ih)
                
                # Center of the face
                face_center_x = x + (w // 2)
                face_center_y = y + (h // 2)

                # Draw Target Box
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                # Draw Center Point
                cv2.circle(frame, (face_center_x, face_center_y), 5, (0, 0, 255), -1)
                
                # Feedback Text
                cv2.putText(frame, "TRACKING FACE", (x, y - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # --- 4. STREAM TO MAC ---
        ret, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

    cap.release()

@app.route('/')
def index():
    return '''
    <html>
        <head>
            <title>AXI6 Face Tracker</title>
            <style>
                body { margin: 0; background-color: #000; display: flex; justify-content: center; align-items: center; height: 100vh; overflow: hidden; }
                img { width: 100vw; height: 100vh; object-fit: contain; image-rendering: pixelated; }
            </style>
        </head>
        <body><img src="/video_feed"></body>
    </html>
    '''

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000, debug=False)