import cv2
import os
from flask import Flask, Response
from ultralytics import YOLO

app = Flask(__name__)

# --- 1. LOAD THE YOLO TFLITE MODEL ---
print("[INFO] Loading YOLO Nano TFLite Model...")
script_dir = os.path.dirname(os.path.abspath(__file__))
# Assumes the model is in a 'models' folder one level up, or adjust as needed for your repo structure
model_path = os.path.join(script_dir, 'models', 'yolov8n_float16.tflite') 

# If you get a file not found error, you can temporarily hardcode the exact path here:
# model_path = "/home/adam/AXI6/models/yolov8n_float16.tflite"

model = YOLO(model_path)
print("[INFO] Model Loaded.")

def generate_frames():
    print("[INFO] Initializing Logitech Brio 101...")
    cap = cv2.VideoCapture(0)
    
    # Drop resolution for the Pi 3B's CPU
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    while True:
        success, frame = cap.read()
        if not success:
            break
            
        frame = cv2.flip(frame, 1)

        # --- 2. RUN AI INFERENCE ---
        # classes=[0] ensures it only looks for "person"
        results = model.predict(frame, classes=[0], verbose=False)
        
        # --- 3. DRAW THE HUD ---
        if len(results) > 0:
            boxes = results[0].boxes.xyxy.cpu().numpy()
            for box in boxes:
                # Get coordinates
                x1, y1, x2, y2 = int(box[0]), int(box[1]), int(box[2]), int(box[3])
                cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
                
                # Draw Bounding Box and Crosshair
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
                cv2.putText(frame, "TARGET LOCKED", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Encode the modified frame to JPEG
        ret, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()

        # Broadcast it
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

    cap.release()

@app.route('/')
def index():
    # CSS updated to make the video feed scale to the full browser window!
    return '''
    <html>
        <head>
            <title>AXI6 AI Tracker</title>
            <style>
                body { margin: 0; background-color: #000; display: flex; justify-content: center; align-items: center; height: 100vh; overflow: hidden; }
                img { width: 100vw; height: 100vh; object-fit: contain; }
            </style>
        </head>
        <body>
            <img src="/video_feed">
        </body>
    </html>
    '''

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    print("\n--- Starting AXI6 AI Server ---")
    app.run(host='0.0.0.0', port=5000, debug=False)