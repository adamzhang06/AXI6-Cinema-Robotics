import cv2
from flask import Flask, Response

app = Flask(__name__)

def generate_frames():
    print("[INFO] Initializing Logitech Brio 101...")
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("[ERROR] Camera not found. Try changing VideoCapture(0) to (1).")
        return

    # Keep resolution manageable for the Pi 3B over Wi-Fi
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    # Cap FPS to 30 to save CPU
    cap.set(cv2.CAP_PROP_FPS, 30) 

    while True:
        success, frame = cap.read()
        if not success:
            break
            
        # Flip the frame to act like a mirror
        frame = cv2.flip(frame, 1)

        # Encode the frame as a JPEG image
        ret, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()

        # Yield the frame in the byte format required for an MJPEG stream
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

    cap.release()

@app.route('/')
def index():
    # A simple HTML page that displays the video stream
    return '''
    <html>
        <head><title>AXI6 Camera Stream</title></head>
        <body style="background-color:#121212; color:white; text-align:center; font-family: sans-serif;">
            <h2>Logitech Brio Live Feed</h2>
            <img src="/video_feed" style="border: 2px solid #00FF00; border-radius: 8px; max-width: 100%;">
        </body>
    </html>
    '''

@app.route('/video_feed')
def video_feed():
    # Route that actually serves the MJPEG frames
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    print("\n--- Starting AXI6 Video Server ---")
    print("Go to your Mac's browser and type the Pi's IP address followed by :5000")
    print("Example: http://raspberrypi.local:5000\n")
    # Host on 0.0.0.0 so it is accessible to other devices on the Wi-Fi network
    app.run(host='0.0.0.0', port=5000, debug=False)