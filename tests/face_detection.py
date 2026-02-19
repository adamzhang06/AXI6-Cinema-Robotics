import cv2
import time
import os
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
# 1. Import the result class directly instead of assigning it to a variable
from mediapipe.tasks.python.vision import FaceDetectorResult

# Resolve the path so the '~' expands correctly on macOS
model_path = os.path.expanduser('./models/blaze_face_short_range.tflite')

# Global variable to safely hold our detection results
latest_result = None

# 2. Use the properly imported FaceDetectorResult
def result_callback(result: FaceDetectorResult, output_image: mp.Image, timestamp_ms: int):
    global latest_result
    latest_result = result

# Setup options using your configuration
options = vision.FaceDetectorOptions(
    base_options=python.BaseOptions(model_asset_path=model_path),
    running_mode=vision.RunningMode.LIVE_STREAM,
    result_callback=result_callback
)

# Initialize the detector and start the camera loop
with vision.FaceDetector.create_from_options(options) as detector:
    cap = cv2.VideoCapture(0)
    print("Tasks API Live Stream Initialized. Press 'q' to quit.")

    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            break

        # Convert OpenCV BGR frame to RGB for MediaPipe
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Convert to a MediaPipe Image object
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
        
        # Calculate current timestamp in milliseconds
        timestamp_ms = int(time.time() * 1000)
        
        # Send the frame asynchronously to the detector
        detector.detect_async(mp_image, timestamp_ms)

        # Extract the offset logic 
        if latest_result and latest_result.detections:
            # Focus on the first detected face
            detection = latest_result.detections[0]
            bbox = detection.bounding_box
            
            # Tasks API gives exact pixel values
            face_center_x = bbox.origin_x + (bbox.width / 2)
            frame_center_x = frame.shape[1] / 2
            
            # The error offset
            offset_x = face_center_x - frame_center_x
            
            # Print the command if it's outside a 15px deadzone
            if abs(offset_x) > 15:
                direction = "RIGHT" if offset_x > 0 else "LEFT"
                print(f"Action: Move Motor {direction} | Offset: {offset_x:.2f} px")

            # Draw a visual bounding box on the Mac for testing
            start_point = (int(bbox.origin_x), int(bbox.origin_y))
            end_point = (int(bbox.origin_x + bbox.width), int(bbox.origin_y + bbox.height))
            cv2.rectangle(frame, start_point, end_point, (0, 255, 0), 2)
        
        # Show the video feed
        cv2.imshow('Tracking Prototype', frame)
        
        # Exit if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()