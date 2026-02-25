from ultralytics import YOLO
import cv2
import os

# 1. Load CoreML model
model_path = os.path.join(os.path.dirname(__file__), "models/yolo26n-pose.mlpackage")
model = YOLO(model_path)

# 2. Run the tracker with M4 Pro GPU (mps)
# 'stream=True' is required for live webcam to prevent memory buildup
results = model.track(
    source="0",           # "0" is usually built-in FaceTime camera
    show=True,            # Pops up a window with detections drawn
    tracker="botsort.yaml", 
    persist=True, 
    device="mps",         # Uses 28-core GPU
    stream=True           # Essential for real-time performance
)

# 3. Iterate through results to keep the script running
print("AXI6 Vision Started. Press 'q' on the video window to stop.")

for r in results:
    # This loop runs every time a new frame is processed (Target: 60fps)
    if r.boxes.id is not None:
        # Get the (x, y) of the person's nose (first keypoint)
        # keypoints = r.keypoints.xy[0][0] 
        pass

    # Close the window or press 'q' to end the script
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()