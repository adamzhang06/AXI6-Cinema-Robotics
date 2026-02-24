"""
Only for exporting CV models

- yolo26n-pose: https://docs.ultralytics.com/tasks/pose/
- CoreML is best for Apple Silicon
"""

from ultralytics import YOLO

# Load a model
model = YOLO("yolo26n-pose.pt")

# Export the model
model.export(format="coreml", nms=True, imgsz=640)