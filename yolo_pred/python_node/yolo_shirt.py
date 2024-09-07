#!/usr/bin/env python3
import cv2
import numpy as np
import yaml
from yaml.loader import SafeLoader

# Load labels from YAML file
with open("/home/administrator/ros2arm_ws/src/yolo_pred/Model_Shirt/data3.yaml", mode='r') as f:
    data_yaml = yaml.load(f, Loader=SafeLoader)
labels = data_yaml['names']

# Load YOLO model
yolo = cv2.dnn.readNetFromONNX("/home/administrator/ros2arm_ws/src/yolo_pred/Model_Shirt/weights/best.onnx")
yolo.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
yolo.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
# Read and prepare image
img = cv2.imread("/home/administrator/ros2arm_ws/src/yolo_pred/Model_Shirt/76.jpeg")
image = img.copy()
row, col, d = image.shape  # height, width, depth
max_rc = max(row, col)
input_image = np.zeros((max_rc, max_rc, 3), dtype=np.uint8)
input_image[0:row, 0:col] = image

# Set input dimensions for YOLO
INPUT_WH_YOLO = 640
blob = cv2.dnn.blobFromImage(input_image, 1/255, (INPUT_WH_YOLO, INPUT_WH_YOLO), swapRB=True, crop=False)
yolo.setInput(blob)
preds = yolo.forward()

detections = preds[0]
boxes = []
confidences = []
classes = []

image_w, image_h = input_image.shape[:2]
x_fac = image_w / INPUT_WH_YOLO
y_fac = image_h / INPUT_WH_YOLO

# Process detections
for i in range(len(detections)):
    row = detections[i]
    confidence = row[4]
    if confidence > 0:
        class_score = row[5:].max()
        class_id = row[5:].argmax()
        if class_score > 0:
            cx, cy, w, h = row[0:4]
            left = int((cx - 0.5 * w) * x_fac)
            top = int((cy - 0.5 * h) * y_fac)
            width = int(w * x_fac)
            height = int(h * y_fac)
            box = [left, top, width, height]
            confidences.append(confidence)
            boxes.append(box)
            classes.append(class_id)

# Non-Max Suppression
result = cv2.dnn.NMSBoxes(boxes, confidences, 0.3, 0.2)

# Initialize a list to store bounding box details
bounding_boxes_info = []

if len(result) > 0:
    flattened_result = result.flatten()
    for ind in flattened_result:
        x, y, w, h = boxes[ind]
        bb_conf = int(confidences[ind] * 100)
        class_id = classes[ind]
        class_name = labels[class_id]
        text = f'{class_name}:{bb_conf}%'
        
        print(x,y,w,h,bb_conf)
        # Store bounding box info
        bounding_box_info = {
            "coordinates": (x, y, w, h),
            # "confidence": bb_conf,
            # "class_id": class_id,
            # "class_name": class_name
        }
        bounding_boxes_info.append(bounding_box_info)
       
        # Draw bounding box and label on the image
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.rectangle(image, (x, y - 30), (x + w, y), (255, 255, 255), -1)
        cv2.putText(image, text, (x, y - 10), cv2.FONT_HERSHEY_PLAIN, 0.7, (0, 0, 0), 1)
else:
    print(" Debug #1 - No detections")

# Print bounding box details
for bbox_info in bounding_boxes_info:
    print(f"Bounding Box Info: {bbox_info}")

# Display and wait
cv2.imshow('yolo_prediction', image)
cv2.waitKey(0)
cv2.destroyAllWindows()