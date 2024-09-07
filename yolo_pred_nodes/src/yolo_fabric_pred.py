#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
from yaml.loader import SafeLoader
from std_msgs.msg import String

rclpy.init()

# Create a node
node = Node("yolo_image_publisher_node")

# Initialize CV Bridge
bridge = CvBridge()

# Create a publisher
publisher = node.create_publisher(Image, "fabric_yolo_image_topic", 10)

publisher2 = node.create_publisher(String, "fabric_conf_data", 10)
# Load labels from YAML file
with open("/home/administrator/ros2arm_ws/src/yolo_pred_nodes/Model_fabric/data2.yaml", mode='r') as f:
    data_yaml = yaml.load(f, Loader=SafeLoader)
labels = data_yaml['names']

# Load YOLO model
yolo = cv2.dnn.readNetFromONNX("/home/administrator/ros2arm_ws/src/yolo_pred_nodes/Model_fabric/weights/best2.onnx")
yolo.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
yolo.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

def image_callback(msg):
    global bridge, yolo, labels, publisher

    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    image = frame.copy()
    row, col, _ = image.shape
    max_rc = max(row, col)
    input_image = np.zeros((max_rc, max_rc, 3), dtype=np.uint8)
    input_image[0:row, 0:col] = image

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
    bb_conf =0
    if len(result) > 0:
        flattened_result = result.flatten()
        for ind in flattened_result:
            x, y, w, h = boxes[ind]
            bb_conf = int(confidences[ind] * 100)
            class_id = classes[ind]
            class_name = labels[class_id]
            text = f'{class_name}:{bb_conf}%'

            print(bb_conf)
           
            # Draw bounding box and label on the frame
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.rectangle(frame, (x, y - 30), (x + w, y), (255, 255, 255), -1)
            cv2.putText(frame, text, (x, y - 10), cv2.FONT_HERSHEY_PLAIN, 0.7, (0, 0, 0), 1)

    # Display the resulting frame
    # cv2.imshow('Frame', frame)
    # Convert the OpenCV image to a ROS message
            
    # ros_image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
    # # Publish the processed image
    # publisher.publish(ros_image_msg)
    else:
        print(" Debug #1 - No detections")
    
    ros_image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")

            # Publish the image
    publisher.publish(ros_image_msg)
    publisher2.publish(String(data=str(bb_conf)))

# Subscribe to camera_frames
subscription = node.create_subscription(
    Image, 
    "camera_frames", 
    image_callback, 
    10)

try:
    # Spin to keep the script for callback execution
    rclpy.spin(node)
except KeyboardInterrupt:
    # Gracefully shutting down
    pass
finally:
    # Cleanup
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()
