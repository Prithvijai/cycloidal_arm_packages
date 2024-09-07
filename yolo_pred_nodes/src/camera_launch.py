#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

def main(args=None):
    rclpy.init(args=args)
    
    # Create a standalone node
    node = Node("video_publisher")
    
    # Create a publisher
    publisher = node.create_publisher(Image, "camera_frames", 10)
    
    # Initialize the video capture
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        node.get_logger().error('Failed to open video capture device')
        return
    
    bridge = CvBridge()
    
    try:
        while rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                node.get_logger().error('Failed to capture frame')
                continue
            
            # Convert the OpenCV image to a ROS Image message
            ros_image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            
            # Publish the ROS Image message
            publisher.publish(ros_image_msg)
            
            # Handle ROS events and callbacks
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
