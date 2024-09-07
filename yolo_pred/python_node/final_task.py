#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import cv2
import numpy as np
import yaml
from yaml.loader import SafeLoader

# Load labels from YAML file
with open("/home/administrator/ros2arm_ws/src/yolo_pred/Model_Shirt/data3.yaml", mode='r') as f:  # Adjust the path as necessary
    data_yaml = yaml.load(f, Loader=SafeLoader)
labels = data_yaml['names']

# Load YOLO model
yolo = cv2.dnn.readNetFromONNX("/home/administrator/ros2arm_ws/src/yolo_pred/Model_Shirt/weights/best2.onnx")  # Adjust the path as necessary
yolo.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
yolo.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

def publish_command(publisher, command):
    msg = String()
    msg.data = command
    publisher.publish(msg)
    print(f'Publishing: "{msg.data}"')

def motor_state_callback(msg):
    global current_state
    current_state = msg.data
    print(f'Current motor state: "{current_state}"')

def shirt_conf_callback(msg):
    global shirt_conf
    shirt_conf = int(msg.data)
    print(f'Current conf: "{shirt_conf}"')

def fabric_conf_callback(msg):
    global fabric_conf
    shirt_conf = int(msg.data)
    print(f'Current fabric conf: "{shirt_conf}"')


def main(args=None):
    global current_state, shirt_conf, fabric_conf
    current_state = ""
    shirt_conf = 0
    fabric_conf = 0
    rest = '0,0,0,0,0'
    Fready = '-30,-20,-100,180,90'
    Fzoom = '-30,20,-70,180,90'
    scans = ['-40,20,-70,180,90', '-30,20,-70,180,90', '-20,20,-70,180,90']

    rclpy.init(args=args)
    node = Node('hardware_robot_controller')
    
    publisher = node.create_publisher(String, '/motor_commands', 10)
    subscription = node.create_subscription(String, '/motor_states', motor_state_callback, 10)
    subscription2 = node.create_subscription(String, '/Shirt_conf_data', shirt_conf_callback, 10)
    subscription3 = node.create_subscription(String, '/fabric_conf_data', fabric_conf_callback, 10)
    # Move to rest position initially
    desired_state = rest
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if current_state == desired_state:
            print(f'Desired state "{desired_state}" achieved, moving to next position.')
            break
        else:
            publish_command(publisher, desired_state)
        time.sleep(1)  # Adjust delay as needed

    repeat = 'y'
    while repeat.lower() == 'y' and rclpy.ok():
        # Move to First Ready position
        desired_state = Fready
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if current_state == desired_state:
                print(f'Desired state "{desired_state}" achieved, moving to next position.')
                break
            else:
                publish_command(publisher, desired_state)
            time.sleep(1)
        
        print("The Robot is in Ready State. Show the Shirt Patch to the robot.")
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if shirt_conf >= 10:
                print(f'Shirt Detected "{shirt_conf}", moving to next position.')
                n = '1'
                break
            else:
                pass
                # publish_command(publisher, desired_state)
            time.sleep(1)
        
        # n = input("Press 1 to continue to zoom.")

        if n == '1':
        # Move to First Zoom position
            desired_state = Fzoom
            print('a')
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
                if current_state == desired_state:
                    print(f'Desired state "{desired_state}" achieved, moving to next position.')
                    break
                else:
                    publish_command(publisher, desired_state)
                time.sleep(1)
            
            print("The Robot is zoomed in. Scanning will commence.")
            for scan in scans:
                desired_state = scan
                while rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=0.1)
                    if fabric_conf >= 50:
                        print(f'There is a defect in fabric!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
                        break
                        
                    if current_state == desired_state:
                        print(f'Desired state "{desired_state}" achieved, moving to next position.')
                        break
                    else:
                        publish_command(publisher, desired_state)
                    time.sleep(1)

        # Optionally, move back to First Ready position after scanning
        desired_state = Fready
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if current_state == desired_state:
                print(f'Desired state "{desired_state}" achieved. Sequence complete.')
                break
            else:
                publish_command(publisher, desired_state)
            time.sleep(1)
            
        repeat = input("Repeat the sequence? (y/n): ")
    
    desired_state = rest
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if current_state == desired_state:
            print(f'Desired state "{desired_state}" achieved, moving to next position.')
            break
        else:
            publish_command(publisher, desired_state)
        time.sleep(1)  # Adjust delay as needed

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
