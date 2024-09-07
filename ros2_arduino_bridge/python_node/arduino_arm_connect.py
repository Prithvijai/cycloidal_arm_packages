#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class HardwareInterfaceNode(Node):
    def __init__(self):
        super().__init__('hardware_interface_node')
        self.subscription = self.create_subscription(
            String,
            '/motor_commands',
            self.command_callback,
            10)
        self.publisher = self.create_publisher(String, '/motor_states', 10)
        
        # Initialize serial connection
        self.serial_port = '/dev/ttyUSB0'
        self.baud_rate = 9600
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Successfully opened serial port {self.serial_port}")
            time.sleep(2)  # Wait for connection to initialize
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening serial port {self.serial_port}: {e}")


    def command_callback(self, msg):
        # Send command to Arduino
        self.get_logger().info(f"Received command: {msg.data}")
        command = msg.data + "\n"
        self.ser.write(command.encode())
        
        # Wait and read response
        time.sleep(1)  # Adjust based on your hardware's response time
        if self.ser.in_waiting:
            response = self.ser.readline().decode('utf-8').rstrip()
            self.publisher.publish(String(data=response))

def main(args=None):
    rclpy.init(args=args)
    node = HardwareInterfaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

