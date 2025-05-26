#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty
import threading

class PathKeyboardController(Node):
    def __init__(self):
        super().__init__('path_keyboard_controller')
        
        # Create publisher for control commands
        self.control_pub = self.create_publisher(String, '/path_control', 10)
        
        # Create subscriber for status updates
        self.status_sub = self.create_subscription(
            String,
            '/path_status',
            self.status_callback,
            10)
            
        self.current_status = "READY"
        self.get_logger().info('Path keyboard controller started')
        self.get_logger().info('Controls:')
        self.get_logger().info('  r - Start recording')
        self.get_logger().info('  s - Stop recording')
        self.get_logger().info('  SPACE - Replay path in reverse')
        self.get_logger().info('  c - Cancel current operation')
        self.get_logger().info('  q - Quit')
    
    def status_callback(self, msg):
        """Handle status updates from the path recorder"""
        self.current_status = msg.data
        self.get_logger().info(f'Status: {self.current_status}')
    
    def send_command(self, command):
        """Send a command to the path recorder"""
        msg = String()
        msg.data = command
        self.control_pub.publish(msg)
        self.get_logger().info(f'Sent command: {command}')
    
    def get_key(self):
        """Get a single keypress from the user"""
        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
    
    def run(self):
        """Run the keyboard controller"""
        self.get_logger().info('Press keys to control path recording...')
        
        while True:
            key = self.get_key()
            
            if key == 'r':
                self.send_command('START_RECORDING')
            elif key == 's':
                self.send_command('STOP_RECORDING')
            elif key == ' ':  # Space
                self.send_command('REPLAY')
            elif key == 'c':
                self.send_command('CANCEL')
            elif key == 'q':
                break
        
        self.get_logger().info('Exiting...')

def main(args=None):
    rclpy.init(args=args)
    controller = PathKeyboardController()
    
    # Run the keyboard input in a separate thread
    thread = threading.Thread(target=controller.run)
    thread.daemon = True
    thread.start()
    
    rclpy.spin(controller)
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
