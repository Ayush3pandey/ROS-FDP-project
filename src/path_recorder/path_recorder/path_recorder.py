#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
import numpy as np

class PathRecorder(Node):
    def __init__(self):
        super().__init__('path_recorder')
        
        # Create publisher for robot commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create subscriber to listen for commands from teleop nodes
        self.cmd_vel_sub = self.create_subscription(
            Twist, 
            '/cmd_vel', 
            self.cmd_vel_callback, 
            10)
            
        # Create subscriber for control commands (start/stop recording)
        self.control_sub = self.create_subscription(
            String,
            '/path_control',
            self.control_callback,
            10)
            
        # Create publisher for status updates
        self.status_pub = self.create_publisher(String, '/path_status', 10)
        
        # Recording variables
        self.recording = False
        self.path_history = []  # List of (twist, timestamp) tuples
        self.start_time = 0
        self.replaying = False
        self.replay_index = 0
        
        # Create timer for replay functionality
        self.timer = self.create_timer(0.05, self.replay_timer_callback)
        
        self.get_logger().info('Path recorder node started')
        self.publish_status("READY")
        
    def cmd_vel_callback(self, msg):
        """Record incoming velocity commands when recording is active"""
        if self.recording and not self.replaying:
            # Store the command and relative timestamp
            current_time = time.time() - self.start_time
            self.path_history.append((msg, current_time))
            self.get_logger().debug(f'Recorded command at t={current_time:.2f}s')
    
    def control_callback(self, msg):
        """Handle control commands for recording/replaying"""
        command = msg.data.upper()
        
        if command == "START_RECORDING":
            if not self.recording and not self.replaying:
                self.start_recording()
                
        elif command == "STOP_RECORDING":
            if self.recording:
                self.stop_recording()
                
        elif command == "REPLAY":
            if not self.recording and not self.replaying and self.path_history:
                self.start_replay()
                
        elif command == "CANCEL":
            self.cancel_all()
    
    def start_recording(self):
        """Start recording the robot's path"""
        self.path_history = []
        self.start_time = time.time()
        self.recording = True
        self.get_logger().info('Started recording path')
        self.publish_status("RECORDING")
    
    def stop_recording(self):
        """Stop recording the robot's path"""
        self.recording = False
        self.get_logger().info(f'Stopped recording. Recorded {len(self.path_history)} commands')
        self.publish_status("RECORDED")
    
    def start_replay(self):
        """Start replaying the recorded path in reverse"""
        if not self.path_history:
            self.get_logger().warn('No path recorded to replay')
            return
            
        self.replaying = True
        self.replay_index = 0
        self.start_time = time.time()
        self.get_logger().info('Starting path replay in reverse')
        self.publish_status("REPLAYING")
        
        # Create reversed path by inverting velocities and preserving timing
        self.replay_path = []
        for twist, timestamp in self.path_history:
            reversed_twist = Twist()
            reversed_twist.linear.x = -twist.linear.x
            reversed_twist.linear.y = -twist.linear.y
            reversed_twist.linear.z = -twist.linear.z
            reversed_twist.angular.x = -twist.angular.x
            reversed_twist.angular.y = -twist.angular.y
            reversed_twist.angular.z = -twist.angular.z
            
            self.replay_path.append((reversed_twist, timestamp))
            
        # Sort by timestamp to ensure proper replay order
        self.replay_path.sort(key=lambda x: x[1])
    
    def cancel_all(self):
        """Cancel any active recording or replay"""
        if self.recording or self.replaying:
            self.recording = False
            self.replaying = False
            
            # Send stop command to robot
            stop_twist = Twist()
            self.cmd_vel_pub.publish(stop_twist)
            
            self.get_logger().info('Cancelled current operation')
            self.publish_status("CANCELLED")
    
    def replay_timer_callback(self):
        """Timer callback for replaying the path"""
        if not self.replaying:
            return
            
        current_time = time.time() - self.start_time
        
        # Find commands that should be executed by now
        while (self.replay_index < len(self.replay_path) and 
               self.replay_path[self.replay_index][1] <= current_time):
            
            # Publish the command
            twist, _ = self.replay_path[self.replay_index]
            self.cmd_vel_pub.publish(twist)
            self.get_logger().debug(f'Replaying command {self.replay_index+1}/{len(self.replay_path)}')
            
            self.replay_index += 1
            
            # Check if we've finished replaying
            if self.replay_index >= len(self.replay_path):
                # Send stop command
                stop_twist = Twist()
                self.cmd_vel_pub.publish(stop_twist)
                
                self.replaying = False
                self.get_logger().info('Path replay completed')
                self.publish_status("READY")
                break
    
    def publish_status(self, status):
        """Publish the current status of the path recorder"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PathRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
