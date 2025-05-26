#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import mediapipe as mp
import numpy as np

class VisionTeleop(Node):
    def __init__(self):
        super().__init__('vision_teleop')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Initialize camera directly - simpler approach
        self.cap = cv2.VideoCapture(0)
        
        # Initialize MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.7)  # Higher confidence threshold like your friend's code
        self.mp_drawing = mp.solutions.drawing_utils
        
        # Control parameters
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        self.cmd = None
        
        # Create timer for regular processing
        self.timer = self.create_timer(0.1, self.process_frame)
        
        self.get_logger().info('Hand gesture teleop started. Press Q in the window to quit.')
    
    def is_hand_open(self, hand_landmarks):
        # Check if fingers are extended
        fingers_extended = []
        for finger_tip_id, finger_pip_id in [
            (self.mp_hands.HandLandmark.INDEX_FINGER_TIP, self.mp_hands.HandLandmark.INDEX_FINGER_PIP),
            (self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP, self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP),
            (self.mp_hands.HandLandmark.RING_FINGER_TIP, self.mp_hands.HandLandmark.RING_FINGER_PIP),
            (self.mp_hands.HandLandmark.PINKY_TIP, self.mp_hands.HandLandmark.PINKY_PIP)
        ]:
            finger_tip = hand_landmarks.landmark[finger_tip_id]
            finger_pip = hand_landmarks.landmark[finger_pip_id]
            fingers_extended.append(finger_tip.y < finger_pip.y)
        
        # Hand is considered open if at least 3 fingers are extended
        return sum(fingers_extended) >= 3
    
    def process_frame(self):
        # Read frame
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame')
            return
        
        # Flip frame horizontally for more intuitive interaction
        frame = cv2.flip(frame, 1)
        
        # Convert to RGB for MediaPipe
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Process frame with MediaPipe
        results = self.hands.process(rgb_frame)
        
        # Reset command
        self.cmd = None
        
        # Draw hand landmarks
        if results.multi_hand_landmarks:
            left_hand_open = None
            right_hand_open = None
            
            for idx, hand_landmarks in enumerate(results.multi_hand_landmarks):
                # Draw landmarks
                self.mp_drawing.draw_landmarks(
                    frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                
                # Determine if hand is left or right
                if results.multi_handedness[idx].classification[0].label == "Left":
                    # This is actually the right hand from camera perspective
                    right_hand_open = self.is_hand_open(hand_landmarks)
                else:
                    # This is actually the left hand from camera perspective
                    left_hand_open = self.is_hand_open(hand_landmarks)
            
            # Determine command based on hand states
            if left_hand_open is not None and right_hand_open is not None:
                # Both hands detected
                if left_hand_open and right_hand_open:
                    self.cmd = 'forward'
                elif not left_hand_open and not right_hand_open:
                    self.cmd = 'backward'
                elif left_hand_open and not right_hand_open:
                    self.cmd = 'left'
                elif not left_hand_open and right_hand_open:
                    self.cmd = 'right'
            elif left_hand_open is not None:
                # Only left hand detected
                if left_hand_open:
                    self.cmd = 'left'
                else:
                    self.cmd = 'backward'
            elif right_hand_open is not None:
                # Only right hand detected
                if right_hand_open:
                    self.cmd = 'right'
                else:
                    self.cmd = 'backward'
        
        # Display command on frame
        if self.cmd:
            cv2.putText(frame, f"Command: {self.cmd}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Create and publish Twist message
            twist = Twist()
            if self.cmd == 'forward':
                twist.linear.x = self.linear_speed
            elif self.cmd == 'backward':
                twist.linear.x = -self.linear_speed
            elif self.cmd == 'left':
                twist.angular.z = self.angular_speed
            elif self.cmd == 'right':
                twist.angular.z = -self.angular_speed
                
            self.publisher.publish(twist)
            self.get_logger().info(f'Command: {self.cmd}')
        else:
            cv2.putText(frame, "No command", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        # Display the frame
        cv2.imshow('Hand Gesture Teleop', frame)
        
        # Check for key press
        key = cv2.waitKey(5) & 0xFF
        if key == ord('q'):
            # Stop the robot when quitting
            twist = Twist()
            self.publisher.publish(twist)
            self.cap.release()
            cv2.destroyAllWindows()
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    vision_teleop = VisionTeleop()
    rclpy.spin(vision_teleop)
    
    # Clean up
    vision_teleop.cap.release()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
