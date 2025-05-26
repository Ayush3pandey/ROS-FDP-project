# 🤖 ROS2 ESP32 Teleoperation Robot

[![Made with ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://www.ros.org/)
[![Platform](https://img.shields.io/badge/Platform-ESP32-green)](https://www.espressif.com/en/products/socs/esp32)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

---

## 🚀 Project Overview

This project is a ROS2-based teleoperated robot built using ESP32, featuring both keyboard (WASD) and computer vision (hand gesture) control.  
Developed as part of the **Foundation of Design Practicum (FDP)** at **IIT Mandi**.

---

## 👨‍💻 Team

- **Arihant Kumar Jain**
- Ayush Pandey
- Arindam Mukherjee
- Arya Mundra

---

## ✨ Features

- **ROS2 Humble + micro-ROS** integration
- **WASD Keyboard Teleoperation**
- **OpenCV + MediaPipe Hand Gesture Teleoperation**
- **Path Recording & Replay** (return-to-origin)
- Real-time robot control over WiFi

---

## 🛠️ Tech Stack

- ROS2 Humble
- micro-ROS
- ESP32
- Python 3 (rclpy, OpenCV, MediaPipe)
- Arduino (for ESP32 firmware)

---

## ⚡ Quick Start

### 1. Clone the Repository

git clone https://github.com/Ayush3pandey/ROS-FDP-project.git
cd ROS-FDP-project


### 2. Setup ROS2 Workspace

cd perp_ws
colcon build
source install/setup.bash


### 3. Flash ESP32

Upload the provided Arduino code to your ESP32 using the Arduino IDE.

### 4. Run the System

- **micro-ROS Agent:**  
  `ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888`

- **WASD Teleop:**  
  `ros2 run wasd_teleop wasd_teleop`

- **Vision Teleop:**  
  `ros2 run vision_teleop vision_teleop`

- **Path Recorder:**  
  `ros2 run path_recorder path_recorder`

- **Path Controller:**  
  `ros2 run path_recorder path_controller`

---

## 📝 How It Works

- **Teleop nodes** publish velocity commands to `/cmd_vel`
- **ESP32** subscribes to `/cmd_vel` and controls the motors
- **Path Recorder** logs all commands and can replay them in reverse

---

<details> <summary>Click to expand ESP32 code</summary>
  
#include <micro_ros_arduino.h>
#include <WiFi.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

// Motor pin definitions
#define RIGHT_MOTOR_PIN1 5      //IN1
#define RIGHT_MOTOR_PIN2 18      //IN2
#define LEFT_MOTOR_PIN1 27      //IN3
#define LEFT_MOTOR_PIN2 14      //IN4
#define RIGHT_ENABLE_PIN 4   // ENA for right motor
#define LEFT_ENABLE_PIN 19   // ENB for left motor

// PWM properties
#define PWM_FREQ 5000        // 5 kHz PWM frequency
#define PWM_RESOLUTION 8     // 8-bit resolution (0-255)
#define RIGHT_PWM_CHANNEL 0  // PWM channel for right motor
#define LEFT_PWM_CHANNEL 1   // PWM channel for left motor

// Initial speed values (you can adjust these based on your motors)
int rightMotorSpeed = 229;   // Speed for right motor (0-255)
int leftMotorSpeed = 255;    // Speed for left motor (0-255)

// Built-in LED pin for ESP32 (usually GPIO 2)
#define BOARD_LED_PIN 2

// WiFi credentials (MUST be mutable char arrays)
char ssid[32] = "YOUR SSID";
char password[32] = "YOUR PASSWORD";
char agent_ip_str[16] = "YOUR IP";  // Mutable buffer for IP string
IPAddress agent_ip(YOUR IP);
uint16_t agent_port = 8888;

// ROS2 entities
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

// Error handling macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if(temp_rc != RCL_RET_OK) { error_loop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if(temp_rc != RCL_RET_OK) { }}

// Error loop for critical failures
void error_loop() {
  while(1) {
    digitalWrite(BOARD_LED_PIN, !digitalRead(BOARD_LED_PIN));
    delay(100);
  }
}

// Callback for /cmd_vel subscription
void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
  float linear_x = msg->linear.x;
  float angular_z = msg->angular.z;

  // Debug print
  Serial.print("Command: linear_x=");
  Serial.print(linear_x);
  Serial.print(", angular_z=");
  Serial.println(angular_z);

  // W - Forward
  if (linear_x > 0) {
    // Set PWM for forward motion
    ledcWrite(LEFT_PWM_CHANNEL, leftMotorSpeed);
    ledcWrite(RIGHT_PWM_CHANNEL, rightMotorSpeed);
    
    digitalWrite(LEFT_MOTOR_PIN1, HIGH);  //ABHI CHANGE KIYA FOR DEBUG
    digitalWrite(LEFT_MOTOR_PIN2, LOW);
    digitalWrite(RIGHT_MOTOR_PIN1, HIGH);
    digitalWrite(RIGHT_MOTOR_PIN2, LOW);
    
    Serial.println("Moving forward");
  }
  // S - Backward
  else if (linear_x < 0) {
    // Set PWM for backward motion
    ledcWrite(LEFT_PWM_CHANNEL, leftMotorSpeed);
    ledcWrite(RIGHT_PWM_CHANNEL, rightMotorSpeed);
    
    digitalWrite(LEFT_MOTOR_PIN1, LOW);
    digitalWrite(LEFT_MOTOR_PIN2, HIGH);
    digitalWrite(RIGHT_MOTOR_PIN1, LOW);
    digitalWrite(RIGHT_MOTOR_PIN2, HIGH);
    
    Serial.println("Moving backward");
  }
  // A - Left
  else if (angular_z > 0) {
    // Set PWM for left turn
    ledcWrite(LEFT_PWM_CHANNEL, leftMotorSpeed);
    ledcWrite(RIGHT_PWM_CHANNEL, rightMotorSpeed);
    
    digitalWrite(LEFT_MOTOR_PIN1, LOW);
    digitalWrite(LEFT_MOTOR_PIN2, HIGH);
    digitalWrite(RIGHT_MOTOR_PIN1, HIGH);
    digitalWrite(RIGHT_MOTOR_PIN2, LOW);
    
    Serial.println("Turning left");
  }
  // D - Right
  else if (angular_z < 0) {
    // Set PWM for right turn
    ledcWrite(LEFT_PWM_CHANNEL, leftMotorSpeed);
    ledcWrite(RIGHT_PWM_CHANNEL, rightMotorSpeed);
    
    digitalWrite(LEFT_MOTOR_PIN1, HIGH);
    digitalWrite(LEFT_MOTOR_PIN2, LOW);
    digitalWrite(RIGHT_MOTOR_PIN1, LOW);
    digitalWrite(RIGHT_MOTOR_PIN2, HIGH);
    
    Serial.println("Turning right");
  }
  // Stop
  else {
    // Set PWM to zero for stop
    ledcWrite(LEFT_PWM_CHANNEL, 0);
    ledcWrite(RIGHT_PWM_CHANNEL, 0);
    
    digitalWrite(LEFT_MOTOR_PIN1, LOW);
    digitalWrite(LEFT_MOTOR_PIN2, LOW);
    digitalWrite(RIGHT_MOTOR_PIN1, LOW);
    digitalWrite(RIGHT_MOTOR_PIN2, LOW);
    
    Serial.println("Stopping");
  }
}

// Function to test motors independently
void testMotors() {
  Serial.println("Testing motors...");
  
  // Test left motor
  Serial.println("Testing left motor forward...");
  ledcWrite(LEFT_PWM_CHANNEL, leftMotorSpeed);
  digitalWrite(LEFT_MOTOR_PIN1, HIGH);
  digitalWrite(LEFT_MOTOR_PIN2, LOW);
  delay(1000);
  
  // Stop left motor
  digitalWrite(LEFT_MOTOR_PIN1, LOW);
  digitalWrite(LEFT_MOTOR_PIN2, LOW);
  ledcWrite(LEFT_PWM_CHANNEL, 0);
  delay(500);
  
  // Test right motor
  Serial.println("Testing right motor forward...");
  ledcWrite(RIGHT_PWM_CHANNEL, rightMotorSpeed);
  digitalWrite(RIGHT_MOTOR_PIN1, HIGH);
  digitalWrite(RIGHT_MOTOR_PIN2, LOW);
  delay(1000);
  
  // Stop right motor
  digitalWrite(RIGHT_MOTOR_PIN1, LOW);
  digitalWrite(RIGHT_MOTOR_PIN2, LOW);
  ledcWrite(RIGHT_PWM_CHANNEL, 0);
  
  Serial.println("Motor test complete");
}

void setup() {
  // Initialize motor pins
  pinMode(LEFT_MOTOR_PIN1, OUTPUT);
  pinMode(LEFT_MOTOR_PIN2, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN1, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN2, OUTPUT);
  
  // Configure PWM for motor speed control
  ledcSetup(RIGHT_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(LEFT_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  
  // Attach PWM channels to enable pins
  ledcAttachPin(RIGHT_ENABLE_PIN, RIGHT_PWM_CHANNEL);
  ledcAttachPin(LEFT_ENABLE_PIN, LEFT_PWM_CHANNEL);
  
  // Set initial motor speeds to 0 (stopped)
  ledcWrite(RIGHT_PWM_CHANNEL, 0);
  ledcWrite(LEFT_PWM_CHANNEL, 0);

  // Stop motors at start
  digitalWrite(LEFT_MOTOR_PIN1, LOW);
  digitalWrite(LEFT_MOTOR_PIN2, LOW);
  digitalWrite(RIGHT_MOTOR_PIN1, LOW);
  digitalWrite(RIGHT_MOTOR_PIN2, LOW);

  // Initialize built-in LED
  pinMode(BOARD_LED_PIN, OUTPUT);
  digitalWrite(BOARD_LED_PIN, HIGH);

  // Start Serial for debugging
  Serial.begin(115200);
  delay(1000);
  Serial.println("Connecting to WiFi...");

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Test motors to verify hardware connections
  testMotors();

  // Convert IPAddress to mutable char array for micro-ROS
  String ip_string = agent_ip.toString();
  ip_string.toCharArray(agent_ip_str, 16);

  // Set up micro-ROS transport with mutable buffers
  set_microros_wifi_transports(ssid, password, agent_ip_str, agent_port);
  delay(2000);

  allocator = rcl_get_default_allocator();

  // Initialize ROS2 node and subscriber
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_teleop_node", "", &support));
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"
  ));

  // Initialize executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  Serial.println("ESP32 teleop node initialized!");
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  delay(10);
}
</details>

---

## 📚 Acknowledgements

Special thanks to **IIT Mandi** and our mentors for their guidance and support.

---

## 📄 License

This project is licensed under the Apache 2.0 License.

---
