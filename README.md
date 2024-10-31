Stepper Motor Control via Hand Gestures

This project allows for stepper motor control using hand gestures detected through a webcam. Utilizing OpenCV and MediaPipe, 
the system identifies whether the hand is open or closed and adjusts the motor speed accordingly. 
The project combines Python code for gesture detection with an Arduino Nano Every and an A4988 driver for motor control.

Features
- Hand Detection with OpenCV and MediaPipe: Detects hands and determines if the hand is open or closed.
- Real-Time Control: The detected hand position and openness control the stepper motor's speed and direction.
- Serial Communication with Arduino: Sends control data via serial communication to the Arduino, which adjusts the motor speed.

Project Requirements
- Python Libraries: 
  - opencv-python for camera input.
  - mediapipe for hand detection.
  - pyserial for serial communication.
  - numpy for array manipulation.
- Hardware:
  - Arduino Nano Every
  - A4988 Stepper Motor Driver
  - Stepper Motor

Installation
1. Clone this repository:
   git clone https://github.com/lukakvesic/Steper-motor-control-with-WEB-camera
2. Install the required Python packages:
   pip install opencv-python mediapipe pyserial numpy

Usage

Python Script for Hand Detection
1. Connect the camera.
2. Run the Python script for hand detection:
   python hand_control.py
3. Select the correct COM port when prompted to establish a serial connection with Arduino.

Arduino Code
1. Open the Arduino IDE.
2. Upload the motor_control.ino file to your Arduino Nano Every.
3. Connect the A4988 driver and stepper motor as specified in the schematic.

Code Overview

hand_control.py
This Python script uses OpenCV and MediaPipe to detect hands, determine their openness, and calculate movement speed. 
Detected hand state and speed are sent to the Arduino for motor control.

motor_control.ino
The Arduino code interprets the received data and adjusts the stepper motor's speed and direction based on hand openness and movement data.

Control Details
- Hand Open: Motor operates at specified speed.
- Hand Closed: Motor stops.
- Speed Adjustment: Movement detected along the x-axis in the camera frame adjusts the motor speed.

License
This project is open-source and available under the MIT License.
