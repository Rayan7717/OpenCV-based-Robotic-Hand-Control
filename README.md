# OpenCV-based-Robotic-Hand-Control
OpenCV and ESP8266 based Robotic Hand Control

Overview
This project involves controlling a robotic hand using hand gestures tracked by OpenCV and MediaPipe. The system consists of two main components: a Python script for hand tracking and angle calculation, and an ESP8266 microcontroller that interfaces with servo motors to control the robotic hand.
Components
Hand Tracking Script (Control-Script.py)
Utilizes OpenCV and MediaPipe to detect and track hand landmarks.
Calculates joint angles for each finger based on the detected landmarks.
Maps these angles to servo motor positions using a remapping function.
Sends the calculated servo angles to the ESP8266 microcontroller.
ESP8266 Microcontroller
Receives servo angle commands from the Python script.
Controls five servo motors corresponding to different parts of the robotic hand.
Detailed Functionality
Hand Tracking and Angle Calculation
Initialization: The script initializes a webcam using OpenCV and sets up MediaPipe for hand tracking with specified detection and tracking confidence levels.
Angle Calculation: For each detected hand, the script calculates angles at key joints using vector mathematics. This involves:
Defining joint pairs for each finger (thumb, index, middle, ring, pinky).
Calculating angles between vectors formed by these joints.
Angle Averaging: To ensure stability, the script averages the angles over the last four frames.
Remapping Angles
The calculated angles are remapped from their natural range to a range suitable for servo motor control (0 to 180 degrees). This is crucial because the physical movement of servos is constrained to this range.
Communication with ESP8266
Functionality: The Connect_to_esp.py module handles communication with the ESP8266 microcontroller.
HTTP Requests: The script sends HTTP GET requests to set servo angles. Each request specifies which servo to adjust and the desired angle.
Error Handling: The script checks the response status code to ensure successful communication.
Example Usage
The provided example in Connect_to_esp.py demonstrates how to send a series of angle commands to the ESP8266, simulating various hand gestures by adjusting servo positions.
