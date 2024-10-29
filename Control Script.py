import cv2
import mediapipe as mp
import numpy as np
import math
from collections import deque
import Connect_to_esp

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5)

def remap(value, in_min, in_max, out_min, out_max):
    if out_min < out_max:
        # Normal range
        remapped_Value = out_min + (float(value - in_min) / float(in_max - in_min) * (out_max - out_min))
    else:
        # Inverted range
        remapped_Value = out_max + (float(value - in_min) / float(in_max - in_min) * (out_min - out_max))
    
    # Clamp the value between the output range
    if remapped_Value > max(out_min, out_max):
        remapped_Value = max(out_min, out_max)
    if remapped_Value < min(out_min, out_max):
        remapped_Value = min(out_min, out_max)
    
    return remapped_Value

def initialize_camera():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return None
    else:
        print("Camera successfully initialized.")
        return cap

def calculate_angle(v1, v2):
    """Calculate the angle in degrees between two vectors."""
    dot_product = np.dot(v1, v2)
    magnitude_v1 = np.linalg.norm(v1)
    magnitude_v2 = np.linalg.norm(v2)
    if magnitude_v1 * magnitude_v2 == 0:
        return 0
    angle_rad = math.acos(dot_product / (magnitude_v1 * magnitude_v2))
    return math.degrees(angle_rad)

def get_joint_angles(hand_landmarks):
    # Define joint pairs for angle calculation: [base, middle, tip]
    joint_indices = {
        'thumb': [2, 3, 4],
        'index': [5, 6, 8],
        'middle': [9, 10, 12],
        'ring': [13, 14, 16],
        'pinky': [17, 18, 20]
    }
    
    joint_angles = {}
    for finger, indices in joint_indices.items():
        base = np.array([hand_landmarks[indices[0]].x, hand_landmarks[indices[0]].y])
        middle = np.array([hand_landmarks[indices[1]].x, hand_landmarks[indices[1]].y])
        tip = np.array([hand_landmarks[indices[2]].x, hand_landmarks[indices[2]].y])
        
        # Vectors for joint angles
        vector1 = middle - base
        vector2 = tip - middle
        
        # Calculate the angle
        angle = calculate_angle(vector1, vector2)
        joint_angles[finger] = angle
    
    return joint_angles

def calculate_average_angles(angles_history):
    """Calculate the average of the last 4 sets of angles."""
    averaged_angles = {}
    num_frames = len(angles_history)
    
    for finger in angles_history[0].keys():
        # Average the angle values for each finger across the last few frames
        averaged_angles[finger] = sum([frame[finger] for frame in angles_history]) / num_frames

    return averaged_angles

def main():
    cap = initialize_camera()
    
    if cap is None:
        return

    # Store the last 4 joint angle outputs
    angles_history = deque(maxlen=4)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(rgb_frame)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp.solutions.drawing_utils.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                # Calculate joint angles and add to history
                joint_angles = get_joint_angles(hand_landmarks.landmark)
                angles_history.append(joint_angles)

                # If we have at least 4 frames, calculate the averaged angles
                if len(angles_history) == 4:
                    averaged_angles = calculate_average_angles(angles_history)

                    index = remap(averaged_angles['index'], 5, 140, 0, 180)
                    ring = remap(averaged_angles['ring'], 5, 130, 0, 180)
                    middle = remap(averaged_angles['middle'], 130, 5, 0, 180)
                    thumb = remap(averaged_angles['thumb'], 50, 20, 0, 180)
            
                    '''
Servo 1 : Index, Home : 0
Servo 2 : Ring and Little, Home : 0
Servo 3 : Middle Finger, Home : 180
Servo 4 : Thumb, Home : 180
Servo 5 : Thumb Angle, Home : 10
'''
                    servoangles = [index, ring, middle, thumb, 10]
                    Connect_to_esp.set_servo_angles('192.168.1.121',servoangles)
                    print(servoangles)

        cv2.imshow('Hand Tracking', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
