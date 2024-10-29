import requests
from time import sleep
def set_servo_angles(ip_address, angles):
    """
    Connects to the ESP8266 microcontroller and sets the angles for each of the 5 servos.

    :param ip_address: The IP address of the ESP8266 device.
    :param angles: A list of 5 integers representing the angles for each servo (0 to 180).

    Servo 1 : Index, Home : 0
    Servo 2 : Ring and Little, Home : 0
    Servo 3 : Middle Finger, Home : 180
    Servo 4 : Thumb, Home : 180
    Servo 5 : Thumb Angle, Home : 10

    """
    if len(angles) != 5:
        raise ValueError("The angles array must contain exactly 5 elements.")

    # Loop through each servo and send the angle
    for i in range(1, 6):  # Servo indices are from 1 to 5
        angle = angles[i-1]
        if not (0 <= angle <= 180):
            raise ValueError(f"Angle for servo {i} is out of range: {angle}. Must be between 0 and 180.")
        
        # Construct the URL for setting the servo angle
        url = f"http://{ip_address}/setServo?servo={i}&angle={angle}"
        
        # Send GET request to the ESP8266 server
        response = requests.get(url)
        
        # Check if the request was successful
        if response.status_code == 200:
            i=1
            #print(f"Servo {i} set to {angle} degrees.")
        else:
            print(f"Failed to set servo {i}. Status code: {response.status_code}")

# Example usage
# Replace '192.168.1.x' with your ESP8266's IP address
'''
ip = '192.168.1.121'
angles = [0, 0, 180, 180, 10]  
set_servo_angles(ip, angles)
sleep(.3)

Servo 1 : Index, Home : 0
Servo 2 : Ring and Little, Home : 0
Servo 3 : Middle Finger, Home : 180
Servo 4 : Thumb, Home : 180
Servo 5 : Thumb Angle, Home : 10

set_servo_angles(ip, [180, 0, 180, 180, 10])
sleep(.3)
set_servo_angles(ip, [0, 0, 180, 180, 10])
sleep(.3)
set_servo_angles(ip, [180, 0, 0, 180, 10])
sleep(.3)
set_servo_angles(ip, [0, 0, 180, 180, 10])
sleep(.3)
set_servo_angles(ip, [180, 180, 0, 180, 10])
sleep(.3)
set_servo_angles(ip, [0, 0, 180, 180, 10])
sleep(.3)
set_servo_angles(ip, [180, 180, 0, 0, 10])
sleep(.3)
set_servo_angles(ip, angles)
sleep(.3)
set_servo_angles(ip, [180, 180, 180, 0, 10])
sleep(.7)
set_servo_angles(ip, angles)
'''