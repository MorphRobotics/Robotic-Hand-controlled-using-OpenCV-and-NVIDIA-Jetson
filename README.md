# Robotic-Hand-controlled-using-OpenCV-and-NVIDIA-Jetson

# Hardware needed
1. Arduino Uno
2. 5 Finger Servo controlled robotic hand
3. PCA9685 Servo Controller
4. NVIDIA Jetson Nano
5. Camera (Realsense or Logitech C270 recommended)

![Screenshot from 2024-07-05 00-17-35](https://github.com/MorphRobotics/Robotic-Hand-controlled-using-OpenCV-and-NVIDIA-Jetson/assets/104451879/df39e66c-6077-4e7d-bd87-33a0b335012f)


# The wiring for the project can be seen in the following image

![image](https://github.com/MorphRobotics/Robotic-Hand-controlled-using-OpenCV-and-NVIDIA-Jetson/assets/104451879/9f620495-f903-4411-a6ff-3ad4b2c7b620)

   

# To get started you will need to install mediapipe, opencv , pyFirmata and numpy

```
pip install mediapipe
pip install opencv-python
pip install pyFirmata
```
Now upload the Arduino script StandardFirmata.ino and then upload the 5fingerservocontrol.ino code

# Now in your terminal run the python script for OpenCV servo control

```
python3 cv_servocontrol.py
```
![Hand Tracking_screenshot_04 07 2024](https://github.com/MorphRobotics/Robotic-Hand-controlled-using-OpenCV-and-NVIDIA-Jetson/assets/104451879/00c6285d-079b-4422-bfb2-bbf58d39aa18)

# Explaining the Python code 
``` python
import cv2
import mediapipe as mp
import math
import numpy as np
import serial
import time

# Initialize serial communication with Arduino
arduino = serial.Serial('/dev/ttyACM1', 57600, timeout=1)
time.sleep(2)  # wait for the serial connection to initialize

# MediaPipe utilities for drawing and hand tracking
mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

# Webcam setup
wCam, hCam = 640, 480
cam = cv2.VideoCapture(4)  # Open the webcam (use appropriate index for your camera)
cam.set(3, wCam)  # Set the width of the webcam frame
cam.set(4, hCam)  # Set the height of the webcam frame

# Function to map a value from one range to another
def map_value(value, in_min, in_max, out_min, out_max):
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

# Use MediaPipe Hands solution
with mp_hands.Hands(
        model_complexity=0,  # Complexity of the hand landmark model
        min_detection_confidence=0.5,  # Minimum confidence for hand detection
        min_tracking_confidence=0.5) as hands:  # Minimum confidence for hand tracking

    while cam.isOpened():  # While the webcam is open
        success, image = cam.read()  # Read a frame from the webcam
        if not success:
            continue  # If the frame is not read successfully, continue to the next iteration

        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # Convert the image from BGR to RGB
        results = hands.process(image)  # Process the image to detect hands
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)  # Convert the image back to BGR

        lmList = []  # List to store landmarks
        if results.multi_hand_landmarks:  # If hand landmarks are detected
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(
                    image,
                    hand_landmarks,
                    mp_hands.HAND_CONNECTIONS,  # Draw hand connections
                    mp_drawing.DrawingSpec(color=(121, 22, 76), thickness=2, circle_radius=4),  # Customize landmark style
                    mp_drawing.DrawingSpec(color=(250, 44, 250), thickness=2, circle_radius=2)  # Customize connection style
                )

                for id, lm in enumerate(hand_landmarks.landmark):  # Enumerate through the landmarks
                    h, w, c = image.shape  # Get image dimensions
                    cx, cy = int(lm.x * w), int(lm.y * h)  # Convert normalized coordinates to pixel values
                    lmList.append([id, cx, cy])  # Append landmark id and coordinates to the list
```

This code initializes the necessary libraries and hardware for tracking hand movements and communicating the data to an Arduino. It imports the required libraries (OpenCV for image processing, MediaPipe for hand tracking, and serial for Arduino communication), sets up a serial connection with an Arduino, and initializes the MediaPipe hand tracking utilities. The webcam is configured to capture frames of a specified width and height.

A function `map_value` is defined to map a value from one range to another. The main logic for hand tracking is enclosed in a `with` statement that initializes the MediaPipe `Hands` solution with specified parameters for model complexity, detection confidence, and tracking confidence.

Inside a `while` loop, the code captures frames from the webcam, converts them from BGR to RGB, and processes them to detect hand landmarks. If hand landmarks are detected, the landmarks and hand connections are drawn on the image using customized styles. The landmarks' normalized coordinates are converted to pixel values and stored in a list `lmList`. The loop continues capturing and processing frames as long as the webcam is open, allowing for real-time hand tracking and data collection for further processing or communication with the Arduino.

``` python
  if lmList:
            fingers = [4, 8, 12, 16, 20]  # Thumb, Index, Middle, Ring, Pinky
            positions = []
            wrist_x, wrist_y = lmList[0][1], lmList[0][2]  # Get wrist coordinates

            for i, finger_id in enumerate(fingers):
                x, y = lmList[finger_id][1], lmList[finger_id][2]  # Get x and y coordinates of the finger
                cv2.circle(image, (x, y), 15, (255, 255, 255))  # Draw a circle at the fingertip

                # Draw a line from the fingertip to the wrist
                cv2.line(image, (wrist_x, wrist_y), (x, y), (255, 0, 0), 3)

                # Calculate the distance between the fingertip and the wrist
                length = math.hypot(x - wrist_x, y - wrist_y)

                if i == 0:  # Special case for the thumb
                    position = map_value(length, 50, 220, 180, 0)  # Invert the max position
                else:
                    position = map_value(length, 50, 220, 0, 180)

                positions.append(position)  # Append the position to the list
                cv2.putText(image, str(position), (50 + 100 * i, 60), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 0))  # Display the position

            # Send positions to Arduino
            data = ','.join(map(str, positions)) + '\n'  # Create a comma-separated string of positions
            arduino.write(data.encode())  # Send the data to Arduino
```
This code segment processes a list of hand landmarks (`lmList`) to determine the positions of the fingers relative to the wrist. The code first checks if `lmList` is not empty, then initializes a list of finger landmark IDs and an empty list for finger positions. The wrist coordinates are extracted from the first landmark. For each finger, the code retrieves its coordinates, draws a circle at the fingertip, and a line from the fingertip to the wrist on the `image`. The distance between the fingertip and the wrist is calculated, then mapped to a range (0-180 degrees) to determine the finger's position. These positions are displayed on the image and collected into a list. Finally, the positions are formatted into a comma-separated string and sent to an Arduino via serial communication.

``` python

        cv2.imshow('Hand Tracking', image)  # Display the image with landmarks and positions
        if cv2.waitKey(1) & 0xFF == ord('q'):  # If 'q' key is pressed, break the loop
            break

cam.release()  # Release the webcam
cv2.destroyAllWindows()  # Close all OpenCV windows
```
This segment of code is responsible for displaying the processed webcam feed with hand landmarks and positions overlaid, and handling the exit condition for the program. It uses OpenCV's `imshow` function to show the image with the hand tracking annotations in a window titled 'Hand Tracking'. The `waitKey` function waits for 1 millisecond for a key press. If the 'q' key is pressed, the loop is exited, effectively ending the program. After breaking the loop, the code releases the webcam resource with `cam.release()` and closes all OpenCV windows with `cv2.destroyAllWindows()`, ensuring that the program exits cleanly.

# Potential applications for this project include

1. Prosthetics Control: Enhance the functionality of prosthetic limbs by providing intuitive control through hand gestures.
2. Accessibility Tools: Enable people with disabilities to control devices, computers, or other assistive technologies using hand movements
3. Teleoperation: Control robots remotely using hand gestures, improving precision and intuitiveness for applications like surgery or hazardous material handling
4. Skill training: Use hand tracking for training in skills that require precise hand movements, such as surgical training or musical instrument learning.

FULL DEMO ON YOUTUBE: https://www.youtube.com/watch?v=gdE6s4S67Qc&t=4s&ab_channel=DozieUbosi

