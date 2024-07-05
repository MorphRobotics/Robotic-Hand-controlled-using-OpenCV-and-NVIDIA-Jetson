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

        cv2.imshow('Hand Tracking', image)  # Display the image with landmarks and positions
        if cv2.waitKey(1) & 0xFF == ord('q'):  # If 'q' key is pressed, break the loop
            break

cam.release()  # Release the webcam
cv2.destroyAllWindows()  # Close all OpenCV windows

