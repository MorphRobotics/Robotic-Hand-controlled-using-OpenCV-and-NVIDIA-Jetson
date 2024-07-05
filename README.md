# Robotic-Hand-controlled-using-OpenCV-and-NVIDIA-Jetson

Hardware needed
1. Arduino Uno
2. 5 Finger Servo controlled robotic hand
3. PCA9685 Servo Controller
4. NVIDIA Jetson Nano
5. Camera (Realsense or Logitech C270 recommended)

The wiring for the project can be seen in the following image

![image](https://github.com/MorphRobotics/Robotic-Hand-controlled-using-OpenCV-and-NVIDIA-Jetson/assets/104451879/9f620495-f903-4411-a6ff-3ad4b2c7b620)

   

To get started you will need to install mediapipe, opencv , pyFirmata and numpy

```
pip install mediapipe
pip install opencv-python
pip install pyFirmata
```
First upload the Arduino script StandardFirmata.ino and then upload the 5fingerservocontrol.ino code

Now in your terminal run the python script for OpenCV servo control

```
python3 cv_servocontrol.py
```
![Hand Tracking_screenshot_04 07 2024](https://github.com/MorphRobotics/Robotic-Hand-controlled-using-OpenCV-and-NVIDIA-Jetson/assets/104451879/00c6285d-079b-4422-bfb2-bbf58d39aa18)
