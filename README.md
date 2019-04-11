# Horizontal Travel Robot Arm (HTA0)


## Overview

Learn about this robot here and perhaps buy a kit:  https://www.fdxlabs.com/products/horizontal-travel-robot-arm-hta0/

Overview of the Robot: https://medium.com/@pacogarcia3/computer-vision-pick-and-place-for-toys-using-raspberry-pi-and-arduino-68a04874c039

See the robot in action: https://youtu.be/f3s_uub4P6Q


## Initial Camera Calibration

**initial_camera_calibration.py** follows very closely the OpenCV documentation and saves the output matrixes.

**camera_data** is where the camera calibration files are stored.

**initial_perspective_calibration.py** enables you to calibrate the camera to the perspective AND enables you to check the accurac of your calibration.

Read about the process to calibrate the camera here: https://www.fdxlabs.com/calculate-x-y-z-real-world-coordinates-from-a-single-camera-using-opencv/

## Image Detection and XYZ calculation

**image_recognition_singlecam.py** enables you to do background extraction and object detection.

**camera_realdworldxyz.py** takes the u,v pixel points of the object detected and translates it to realy world coordinates (for the arm to grab).

## Arm Control

**arduino_sketch** contains the arduino sketch that controls the arm.  This folder also contains the formulas used to translate desired Y, Z position into servo angles.

Read about the calculation of desired Y, Z position here: https://www.fdxlabs.com/converting-y-z-coordinates-into-angles-for-a-two-axis-robot-arm/

**commands_arduino.py** is where the most commonly used operations of the arm are defined for easy reference from the main_loop.py.

## Running the Program

**main_loop.py** this is the main loop of the program and is primarily a "camera Loop".  It has ample opportunity to be simplified for better readability (and also better camera refresh).

**main.py** this is the main file used to run the program, and it enables you to switch between Testing, Calibration and Running the Pick and Place program easily as you work on your setup.
