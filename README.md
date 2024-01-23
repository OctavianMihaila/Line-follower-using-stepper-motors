# Line Follower Robot Project

## Description

This repository contains the code and documentation for a Line Follower Robot project. The robot utilizes stepper motors with microstepping control for precise movements and is powered by a 12V battery. Two infrared (IR) sensors are used to detect the tape on the track, enabling the robot to follow a designated path. Additionally, an inclination sensor is employed to detect ramps.

<div style="display: flex; justify-content: space-between;">
  <img src="https://github.com/OctavianMihaila/Line-follower-using-stepper-motors/blob/main/photo1.jpg?raw=true" alt="Photo 1" width="200"/>
  <img src="https://github.com/OctavianMihaila/Line-follower-using-stepper-motors/blob/main/photo2.jpg?raw=true" alt="Photo 2" width="200"/>
  <img src="https://github.com/OctavianMihaila/Line-follower-using-stepper-motors/blob/main/photo3.jpg?raw=true" alt="Photo 3" width="200"/>
</div>

### Video

[click](https://drive.google.com/file/d/1AFfCffdHiyz3Ny4kCnAHTGUHm5Ol00mJ/view)

## Implementation

The implementation of the Line Follower Robot is built upon the Arduino platform using the AccelStepper library to control the stepper motors. The code defines the necessary pins for two stepper motors, two infrared (IR) sensors for tape detection, an infrared obstacle sensor, a tilt sensor for ramp detection, and pins for power supply control and debugging.

### Microstepping and Movement Control

The implementation showcases the use of microstepping for precise control of the stepper motors. Microstepping helps achieve smoother movements and finer positioning. The microstepping configuration is set individually for both Motor 1 and Motor 2 using pins MS1, MS2, and MS3.

The AccelStepper library is employed to manage the acceleration and speed of the stepper motors. Speed adjustments are made dynamically based on the robot's state, such as turning left or right, to enhance maneuverability.

### Tilt Sensor and Ramp Detection

An inclination sensor (tilt sensor) is integrated to detect ramps. The code includes mechanisms to handle tilt sensor triggers due to vibrations, ensuring accurate detection of ramps. The robot adjusts its behavior when on a ramp, demonstrating adaptability to different terrains.

### Additional Features

The implementation incorporates features such as power supply control through a start/stop button, debugging pins to visualize the robot's state, and obstacle detection using IR sensors. The code structure includes functions for handling power supply, adjusting motor speed during turns, and determining the next movement based on IR sensor readings.

Overall, the implementation provides a robust foundation for a line-following robot with advanced control features, making it suitable for diverse environments and challenges.
