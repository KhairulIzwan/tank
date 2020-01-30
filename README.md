# TANK: ROS Packages

Create the basic ROS packages control for **Tank** type robot. Consist of:
* Vision Control,
* Motor Control, and
* RPLidar Control

## Basic Vision
###### ROS Image.msg --> OpenCV Image [Preview]
1. roslaunch tank tank_camera_robot.launch
2. roslaunch tank tank_camera_remote.launch
**required**: git clone xxxx packages

## Basic Navigation
###### Keyboard Controlled [tele-operation]
1. roslaunch tank tank_motor_robot.launch
2. roslaunch tank tank_motor_remote.launch
**required**: git clone xxxx packages

## Basic RPLidar
######Simulataneous Localization and Mapping [SLAM]
1. roslaunch rpilidar_ros
**required**: git clone rplidar_ros packages

## Face Detector
######Face Detection [Haar-Cascade]
1. roslaunch tank tank_camera_face_detection.launch
**required**: git clone xxxx packages
