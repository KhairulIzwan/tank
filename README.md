# TANK: ROS Packages

Create the basic ROS packages control for **Tank** type robot. Consist of:
* Vision Control,
* Motor Control, and
* RPLidar Control

## Basic Vision
#### Basic previewing of camera
###### ROS Image.msg --> OpenCV Image [Preview]
1. roscore
2. rosrun cv_camera cv_camera_node _device:=/dev/video0
3. rosrun tank tank_vision.py 38 80 10

###### OR

1. roslaunch tank tank_camera_robot.launch
2. roslaunch tank tank_camera_remote.launch
**required**: git clone xxxx packages

## Basic Navigation
#### Basic control of robot navigation using keyboard
###### Keyboard Controlled [tele-operation]
1. roscore
2. rosrun rosserial_arduino serial_node.py /dev/ttyACM0
3. rosrun tank tank_teleop_key.py

###### OR

1. roslaunch tank tank_motor_robot.launch
2. roslaunch tank tank_motor_remote.launch
**required**: git clone xxxx packages

## Basic RPLidar
######Simulataneous Localization and Mapping [SLAM]
1. roslaunch rpilidar_ros
**required**: git clone rplidar_ros packages

## Face Detector
######Face Detection [Haar-Cascade]
1. roscore
2. rosrun cv_camera cv_camera_node _device:=/dev/video0
3. rosrun tank tank_face_detection.py

###### OR

1. roslaunch tank tank_camera_face_detection.launch
**required**: git clone xxxx packages

## Basic Vision
#### Color Trackbar -- useful in finding tracked color
###### ROS Image.msg --> OpenCV Image [Preview]
1. roscore
2. rosrun cv_camera cv_camera_node _device:=/dev/video0
3. rosrun tank tank_color_range_trackbar.py

###### OR

1. roslaunch tank tank_color_range_trackbar.launch
**required**: git clone xxxx packages
