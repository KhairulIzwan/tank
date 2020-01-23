#!/usr/bin/env python3

#Title: Python subscriber template
#Author: https://www.intorobotics.com/template-for-a-ros-subscriber-in-python/
#Description: Template for a ROS Publisher in Python

#remove or add the library/libraries for ROS
import rospy
import time
 
#remove or add the message type
from std_msgs.msg import String, Float32, Image, LaserScan, Int32
from geometry_msgs.msg import Twist
 
#define function/functions to provide the required functionality
def name_callback(msg):
	make_something_here_with msg.data
	rospy.loginfo("I heard %s", msg.data)

if __name__=='__main__':
	#Add here the name of the ROS. In ROS, names are unique named.
	rospy.init_node('THE_NAME_OF_THE_NODE')

	#subscribe to a topic using rospy.Subscriber class
	sub=rospy.Subscriber('TOPIC_NAME', TOPIC_MESSAGE_TYPE, name_callback)
	rospy.spin()
