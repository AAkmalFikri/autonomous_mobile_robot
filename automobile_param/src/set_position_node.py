#!/usr/bin/env python3

import os, sys
import time
import math

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def get_pos (msg):
	print ("Pos X = " + str(msg.pose.pose.position.x))
	print ("Pos Y = " + str(msg.pose.pose.position.y))
	print ("Orient Z = " + str(msg.pose.pose.orientation.z))
	print ("Orient W = " + str(msg.pose.pose.orientation.w))

	with open("/home/robotkece2gen3/catkin_ws/src/autonomous_robotkece/map/maps/" + sys.argv[1] + "/" + sys.argv[2] + ".txt", "w") as file:
		file.write(str(msg.pose.pose.position.x) + "\n")
		file.write(str(msg.pose.pose.position.y) + "\n")
		file.write(str(msg.pose.pose.orientation.z) + "\n")
		file.write(str(msg.pose.pose.orientation.w) + "\n")

if len(sys.argv) != 3:
	print("invalid argument")
else:
	rospy.init_node('set_position_node')
	sub = rospy.Subscriber ('/amcl_pose', PoseWithCovarianceStamped, get_pos)
	r = rospy.Rate(1) 
	while not rospy.is_shutdown(): 
		r.sleep()
		break



