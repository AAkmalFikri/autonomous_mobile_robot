#!/usr/bin/env python3

import os
import serial
import time
import math

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

roll = pitch = yaw = 0.0

def get_rotation (msg):
	global roll, pitch, yaw 
	orientation_q = msg.pose.pose.orientation 
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	print('X =',msg.pose.pose.position.x, 'Y =',msg.pose.pose.position.y, 'Yaw =',math.degrees(yaw))

rospy.init_node('my_quaternion_to_euler')

sub = rospy.Subscriber ('/rtabmap/localization_pose', PoseWithCovarianceStamped, get_rotation) # geometry_msgs/PoseWithCovariance pose

r = rospy.Rate(1) 
while not rospy.is_shutdown(): 
	quat = quaternion_from_euler (roll, pitch,yaw) # print quat r.sleep()

