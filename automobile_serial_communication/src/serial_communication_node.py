#!/usr/bin/env python3

import os
import serial
import time

import rospy
import tf
from serial_communication import * 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import String

class serialCommunicationRobot:

	def __init__(self):
		#init node
		rospy.init_node("ros_serial_communication_node")

		#get node name
		self.node_name = rospy.get_name()

		#get ros params
		self.get_ros_params()

		#set variabel
		self.current_time = rospy.Time.now()
		self.last_time = rospy.Time.now()
		self.th = 0
		self.instruction_key = 's'
		self.instruction_vel_x = 's'
		self.instruction_vel_y = 's'
		self.instruction_ang_z = 's'
		self.instruction_robot = '*ssss#'

		# Create an Odometry instance
		self.serial_communication = serialCommunication(port = self.serial_port)

		#internal variables
		self.stop_request = False

        #create topics
		rospy.Subscriber("keyboard", String, self.callback_keyboard)
		rospy.Subscriber("gamepad", String, self.callback_gamepad)
		rospy.Subscriber("/cmd_vel", Twist, self.callback_otonom)
		# Print node status
		rospy.loginfo(self.node_name + " ready!")

	def get_ros_params(self):
		self.serial_port = rospy.get_param(self.node_name + '/serial_port','/dev/ttyUSB0')

	def callback_keyboard(self, data):
		self.instruction_key = str(data.data)

	def callback_gamepad(self, data):
		self.instruction_key = str(data.data)

	def callback_otonom(self, data):
		vel_x = data.linear.y * -1
		vel_y = data.linear.x
		ang_z = data.angular.z * -1

#		if vel_x < -0.2:
#			self.instruction_vel_x = 'L'
#		elif vel_x < -0.0 and vel_x >= -0.2:
#			self.instruction_vel_x = 'l'
#		elif vel_x > 0.0 and vel_x <= 0.2:
#			self.instruction_vel_x = 'r'
#		elif vel_x > 0.2:
#			self.instruction_vel_x = 'R'
#		else :
#			self.instruction_vel_x = 's'

		if vel_y < 0:
			self.instruction_vel_y = 's'
			if ang_z >= 0:
				self.instruction_ang_z = 'L'
			elif ang_z < 0:
				self.instruction_ang_z = 'l'
		elif vel_y >= 0:
			if ang_z < -0.35 or ang_z > 0.35:
				self.instruction_vel_y = 's'
				if ang_z < -0.35:
					self.instruction_ang_z = 'l'
				elif ang_z > 0.35:
					self.instruction_ang_z = 'L'
			else:
				if vel_y == 0:
					self.instruction_vel_y = 's'
				elif vel_y >= 0.005 and vel_y < 0.11:
					self.instruction_vel_y = 'A'
				elif vel_y >= 0.11 and vel_y < 0.125:
					self.instruction_vel_y = 'B'
				elif vel_y >= 0.125 and vel_y < 0.17:
					self.instruction_vel_y = 'C'
				elif vel_y >= 0.17 and vel_y < 0.21:
					self.instruction_vel_y = 'D'
				elif vel_y >= 0.21:
					self.instruction_vel_y = 'E'

				if ang_z == 0:
					self.instruction_ang_z = 's'
				elif ang_z <= -0.35:
					self.instruction_ang_z = 'l'
				elif ang_z <= -0.3 and ang_z > -0.35:
					self.instruction_ang_z = 'f'
				elif ang_z <= -0.25 and ang_z > -0.3:
					self.instruction_ang_z = 'e'
				elif ang_z <= -0.2 and ang_z > -0.25:
					self.instruction_ang_z = 'd'
				elif ang_z <= -0.15 and ang_z > -0.2:
					self.instruction_ang_z = 'c'
				elif ang_z <= -0.1 and ang_z > -0.15:
					self.instruction_ang_z = 'b'
				elif ang_z <= -0.05 and ang_z > -0.1:
					self.instruction_ang_z = 'a'
				elif ang_z >= 0.05 and ang_z < 0.1:
					self.instruction_ang_z = 'A'
				elif ang_z >= 0.1 and ang_z < 0.15:
					self.instruction_ang_z = 'B'
				elif ang_z >= 0.15 and ang_z < 0.2:
					self.instruction_ang_z = 'C'
				elif ang_z >= 0.2 and ang_z < 0.25:
					self.instruction_ang_z = 'D'
				elif ang_z >= 0.25 and ang_z < 0.3:
					self.instruction_ang_z = 'E'
				elif ang_z >= 0.3 and ang_z < 0.35:
					self.instruction_ang_z = 'F'
				elif ang_z >= 0.35:
					self.instruction_ang_z = 'L'

		if vel_x <= -0.21:
			self.instruction_vel_x = 'e'
		elif vel_x <= -0.17 and vel_x > -0.21:
			self.instruction_vel_x = 'd'
		elif vel_x <= -0.125 and vel_x > -0.17:
			self.instruction_vel_x = 'c'
		elif vel_x <= -0.11 and vel_x > -0.125:
			self.instruction_vel_x = 'b'
		elif vel_x <= -0.005 and vel_x > -0.11:
			self.instruction_vel_x = 'a'
		elif vel_x >= 0.005 and vel_x < 0.11:
			self.instruction_vel_x = 'A'
		elif vel_x >= 0.11 and vel_x < 0.125:
			self.instruction_vel_x = 'B'
		elif vel_x >= 0.125 and vel_x < 0.17:
			self.instruction_vel_x = 'C'
		elif vel_x >= 0.17 and vel_x < 0.21:
			self.instruction_vel_x = 'D'
		elif vel_x >= 0.21:
			self.instruction_vel_x = 'E'
		else:
			self.instruction_vel_x = 's'

#		if vel_y <= -0.21:
#			self.instruction_vel_y = 'e'
#		elif vel_y <= -0.17 and vel_y > -0.21:
#			self.instruction_vel_y = 'd'
#		elif vel_y <= -0.125 and vel_y > -0.17:
#			self.instruction_vel_y = 'c'
#		elif vel_y <= -0.11 and vel_y > -0.125:
#			self.instruction_vel_y = 'b'
#		elif vel_y <= -0.005 and vel_y > -0.11:
#			self.instruction_vel_y = 'a'
#		elif vel_y >= 0.005 and vel_y < 0.11:
#			self.instruction_vel_y = 'A'
#		elif vel_y >= 0.11 and vel_y < 0.125:
#			self.instruction_vel_y = 'B'
#		elif vel_y >= 0.125 and vel_y < 0.17:
#			self.instruction_vel_y = 'C'
#		elif vel_y >= 0.17 and vel_y < 0.21:
#			self.instruction_vel_y = 'D'
#		elif vel_y >= 0.21:
#			self.instruction_vel_y = 'E'
#		else:
#			self.instruction_vel_y = 's'

#		if ang_z <= -0.6:
#			self.instruction_ang_z = 'l'
#		elif ang_z <= -0.55 and ang_z > -0.6:
#			self.instruction_ang_z = 'k'
#		elif ang_z <= -0.5 and ang_z > -0.55:
#			self.instruction_ang_z = 'j'
#		elif ang_z <= -0.45 and ang_z > -0.5:
#			self.instruction_ang_z = 'i'
#		elif ang_z <= -0.4 and ang_z > -0.45:
#			self.instruction_ang_z = 'h'
#		elif ang_z <= -0.35 and ang_z > -0.4:
#			self.instruction_ang_z = 'g'
#		elif ang_z <= -0.3 and ang_z > -0.35:
#			self.instruction_ang_z = 'f'
#		elif ang_z <= -0.25 and ang_z > -0.3:
#			self.instruction_ang_z = 'e'
#		elif ang_z <= -0.2 and ang_z > -0.25:
#			self.instruction_ang_z = 'd'
#		elif ang_z <= -0.15 and ang_z > -0.2:
#			self.instruction_ang_z = 'c'
#		elif ang_z <= -0.1 and ang_z > -0.15:
#			self.instruction_ang_z = 'b'
#		elif ang_z <= -0.05 and ang_z > -0.1:
#			self.instruction_ang_z = 'a'
#		elif ang_z >= 0.05 and ang_z < 0.1:
#			self.instruction_ang_z = 'A'
#		elif ang_z >= 0.1 and ang_z < 0.15:
#			self.instruction_ang_z = 'B'
#		elif ang_z >= 0.15 and ang_z < 0.2:
#			self.instruction_ang_z = 'C'
#		elif ang_z >= 0.2 and ang_z < 0.25:
#			self.instruction_ang_z = 'D'
#		elif ang_z >= 0.25 and ang_z < 0.3:
#			self.instruction_ang_z = 'E'
#		elif ang_z >= 0.3 and ang_z < 0.35:
#			self.instruction_ang_z = 'F'
#		elif ang_z >= 0.35 and ang_z < 0.4:
#			self.instruction_ang_z = 'G'
#		elif ang_z >= 0.4 and ang_z < 0.45:
#			self.instruction_ang_z = 'H'
#		elif ang_z >= 0.45 and ang_z < 0.5:
#			self.instruction_ang_z = 'I'
#		elif ang_z >= 0.5 and ang_z < 0.55:
#			self.instruction_ang_z = 'J'
#		elif ang_z >= 0.55 and ang_z < 0.6:
#			self.instruction_ang_z = 'K'
#		elif ang_z >= 0.6:
#			self.instruction_ang_z = 'L'
#		else:
#			self.instruction_ang_z = 's'

	def run(self):
		while not rospy.is_shutdown():

			self.instruction_robot = '*' + self.instruction_key + self.instruction_vel_x + self.instruction_vel_y + self.instruction_ang_z + '#'
			self.serial_communication.write_serial_communication(self.instruction_robot)
#			print(self.instruction_robot)


if __name__ == '__main__':

	robot = serialCommunicationRobot()

	try:
		robot.run()

	except rospy.ROSInterruptException:
		pass


