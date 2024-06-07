#!/usr/bin/env python3

from inputs import get_gamepad
import threading
import time
import rospy
from std_msgs.msg import String


class gamepad_joystick:

	def __init__(self):

		rospy.init_node('robotkece_teleop_gamepad_node', anonymous=True)
		self.pub = rospy.Publisher('gamepad', String, queue_size=10)

		self.hatx = 0
		self.haty = 0
		self.x = 127
		self.y = 127
		self.ins = 's'

		self.moveIns = {
			'f':'Robot Moving Forward',
			'b':'Robot Moving Backward',
			'l':'Robot Moving Left',
			'r':'Robot Moving Right',
			'C':'Robot Moving Counter Clockwise',
			'c':'Robot Moving Clockwise',
			's':'Robot Stop Moving',
			}

	def get_joy(self):
#		while True:
#			try:
#				events = get_gamepad()
#				for event in events:
#					if event.ev_type == 'Absolute':
#						if event.code == 'ABS_HAT0Y':
#							self.haty = event.state
#						elif event.code == 'ABS_HAT0X':
#							self.hatx = event.state
#						elif event.code == 'ABS_Y':
#							self.y = event.state
#						elif event.code == 'ABS_X':
#							self.x = event.state
#						else:
#							break
#					else:
#						break
#			except Exception as e:
#				print(e)
		events = get_gamepad()
		for event in events:
			if event.ev_type == 'Absolute':
				if event.code == 'ABS_HAT0Y':
					self.haty = event.state
				elif event.code == 'ABS_HAT0X':
					self.hatx = event.state
				elif event.code == 'ABS_Y':
					self.y = event.state
				elif event.code == 'ABS_X':
					self.x = event.state
				else:
					break
			else:
				break

	def main(self):

		while not rospy.is_shutdown():
			self.get_joy()
			if self.haty == -1:
				self.ins = 'f'
			elif self.haty == 1:
				self.ins = 'b'
			elif self.hatx == 1:
				self.ins = 'c'
			elif self.hatx == -1:
				self.ins = 'C'
			elif self.y < 50:
				self.ins = 'f'
			elif self.y > 205:
				self.ins = 'b'
			elif self.x < 50:
				self.ins = 'l'
			elif self.x > 205:
				self.ins = 'r'
			else:
				self.ins = 's'

			print(self.moveIns[self.ins])

			self.pub.publish(self.ins)

if __name__ == "__main__":
	joystick = gamepad_joystick()
#	joy = threading.Thread(target=joystick.get_joy, args=())
#	joy.start()

	try:
		joystick.main()

	except rospy.ROSInterruptException:
		pass


