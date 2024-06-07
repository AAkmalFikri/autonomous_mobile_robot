#!/usr/bin/env python3

import serial
import math
import time

class serialCommunication:

	serial.port = serial.Serial()

	def __init__(self, port = "/dev/ttyUSB0"):
		#variable odometry data
		#set serial port serial stm32f4 master robot
		self.serial_port = serial.Serial(port = port,
										 baudrate = 115200,
										 timeout = 0.1
										)

	def write_serial_communication(self, data):
		self.serial_port.write(data.encode())

	def read_serial_communication(self):
		response = self.serial_port.readline()
		
		if len(response) != 0:
			response = response.decode()
			status = 1
		else :
			status = 0
		
		return response, status


