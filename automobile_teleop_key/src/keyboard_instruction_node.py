#!/usr/bin/env python3

import rospy

from std_msgs.msg import String

import sys, select, termios, tty

first_msg = """
Control Your Robot!
---------------------------
Moving around:
   q    w    e
   a    s    d
w : Moving Robot Forward
s : Moving Robot Backward
a : Moving Robot Left
d : Moving Robot Right
q : Moving Robot CCW
e : Moving Robot CW

CTRL-C to quit
"""

moveBindings = {
		'w':'f',
		's':'b',
		'a':'l',
		'd':'r',
		'q':'C',
		'e':'c',
			}

moveIns = {
		'f':'Robot Moving Forward',
		'b':'Robot Moving Backward',
		'l':'Robot Moving Left',
		'r':'Robot Moving Right',
		'C':'Robot Moving Counter Clockwise',
		'c':'Robot Moving Clockwise',
			}

def getKey():
	tty.setraw(sys.stdin.fileno())
	rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
	if rlist:
		key = sys.stdin.read(1)
	else:
		key = ''

	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

if __name__=="__main__":
	settings = termios.tcgetattr(sys.stdin)
    
	rospy.init_node('robotkece_teleop', anonymous=True)
	pub = rospy.Publisher('keyboard', String, queue_size=10)

	status = 0

	try:
		print(first_msg)
		while(1):
			key = getKey()
			if key in moveBindings.keys():
				n += 1
				if n > 2:
					ins = moveBindings[key]
					print(moveIns[ins])
					if (status == 19):
						status = 0
						print(first_msg)
					status = (status + 1)
			else:
				n = 0
				ins = 's'
				if (key == '\x03'):
					break

			pub.publish(ins)

	except Exception as e:
		print(e)

	finally:
		ins = 's'
		pub.publish(ins)

	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
