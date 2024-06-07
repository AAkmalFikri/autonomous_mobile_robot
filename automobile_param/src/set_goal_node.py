#!/usr/bin/env python3

import os, sys
import time
import math

import rospy

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

x = 0.00
y = 0.00
z = 0.00
w = 0.00

def movebase_client():
    f = open("/home/robotkece2gen3/catkin_ws/src/autonomous_robotkece/map/maps/" + sys.argv[1] + "/" + sys.argv[2] + ".txt", 'r')
    data = f.readlines()
    f.close()
    x = float(data[0][:-1])
    y = float(data[1][:-1])
    z = float(data[2][:-1])
    w = float(data[3][:-1])
   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
   # Move 0.5 meters forward along the x axis of the "map" coordinate frame 
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
   # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.z = z
    goal.target_pose.pose.orientation.w = w

   # Sends the goal to the action server.
    client.send_goal(goal)

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("invalid argument")
    else:
        try:
           # Initializes a rospy node to let the SimpleActionClient publish and subscribe
            rospy.init_node('movebase_client_py')
            movebase_client()
            rospy.loginfo("Starting Navigation Move")
        except rospy.ROSInterruptException:
            rospy.loginfo("Error Navigation Move")
