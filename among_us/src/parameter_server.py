
#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import sys


from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
import random 
from time import sleep
import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs
from among_us.msg import RobotTaskUpdate
from a_star_function import a_star_function
import math
import time
from time import sleep
from imposter_search import find_nearest_robot, kill_nearest_robot




#Define the method which contains the main functionality of the node.


def parameter_server():
	timeout = 1
    while not rospy.is_shutdown():
      try:
      	robot0 = rospy.wait_for_message("/robot0/odom", Odometry, timeout)
      	robot0X = robot0.pose.pose.position.x
        robot0Y = robot0.pose.pose.position.Y
        rospy.set_param('robot0/positionX', robot0X)
        rospy.set_param('robot0/positionY', robot0Y)
      except:
      	pass
      try:
      	robot1 = rospy.wait_for_message("/robot1/odom", Odometry, timeout)
      	robot1X = robot0.pose.pose.position.x
        robot1Y = robot0.pose.pose.position.Y
        rospy.set_param('robot1/positionX', robot0X)
        rospy.set_param('robot1/positionY', robot0Y)
      except:
      	pass
      try:
      	robot2 = rospy.wait_for_message("/robot2/odom", Odometry, timeout)
      	robot2X = robot0.pose.pose.position.x
        robot2Y = robot0.pose.pose.position.Y
        rospy.set_param('robot2/positionX', robot0X)
        rospy.set_param('robot2/positionY', robot0Y)
      except:
      	pass
      try:
      	robot3 = rospy.wait_for_message("/robot3/odom", Odometry, timeout)
      	robot3X = robot0.pose.pose.position.x
        robot3Y = robot0.pose.pose.position.Y
        rospy.set_param('robot3/positionX', robot0X)
        rospy.set_param('robot3/positionY', robot0Y)
      except:
      	pass
      try:
      	robot4 = rospy.wait_for_message("/robot4/odom", Odometry, timeout)
      	robot4X = robot0.pose.pose.position.x
        robot4Y = robot0.pose.pose.position.Y
        rospy.set_param('robot4/positionX', robot0X)
        rospy.set_param('robot4/positionY', robot0Y)
      except:
      	pass
      try:
      	robot5 = rospy.wait_for_message("/robot5/odom", Odometry, timeout)
      	robot5X = robot0.pose.pose.position.x
        robot5Y = robot0.pose.pose.position.Y
        rospy.set_param('robot5/positionX', robot0X)
        rospy.set_param('robot5/positionY', robot0Y)
      except:
      	pass
      try:
      	robot6 = rospy.wait_for_message("/robot6/odom", Odometry, timeout)
      	robot6X = robot0.pose.pose.position.x
        robot6Y = robot0.pose.pose.position.Y
        rospy.set_param('robot6/positionX', robot0X)
        rospy.set_param('robot6/positionY', robot0Y)
      except:
      	pass
      try:
      	robot7 = rospy.wait_for_message("/robot7/odom", Odometry, timeout)
      	robot7X = robot0.pose.pose.position.x
        robot7Y = robot0.pose.pose.position.Y
        rospy.set_param('robot7/positionX', robot0X)
        rospy.set_param('robot7/positionY', robot0Y)
      except:
      	pass