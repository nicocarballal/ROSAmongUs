
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


def parameter_server(robot_name):

    #Create a new instance of the rospy.Subscriber object which we can 
    #use to receive messages of type std_msgs/String from the topic /chatter_talk.
    #Whenever a new message is received, the method callback() will be called
    #with the received message as its first argument.
    while not rospy.is_shutdown():
	    message = rospy.wait_for_message("/" + robot_name + "/odom", Odometry)

	    robotX = message.pose.pose.position.x
	    robotY = message.pose.pose.position.y
	    rospy.set_param('/' + robot_name + '/positionX', robotX)
	    rospy.set_param('/' + robot_name + '/positionY', robotY)
	    sleep(2)





if __name__ == '__main__':
    robot_name = sys.argv[1]
    print('Initialized parameter server for: ' + robot_name)
    rospy.init_node('parameter_server', anonymous=True)
    parameter_server(robot_name)