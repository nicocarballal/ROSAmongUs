
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
import numpy as np



#Define the method which contains the main functionality of the node.
# Function to convert 
        
def initialize():

    while not rospy.is_shutdown():
        game_ending_checker()

def game_ending_checker():
    
    robotsWithTasks = rospy.get_param('robots_with_tasks')
    robotsWithTasks = robotsWithTasks.split()
    aliveCrewmates = rospy.get_param('alive_crewmates')
    aliveCrewmates = aliveCrewmates.split()

    if len(robotsWithTasks) == 0:
        while not rospy.is_shutdown():
            pub = rospy.Publisher('game_ending_text', Marker, queue_size = 1)
            marker = Marker()
            marker.header.frame_id = "world"
            marker.ns = "my_namespace"
            marker.id = 12
            marker.type = marker.TEXT_VIEW_FACING
            marker.action = marker.ADD
            marker.pose.position.x = 12
            marker.pose.position.y = 8
            marker.pose.position.z = 1
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 1
            marker.pose.orientation.w = 0
            marker.scale.z = 1
              
            marker.color.a = 1 # Don't forget to set the alpha!
            marker.color.r = 0
            marker.color.g = 1
            marker.color.b = 0
            marker.lifetime.secs = 1000
            marker.lifetime.nsecs = 1000
            marker.text = "CREWMATES HAVE WON BY TASKS!"

            pub.publish(marker)
    elif len(aliveCrewmates) == 0:
        
        while not rospy.is_shutdown():
            pub = rospy.Publisher('game_ending_text', Marker, queue_size = 1)
            marker = Marker()
            marker.header.frame_id = "world"
            marker.ns = "my_namespace"
            marker.id = 12
            marker.type = marker.TEXT_VIEW_FACING
            marker.action = marker.ADD
            marker.pose.position.x = 12
            marker.pose.position.y = 10
            marker.pose.position.z = 1
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 1
            marker.pose.orientation.w = 0
            marker.scale.z = 1
              
            marker.color.a = 1 # Don't forget to set the alpha!
            marker.color.r = 0
            marker.color.g = 1
            marker.color.b = 0
            marker.lifetime.secs = 1000
            marker.lifetime.nsecs = 1000
            marker.text = "IMPOSTORS HAVE KILLED EVERYONE!"

            pub.publish(marker)
    sleep(5)








if __name__ == '__main__':
    print('Initialized game ending checkers.')
    rospy.init_node('game_ending_checker', anonymous=True)
    initialize()