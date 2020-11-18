
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


#Define the method which contains the main functionality of the node.


def taskmaster():
    while not rospy.is_shutdown():
        create_tasks()


## Creates one task currently and publishes it to RVIZ
def create_tasks():
    markerArray = MarkerArray()
    marker = Marker()
    marker.header.frame_id = "world"
    marker.ns = "my_namespace"
    marker.id = 1
    marker.type = marker.CYLINDER
    marker.action = marker.ADD
    marker.pose.position.x = 3
    marker.pose.position.y = 3
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 0
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.5
    marker.color.a = 1
    marker.color.r = 1
    marker.color.g = 1
    marker.color.b = 0
    marker.lifetime.secs = 1000
    marker.lifetime.nsecs = 1000
    pub = rospy.Publisher('/tasks/markers', MarkerArray, queue_size=10)
    markerArray.markers.append(marker)
    pub.publish(markerArray)

#Python's syntax for a main() method
if __name__ == '__main__':
    #Run this program as a new node in the ROS computation graph
    #called /listener_<id>, where <id> is a randomly generated numeric
    #string. This randomly generated name means we can start multiple
    #copies of this node without having multiple nodes with the same
    #name, which ROS doesn't allow.
    rospy.init_node('taskmaster', anonymous=True)

    taskmaster()
