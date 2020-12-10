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
import math
import time
from time import sleep




#Define the method which contains the main functionality of the node.


def marker_creator():
    while not rospy.is_shutdown():
      create_tasks()
      tf_frames()



## Creates one task currently and publishes it to RVIZ
def create_tasks():
    markerArray = MarkerArray()
    i = 1
    for key in taskLocations:

      marker = Marker()
      marker.header.frame_id = "world"
      marker.ns = "my_namespace"
      marker.id = i 
      marker.type = marker.CYLINDER
      marker.action = marker.ADD
      marker.pose.position.x = taskLocations[key][0]
      marker.pose.position.y = taskLocations[key][1]
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
      markerArray.markers.append(marker)
      i = i + 1
    pub = rospy.Publisher('/tasks/markers', MarkerArray, queue_size=10)
    pub.publish(markerArray)
    r.sleep()

def tf_frames():
    pub = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=50)

    for key in taskLocations:
      t = geometry_msgs.msg.TransformStamped()
      t.header.frame_id = "map_static"
      t.header.stamp = rospy.Time.now()
      t.child_frame_id = key
      t.transform.translation.x = taskLocations[key][0]
      t.transform.translation.y = taskLocations[key][1]
      t.transform.translation.z = 0.0
      t.transform.rotation.x = 0.0
      t.transform.rotation.y = 0.0
      t.transform.rotation.z = 0.0
      t.transform.rotation.w = 1.0
      tfm = tf2_msgs.msg.TFMessage([t])
      pub.publish(tfm)
    r.sleep()

#Python's syntax for a main() method
if __name__ == '__main__':
    #Run this program as a new node in the ROS computation graph
    #called /listener_<id>, where <id> is a randomly generated numeric
    #string. This randomly generated name means we can start multiple
    #copies of this node without having multiple nodes with the same
    #name, which ROS doesn't allow.
    print('Task Markers getting created')

    taskLocations = {"task1": (12, 12), "task2": (8, 5), "task3": (12, 1), "task4": (15,1), "task5": (1,6.5), 
    "task6": (9, 7.5), "task7": (15.5, 5), "task8": (16, 8), "task9": (22, 7), "task10": (18,10)}

    rospy.init_node('task_markers', anonymous=True)


    r = rospy.Rate(10) # 10hz

    marker_creator()