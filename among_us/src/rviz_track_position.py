
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


def callback(message):

  idx = robotArray.index(message.child_frame_id)
  marker = Marker()
  marker.header.frame_id = "world"
  marker.ns = "my_namespace"
  marker.id = idx
  marker.type = marker.CYLINDER
  marker.action = marker.ADD
  marker.pose.position.x = message.pose.pose.position.x
  marker.pose.position.y = message.pose.pose.position.y
  marker.pose.position.z = message.pose.pose.position.z
  marker.pose.orientation.x = message.pose.pose.orientation.x
  marker.pose.orientation.y = message.pose.pose.orientation.y
  marker.pose.orientation.z = message.pose.pose.orientation.z
  marker.pose.orientation.w = message.pose.pose.orientation.w
  marker.scale.x = 0.5
  marker.scale.y = 0.5
  marker.scale.z = 0.5
  marker.color.a = colorArray[idx][0]# Don't forget to set the alpha!
  marker.color.r = colorArray[idx][1]
  marker.color.g = colorArray[idx][2]
  marker.color.b = colorArray[idx][3]
  marker.lifetime.secs = 1000
  marker.lifetime.nsecs = 1000

  if len(markerArray.markers) > 4:
    markerArray.markers.pop(0)
  markerArray.markers.append(marker)

  pub = rospy.Publisher('/stdr_server/sources_visualization_markers', MarkerArray, queue_size=10)

  sleep(.1)
  pub.publish(markerArray)



def listener():

    #Create a new instance of the rospy.Subscriber object which we can 
    #use to receive messages of type std_msgs/String from the topic /chatter_talk.
    #Whenever a new message is received, the method callback() will be called
    #with the received message as its first argument.
    rospy.Subscriber("robot0/odom", Odometry, callback)
    rospy.Subscriber("robot1/odom", Odometry, callback)
    rospy.Subscriber("robot2/odom", Odometry, callback)
    rospy.Subscriber("robot3/odom", Odometry, callback)

    #Wait for messages to arrive on the subscribed topics, and exit the node
    #when it is killed with Ctrl+C
    rospy.spin()


#Python's syntax for a main() method
if __name__ == '__main__':
    robotArray = ["robot0", "robot1", "robot2", "robot3"]
    colorArray= [[1, .5, .5, .5], [1, 1, 0, 0], [1, 0, 1, 0], [1, 0, 0, 1]]
    markerArray = MarkerArray()
    #Run this program as a new node in the ROS computation graph
    #called /listener_<id>, where <id> is a randomly generated numeric
    #string. This randomly generated name means we can start multiple
    #copies of this node without having multiple nodes with the same
    #name, which ROS doesn't allow.
    rospy.init_node('listener', anonymous=True)

    listener()
