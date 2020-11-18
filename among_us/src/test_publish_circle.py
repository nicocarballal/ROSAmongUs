
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


#Define the method which contains the main functionality of the node.


def talker():

  markerArray = MarkerArray()
  marker = Marker()
  marker.header.frame_id = "world"
  marker.ns = "my_namespace"
  marker.id = 1
  marker.type = marker.CYLINDER
  marker.action = marker.ADD
  marker.pose.position.x = 7.41
  marker.pose.position.y = 7.41
  marker.pose.position.z = 0
  marker.pose.orientation.x = 0.0
  marker.pose.orientation.y = 0.0
  marker.pose.orientation.z = 0.0
  marker.pose.orientation.w = 1.0
  marker.scale.x = 1
  marker.scale.y = 1
  marker.scale.z = 1
  marker.color.a = 1.0 # Don't forget to set the alpha!
  marker.color.r = 0.0
  marker.color.g = 1.0
  marker.color.b = 0.0
  marker.lifetime.secs = 1000
  marker.lifetime.nsecs = 1000

  if len(markerArray.markers) > 0:
    markerArray.markers.pop(0)
  markerArray.markers.append(marker)

  print(markerArray)
  pubMarker = rospy.Publisher('visualization_marker', Marker, queue_size=10)

  pub = rospy.Publisher('/stdr_server/sources_visualization_markers', MarkerArray, queue_size=10)

  while not rospy.is_shutdown():
    # Construct a string that we want to publish
    # (In Python, the "%" operator functions similarly
    #  to sprintf in C or MATLAB)
    msg = raw_input("Please enter a line of text and press <Enter>:")
    pub.publish(markerArray)



  
  # Create a timer object that will sleep long enough to result in
  # a 10Hz publishing rate
  r = rospy.Rate(10) # 10hz
      
# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method

  #Run this program as a new node in the ROS computation graph 
  #called /talker.
  rospy.init_node('talker', anonymous=True)
  
  try:
    talker()
  except rospy.ROSInterruptException: pass