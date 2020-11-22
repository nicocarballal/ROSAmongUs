#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import tf2_ros
import sys

from geometry_msgs.msg import Twist

#Define the method which contains the main functionality of the node.
def controller():
  """
  Controls a robot whose position is denoted by robot_frame,
  to go to a position denoted by target_frame
  Inputs:
  - robot_frame: the tf frame of the robot base.
  - target_frame: the tf frame of the desired position.
  """

  ################################### YOUR CODE HERE ##############

  #Create a publisher and a tf buffer, which is primed with a tf listener
  #TODO: replace 'INPUT TOPIC' with the correct name for the ROS topic on which
  # the robot accepts velocity inputs.

  pub = rospy.Publisher('robot0/cmd_vel', Twist, queue_size=10)
  tfBuffer = tf2_ros.Buffer()
  tfListener = tf2_ros.TransformListener(tfBuffer)
  
  # Create a timer object that will sleep long enough to result in
  # a 10Hz publishing rate
  r = rospy.Rate(10) # 10hz

  K1 = 0.3
  K2 = 1
  # Loop until the node is killed with Ctrl-C
  while not rospy.is_shutdown():
    try:
      #TODO: Replace 'SOURCE FRAME' and 'TARGET FRAME' with the appropriate TF frame names.
      trans = tfBuffer.lookup_transform(robot_frames[0], task_frames[0], rospy.Time())
      # Process trans to get your state error
      # Generate a control command to send to the robot
      print(trans.transform.translation)
      translation_error = trans.transform.translation.x * K1
      rotation_error = trans.transform.translation.y 


      control_command = Twist()

      control_command.linear.x = translation_error
      control_command.angular.z = rotation_error * -K2
     

      #TODO: Generate this

      #################################### end your code ###############

      pub.publish(control_command)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      pass
    # Use our rate object to sleep until it is time to publish again
    r.sleep()

      
# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method

  #Run this program as a new node in the ROS computation graph 
  #called /turtlebot_controller.
  task_frames = ["task1", "task2", "task3", "task4", "task5", "task6", "task7", "task8", "task9", "task10"]
  robot_frames = ["robot0", "robot1", "robot2", "robot3", "robot4", "robot5", "robot6", "robot7", "robot8", "robot9"]
  rospy.init_node('among_us_controller', anonymous=True)

  try:
    controller()
  except rospy.ROSInterruptException:
    pass

