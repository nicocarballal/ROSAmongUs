#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import sys
import tf2_ros
import tf2_msgs.msg

import geometry_msgs
from geometry_msgs.msg import Twist
from among_us.msg import RobotTaskUpdate
from visualization_msgs.msg import Marker, MarkerArray


#Define the method which contains the main functionality of the node.
def controller(robot_frame, target_frame):
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

  tfBuffer = tf2_ros.Buffer()
  tfListener = tf2_ros.TransformListener(tfBuffer)

  pub = rospy.Publisher(robot_frame + '/cmd_vel', Twist, queue_size=10)
  
  
  # Create a timer object that will sleep long enough to result in
  # a 10Hz publishing rate
  r = rospy.Rate(10) # 10hz

  K1 = .3
  K2 = .3
  # Loop until the node is killed with Ctrl-C

  keepPath = True

  while not rospy.is_shutdown():
    
    #TODO: Replace 'SOURCE FRAME' and 'TARGET FRAME' with the appropriate TF frame names.
    #trans = tfBuffer.lookup_transform('SOURCE FRAME', 'TARGET FRAME', rospy.Time())
    try: 
      trans = tfBuffer.lookup_transform(robot_frame, target_frame, rospy.Time()) ##MAKE CHANGES HERE TO ARGUMENTS
      print('check')
      # Process trans to get your state error
      # Generate a control command to send to the robot

      print(trans)
      
      translation_x_error = trans.transform.translation.x * K1
      rotation_error = trans.transform.translation.y 

      '''
      if abs(translation_x_error) < .01:
        publish_task_update(robot_frame, False, True)
        break
      '''
    
        ### Publish to 'robot_name/task' with need_path_update!

      control_command = Twist()

      control_command.linear.x = translation_x_error
      control_command.angular.z = rotation_error * -K2

      print(control_command)

      #TODO: Generate this

      #################################### end your code ###############
      pub.publish(control_command)

      print('Check')
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
      print(e)


  
    # Use our rate object to sleep until it is time to publish again
    r.sleep()
    
      


def publish_task_update(robot_name, need_task_update, need_path_update):
    pub_update = rospy.Publisher(robot_name + '/taskUpdate', RobotTaskUpdate, queue_size=10)
    updateMsg = RobotTaskUpdate()
    updateMsg.robot_name = robot_name
    updateMsg.need_task_update = False
    updateMsg.need_path_update = True
    pub_update.publish(updateMsg)

# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method

  #Run this program as a new node in the ROS computation graph 
  #called /turtlebot_controller.
  print("Robot controller started.")
  taskLocations = {"task1": (12, 12), "task2": (8, 5), "task3": (12, 1), "task4": (15,1), "task5": (1,6.5), 
    "task6": (9, 6.5), "task7": (15.5, 5), "task8": (15.5, 8.5), "task9": (22, 7), "task10": (18,10)}

  robot0Tasks = ["task4", "task7", "task1", "task3"]
  robot1Tasks = ["task5", "task3", "task2", "task9"]
  robot2Tasks = ["task6", "task5", "task10", "task1"]
  robot3Tasks = ["task7", "task1", "task2", "task4"]
  robot4Tasks = ["task8", "task9", "task4", "task2"]
  robot5Tasks = ["task9", "task8", "task3", "task6"]
  robot6Tasks = ["task10", "task6", "task1", "task8"]
  robot7Tasks = ["task1", "task4", "task5", "task10"]

  robotTasks = {"robot0": robot0Tasks, "robot1": robot1Tasks, "robot2": robot2Tasks, 
  "robot3": robot3Tasks, "robot4": robot4Tasks, "robot5": robot5Tasks, "robot6": robot6Tasks, "robot7": robot7Tasks}

  

  rospy.init_node('among_us_controller', anonymous=True)

  try:
    controller('robot0', 'robot0goal')
  except rospy.ROSInterruptException:
    pass
