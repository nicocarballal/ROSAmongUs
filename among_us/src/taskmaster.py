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




#Define the method which contains the main functionality of the node.


def taskmaster():
    robot_name = sys.argv[1]
    print(robot_name)
    while not rospy.is_shutdown():
      task_manager(robot_name)
      #task_manager('robot2')

def task_manager(robot_name):
  try: 
    timeout = 1
    msg2 = rospy.wait_for_message("/" + robot_name + "/odom", Odometry, timeout)
    msg1 = rospy.wait_for_message("/" + robot_name + "/taskUpdate", RobotTaskUpdate, timeout)
    
    
    '''
    manhattanDistance = math.sqrt((X-taskX)**2 + (Y-taskY)**2)
    if (manhattanDistance < .01):
      if len(robotTasks[robot_name]) > 0:
        #robotTasks[msg.robot_name].pop(0)
        taskX = taskLocations[robotTasks[robot_name][0]][0]
        taskY = taskLocations[robotTasks[robot_name][0]][1]
        path = a_star_function(X, Y, taskX, taskY)
        robotPaths[robot_name] = path 
      else:
        return
    if len(robotPaths[robot_name]) > 0:
      waypoint = robotPaths[robot_name][2]
      print('Waypoint: ' + str(waypoint))
    else:
      return
    
    '''
    if msg1.need_path_update:
      if len(robotPaths[robot_name]) > 0:
        robotPaths[robot_name].pop(0)
        pub_update = rospy.Publisher(robot_name + '/taskUpdate', RobotTaskUpdate, queue_size=10)
        updateMsg = RobotTaskUpdate()
        updateMsg.robot_name = robot_name
        updateMsg.need_task_update = False
        updateMsg.need_path_update = False
        pub_update.publish(updateMsg)
      elif len(robotTasks[robot_name]) > 0:
        X = msg2.pose.pose.position.x
        Y = msg2.pose.pose.position.y
        X = round(X*4)/4
        Y = round(Y*4)/4
        taskX = taskLocations[robotTasks[robot_name][0]][0]
        taskY = taskLocations[robotTasks[robot_name][0]][1]
        path = a_star_function(X, Y, taskX, taskY)
        robotPaths[robot_name] = path
        robotPaths[robot_name].pop(0)
        lastTask = robotTasks[robot_name].pop(0)
      else:
        ## don't want the controller to go anywhere new
        return
    elif msg1.need_task_update:
      if len(robotPaths[robot_name]) > 0:
        X = msg2.pose.pose.position.x
        Y = msg2.pose.pose.position.y
        X = round(X*4)/4
        Y = round(Y*4)/4
        taskX = lastTask
        taskY = lastTask
        path = a_star_function(X, Y, taskX, taskY)
        robotPaths[robot_name] = path
        robotPaths[robot_name].pop(0)
    if len(robotTasks[robot_name]) > 0 and len(robotPaths[robot_name]) == 0:
      return 


    pub0 = rospy.Publisher('/tf', tf2_msgs.msg.TFMessage, queue_size = 50)

    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = "map_static"
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = robot_name + 'goal'
    t.transform.translation.x = robotPaths[robot_name][0][0]
    t.transform.translation.y = robotPaths[robot_name][0][1]
    t.transform.translation.z = 0.0
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0
    tfm = tf2_msgs.msg.TFMessage([t])

    pub0.publish(tfm)
    r.sleep()

  except Exception as e:
    if (len(robotPaths[robot_name]) == 0 and len(robotTasks[robot_name]) == 4):
      X = msg2.pose.pose.position.x
      Y = msg2.pose.pose.position.y
      X = round(X*4)/4
      Y = round(Y*4)/4
      taskX = taskLocations[robotTasks[robot_name][0]][0]
      taskY = taskLocations[robotTasks[robot_name][0]][1]
      path = a_star_function(X, Y, taskX, taskY)
      robotPaths[robot_name] = path
      robotPaths[robot_name].pop(0)
      lastTask = robotTasks[robot_name].pop(0)
    if len(robotTasks[robot_name]) > 0 and len(robotPaths[robot_name]) == 0:
      X = msg2.pose.pose.position.x
      Y = msg2.pose.pose.position.y
      X = round(X*4)/4
      Y = round(Y*4)/4
      taskX = taskLocations[robotTasks[robot_name][0]][0]
      taskY = taskLocations[robotTasks[robot_name][0]][1]
      path = a_star_function(X, Y, taskX, taskY)
      robotPaths[robot_name] = path
      robotPaths[robot_name].pop(0)
      lastTask = robotTasks[robot_name].pop(0)

    if len(robotTasks[robot_name]) > 0 and len(robotPaths[robot_name]) == 0:
      return 
    pub0 = rospy.Publisher('/tf', tf2_msgs.msg.TFMessage, queue_size = 50)
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = "map_static"
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = robot_name + 'goal'
    t.transform.translation.x = robotPaths[robot_name][0][0]
    t.transform.translation.y = robotPaths[robot_name][0][1]
    t.transform.translation.z = 0.0
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0
    tfm = tf2_msgs.msg.TFMessage([t])
    pub0.publish(tfm)

    r.sleep()
    #robotPaths[robot_name].pop(0)


#Python's syntax for a main() method
if __name__ == '__main__':
    #Run this program as a new node in the ROS computation graph
    #called /listener_<id>, where <id> is a randomly generated numeric
    #string. This randomly generated name means we can start multiple
    #copies of this node without having multiple nodes with the same
    #name, which ROS doesn't allow.
    print("Taskmaster initiated.")
    taskLocations = {"task1": (12, 12), "task2": (8, 5), "task3": (12, 1), "task4": (15,1), "task5": (1,6.5), 
    "task6": (9, 7.5), "task7": (15.5, 5), "task8": (16, 8), "task9": (22, 7), "task10": (18,10)}


    robot0Tasks = ["task1", "task7", "task4", "task3"]
    robot1Tasks = ["task5", "task3", "task2", "task9"]
    robot2Tasks = ["task7", "task5", "task10", "task1"]
    robot3Tasks = ["task7", "task1", "task2", "task4"]
    robot4Tasks = ["task8", "task9", "task4", "task2"]
    robot5Tasks = ["task9", "task8", "task3", "task6"]
    robot6Tasks = ["task10", "task6", "task1", "task8"]
    robot7Tasks = ["task1", "task4", "task5", "task10"]

    initialize = True

    lastTask = None

    robotPaths = {"robot0": [], "robot1": [], "robot2": [], "robot3": [], "robot4": [], 
    "robot5": [], "robot6": [], "robot7": []}

    robotTasks = {"robot0": robot0Tasks, "robot1": robot1Tasks, "robot2": robot2Tasks, 
    "robot3": robot3Tasks, "robot4": robot4Tasks, "robot5": robot5Tasks, "robot6": robot6Tasks, "robot7": robot7Tasks}

    rospy.init_node('taskmaster', anonymous=True)

    r = rospy.Rate(10) # 10hz

    taskmaster()