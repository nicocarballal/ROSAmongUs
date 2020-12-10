#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import sys
import numpy as np

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
from sensor_msgs.msg import LaserScan
from utils import listToString

def taskmaster():
    robot_name = sys.argv[1]
    if robot_name == 'robot6' or robot_name == 'robot7':
      sleep(30)
    while not rospy.is_shutdown():
      if robot_name == 'robot6' or robot_name == 'robot7':
        task_manager_imposter(robot_name)
      else:
        task_manager(robot_name)
      #task_manager('robot2')


def task_manager_imposter(robot_name):
      try:
        #find nearest robot
        timeout = 1
        # wait for message from imposter and target
        imposterX = rospy.get_param(robot_name + "/positionX")
        imposterY = rospy.get_param(robot_name + "/positionY")
        taskUpdate_msg = rospy.wait_for_message("/" + robot_name + "/taskUpdate", RobotTaskUpdate, timeout)
        ## if robot doesn't have a target

        # THIS LOOP ONLY KILLS WHEN AT A WAYPOINT
        
        if taskUpdate_msg.need_path_update: 
          if len(imposterPaths[robot_name]) > 0:
            imposterPaths[robot_name].pop(0)
            pub_update = rospy.Publisher(robot_name + '/taskUpdate', RobotTaskUpdate, queue_size=10)
            updateMsg = RobotTaskUpdate()
            updateMsg.robot_name = robot_name
            updateMsg.need_task_update = False
            updateMsg.need_path_update = False
            pub_update.publish(updateMsg)
          elif len(alive_crewmates) > 0:
            target = find_nearest_robot(robot_name)
            if robot_name == 'robot6':
              rospy.set_param('imposter1/target', target)
            else:
              rospy.set_param('imposter2/target', target)
            targetX = rospy.get_param(target + "/positionX")
            targetY = rospy.get_param(target + "/positionY")
            X = imposterX
            Y = imposterY
            X = round(X*4)/4
            Y = round(Y*4)/4
            targetX = round(targetX*4)/4
            targetY = round(targetY*4)/4
            path = a_star_function(X, Y, targetX, targetY, robot_name)
            imposterPaths[robot_name] = path
            imposterPaths[robot_name].pop(0)
            targets[robot_name] = target
          else: 
            return 

        if len(alive_crewmates) > 0 and len(imposterPaths[robot_name]) == 0:
        
          return 
        pub0 = rospy.Publisher('/tf', tf2_msgs.msg.TFMessage, queue_size = 50)
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = "map_static"
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = robot_name + 'goal'
        t.transform.translation.x = imposterPaths[robot_name][0][1]
        t.transform.translation.y = imposterPaths[robot_name][0][1]
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        tfm = tf2_msgs.msg.TFMessage([t])


        pub0.publish(tfm)
        r.sleep()

      except Exception as e:
        ### Check that there is an alive crewmate if the imposter has nowhere to go
          if len(alive_crewmates) > 0 and len(imposterPaths[robot_name]) == 0:
            X = imposterX
            Y = imposterY
            X = round(X*4)/4
            Y = round(Y*4)/4
            target = find_nearest_robot(robot_name)
            if robot_name == 'robot6':
              rospy.set_param('imposter1/target', target)
            else:
              rospy.set_param('imposter2/target', target)
            targetX = rospy.get_param(target + "/positionX")
            targetY = rospy.get_param(target + "/positionY")
            targetX = round(targetX*4)/4
            targetY = round(targetY*4)/4
            path = a_star_function(X, Y, targetX, targetY, robot_name)
            imposterPaths[robot_name] = path
            imposterPaths[robot_name].pop(0)
            targets[robot_name] = target
          if len(alive_crewmates) == 0 and len(imposterPaths[robot_name]) == 0:
            return 
          pub0 = rospy.Publisher('/tf', tf2_msgs.msg.TFMessage, queue_size = 50)
          t = geometry_msgs.msg.TransformStamped()
          t.header.frame_id = "map_static"
          t.header.stamp = rospy.Time.now()
          t.child_frame_id = robot_name + 'goal'
          t.transform.translation.x = imposterPaths[robot_name][0][0]
          t.transform.translation.y = imposterPaths[robot_name][0][1]
          t.transform.translation.z = 0.0
          t.transform.rotation.x = 0.0
          t.transform.rotation.y = 0.0
          t.transform.rotation.z = 0.0
          t.transform.rotation.w = 1.0
          tfm = tf2_msgs.msg.TFMessage([t])

          pub0.publish(tfm)

          r.sleep()


def task_manager(robot_name):
  try: 
    timeout = 1
    crewmateX = rospy.get_param(robot_name + "/positionX")
    crewmateY = rospy.get_param(robot_name + "/positionY")
    msg1 = rospy.wait_for_message("/" + robot_name + "/taskUpdate", RobotTaskUpdate, timeout)
    
    
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
        print("hello")
        sleep(1)
        lasermsg = rospy.wait_for_message("/" + robot_name + "/laser_0", LaserScan, 10)
        print("recieved laser message")
        ranges = lasermsg.ranges
        mindist = min([i for i in lasermsg.ranges if i > 0.15])
        minindex = ranges.index(mindist)
        if minindex < 100:
            ranges = ranges[minindex + 567:] + ranges
        elif minindex > 567:
            ranges = ranges + ranges[:100]
        ranges = [i for i in ranges if i < mindist + .2]
        if len(ranges)%2 == 0:
            ranges = ranges[:len(ranges)-1]
        print("DEBUG RANGES", len(ranges))
        def dummy_array(min_pt, size):
            dummy = []
            theta = (np.pi * 2) / 667
            for i in range(size//2+1):
                distance = min_pt / np.cos(i*theta)
                dummy.append(distance)
            left = dummy[:]
            left.reverse()
            left.pop()
            dummy_list = left + dummy
            return dummy_list
        flat = dummy_array(mindist, len(ranges))
        print("DEBUG:FLAT", len(flat))
        difference = np.asarray(flat) - np.asarray(ranges)
        if difference[0] and difference[len(difference)-1] < 0:
            print(robot_name, "This task is convex!")
        else:
            print(robot_name, "this task is concave!")
        X = crewmateX
        Y = crewmateY
        X = round(X*4)/4
        Y = round(Y*4)/4
        print(robotTasks[robot_name])
        taskX = taskLocations[robotTasks[robot_name][0]][0]
        taskY = taskLocations[robotTasks[robot_name][0]][1]
        path = a_star_function(X, Y, taskX, taskY, robot_name)
        robotPaths[robot_name] = path
        robotPaths[robot_name].pop(0)
        robotTasks[robot_name].pop(0)
      else:
        ## don't want the controller to go anywhere new
        return
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
      X = crewmateX
      Y = crewmateY
      X = round(X*4)/4
      Y = round(Y*4)/4
      taskX = taskLocations[robotTasks[robot_name][0]][0]
      taskY = taskLocations[robotTasks[robot_name][0]][1]
      path = a_star_function(X, Y, taskX, taskY, robot_name)
      robotPaths[robot_name] = path
      robotPaths[robot_name].pop(0)
      robotTasks[robot_name].pop(0)
    if len(robotTasks[robot_name]) > 0 and len(robotPaths[robot_name]) == 0:
        X = crewmateX
        Y = crewmateY
        taskX = taskLocations[robotTasks[robot_name][0]][0]
        taskY = taskLocations[robotTasks[robot_name][0]][1]
        path = a_star_function(X, Y, taskX, taskY, robot_name)
        robotPaths[robot_name] = path
        robotPaths[robot_name].pop(0)
        robotTasks[robot_name].pop(0)

    if len(robotTasks[robot_name]) > 0 and len(robotPaths[robot_name]) == 0:
      return 

    if not finishedTasks[robot_name]:
      if len(robotTasks[robot_name]) == 0 and len(robotPaths[robot_name]) == 0:
        robots_with_tasks = rospy.get_param('robots_with_tasks')
        robots_with_tasks = robots_with_tasks.split()  
        robots_with_tasks.remove(robot_name)
        finishedTasks[robot_name] = True
        rospy.set_param('robots_with_tasks', listToString(robots_with_tasks))
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
    taskLocations = {"task1": (12, 12), "task2": (8.25, 5), "task3": (12, 1), "task4": (15,1), "task5": (1.25,6.5), 
    "task6": (9, 7.5), "task7": (15.5, 5.5), "task8": (16.25, 7.75), "task9": (22.25, 7.25), "task10": (18,10)}


    robot0Tasks = ["task1", "task6", "task4", "task3"]
    robot1Tasks = ["task5", "task3", "task2", "task9"]
    robot2Tasks = ["task7", "task5", "task10", "task1"]
    robot3Tasks = ["task2", "task1", "task8", "task4"]
    robot4Tasks = ["task8", "task9", "task4", "task2"]
    robot5Tasks = ["task4", "task8", "task3", "task6"]
 
    robot6Tasks = ["task10", "task6", "task1", "task8"]
    robot7Tasks = ["task9", "task2", "task5", "task10"]

    finishedTasks = {"robot0": False, "robot1": False, "robot2": False, "robot3": False, "robot4": False, 
    "robot5": False}

    initialize = True

    alive_crewmates = ["robot0", "robot1", "robot2", "robot3", "robot4", "robot5"]

    targets = {"robot6": None, "robot7": None}

    robotPaths = {"robot0": [], "robot1": [], "robot2": [], "robot3": [], "robot4": [], 
    "robot5": [], "robot6": [], "robot7": []}

    imposterPaths = {"robot6": [], "robot7": []}

    robotTasks = {"robot0": robot0Tasks, "robot1": robot1Tasks, "robot2": robot2Tasks, 
    "robot3": robot3Tasks, "robot4": robot4Tasks, "robot5": robot5Tasks, "robot6": robot6Tasks, "robot7": robot7Tasks}

    rospy.init_node('taskmaster', anonymous=True)

    r = rospy.Rate(10) # 10hz

    taskmaster()