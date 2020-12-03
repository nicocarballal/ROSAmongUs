
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
    c = True 
    while not rospy.is_shutdown():
      create_tasks()
      tf_frames()
      task_manager()
      
      
        
        #task_manager()



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

def task_manager():
  robot_name = 'robot0'
  try: 
    timeout = 0.3
    msg2 = rospy.wait_for_message("/robot0/odom", Odometry, timeout)
    msg1 = rospy.wait_for_message("/robot0/taskUpdate", RobotTaskUpdate, timeout)
    
    
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
    print(msg1)
    if msg1.need_path_update:
      if len(robotPaths[robot_name]) > 0:
        robotPaths[robot_name].pop(0)
        pub_update = rospy.Publisher(robot_name + '/taskUpdate', RobotTaskUpdate, queue_size=10)
        updateMsg = RobotTaskUpdate()
        updateMsg.robot_name = robot_name
        updateMsg.need_task_update = False
        updateMsg.need_path_update = False
        pub_update.publish(updateMsg)
        print('New Path:')
        print('--------')
        print(robotPaths[robot_name][0])
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
        robotTasks[robot_name].pop(0)
      else:
        ## don't want the controller to go anywhere new
        return
    if len(robotTasks[robot_name]) > 0 and len(robotPaths[robot_name]) == 0:
      return 


    X = msg2.pose.pose.position.x
    Y = msg2.pose.pose.position.y
    taskX = taskLocations[robotTasks[robot_name][0]][0]
    taskY = taskLocations[robotTasks[robot_name][0]][1]

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
    print(e)
    print('publishing initial goal')
    if (len(robotPaths[robot_name]) == 0 and len(robotTasks[robot_name]) == 4):
      X = 12
      Y = 10
      taskX = taskLocations[robotTasks[robot_name][0]][0]
      taskY = taskLocations[robotTasks[robot_name][0]][1]
      path = a_star_function(X, Y, taskX, taskY)
      robotPaths[robot_name] = path
      robotPaths[robot_name].pop(0)
      robotTasks[robot_name].pop(0)
    if len(robotTasks[robot_name]) > 0 and len(robotPaths[robot_name]) == 0:
        X = msg2.pose.pose.position.x
        Y = msg2.pose.pose.position.y
        X = round(X*4)/4
        Y = round(Y*4)/4
        print(X)
        print(Y)
        taskX = taskLocations[robotTasks[robot_name][0]][0]
        taskY = taskLocations[robotTasks[robot_name][0]][1]
        path = a_star_function(X, Y, taskX, taskY)
        robotPaths[robot_name] = path
        robotPaths[robot_name].pop(0)
        robotTasks[robot_name].pop(0)

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
    "task6": (9, 6.5), "task7": (15.5, 5), "task8": (15.5, 8.5), "task9": (22, 7), "task10": (18,10)}

    robot0Tasks = ["task1", "task7", "task4", "task3"]
    robot1Tasks = ["task5", "task3", "task2", "task9"]
    robot2Tasks = ["task6", "task5", "task10", "task1"]
    robot3Tasks = ["task7", "task1", "task2", "task4"]
    robot4Tasks = ["task8", "task9", "task4", "task2"]
    robot5Tasks = ["task9", "task8", "task3", "task6"]
    robot6Tasks = ["task10", "task6", "task1", "task8"]
    robot7Tasks = ["task1", "task4", "task5", "task10"]

    

    initialize = True

    robotPaths = {"robot0": [], "robot1": [], "robot2": [], "robot3": [], "robot4": [], 
    "robot5": [], "robot6": [], "robot7": []}

    robotTasks = {"robot0": robot0Tasks, "robot1": robot1Tasks, "robot2": robot2Tasks, 
    "robot3": robot3Tasks, "robot4": robot4Tasks, "robot5": robot5Tasks, "robot6": robot6Tasks, "robot7": robot7Tasks}

    rospy.init_node('taskmaster', anonymous=True)

    r = rospy.Rate(10) # 10hz

    taskmaster()
