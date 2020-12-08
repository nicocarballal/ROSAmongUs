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
from imposter_search import find_nearest_robot, kill_nearest_robot
import math
import time
from time import sleep

def imposter_master():
    rospy.sleep(10.)
    robot_name = sys.argv[1]

    while not rospy.is_shutdown():
      imposter_manager(robot_name)


def imposter_manager(robot_name):
    try:
        #find nearest robot
        print('In ImposterMaster')
        timeout = 1
        if not targets[robot_name]:
            target = find_nearest_robot(robot_name, alive_crewmates)
            print(target)
            targets[robot_name] = target

            imposter_msg = rospy.wait_for_message("/" + robot_name + "/odom", Odometry, timeout)
            target_msg = rospy.wait_for_message("/" + target + "/odom", Odometry, timeout)

            i_x = imposter_msg.pose.pose.position.x
            i_y = imposter_msg.pose.pose.position.y
            i_x = round(i_x*4)/4
            i_y = round(i_x*4)/4

            t_x = target_msg.pose.pose.position.x
            t_y = target_msg.pose.pose.position.y
            t_x = round(t_x*4)/4
            t_y = round(t_y*4)/4

            path = a_star_function(i_x, i_y, t_x, t_y)
            imposterPaths[robot_name] = path
            pub0 = rospy.Publisher('/tf', tf2_msgs.msg.TFMessage, queue_size = 50)

            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = "map_static"
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = robot_name + 'goal'
            t.transform.translation.x = imposterPaths[robot_name][1][0]
            t.transform.translation.y = imposterPaths[robot_name][1][1]
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            tfm = tf2_msgs.msg.TFMessage([t])
            pub0.publish(tfm)
       
        #try, except
        msg1 = rospy.wait_for_message("/" + robot_name + "/taskUpdate", RobotTaskUpdate, timeout)

        if msg1.need_path_update:
            target = targets[robot_name]
            print('need path update for target: ' + target)
            imposter_msg = rospy.wait_for_message("/" + robot_name + "/odom", Odometry, timeout)
            print('first chace')
            print(targets[robot_name])
            target_msg = rospy.wait_for_message("/" + targets[robot_name] + "/odom", Odometry, timeout)
            print('second chance')

            #check if imposter is within killing range
            i_x = imposter_msg.pose.pose.position.x
            i_y = imposter_msg.pose.pose.position.y
            print('third chance')
            t_x = target_msg.pose.pose.position.x
            t_y = target_msg.pose.pose.position.y
            print('fourth chance')
            print(i_x, i_y, t_x, t_y)
            print(np.sqrt((i_x - t_x)**2 + (i_y - t_y)**2))
            if np.sqrt((i_x - t_x)**2 + (i_y - t_y)**2) < 1:
                print('kill')
                targets[robot_name] = None
                alive_crewmates.remove(target)
                #kill_nearest_robot()
                #do something that kills robot

                if alive_crewmates:
                    return


                else:
                    return
                    #end the game
                    
            else:
                print('update')
                path = a_star_function(i_x, i_y, t_x, t_y)
                imposterPaths[robot_name] = path
                pub0 = rospy.Publisher('/tf', tf2_msgs.msg.TFMessage, queue_size = 50)

                t = geometry_msgs.msg.TransformStamped()
                t.header.frame_id = "map_static"
                t.header.stamp = rospy.Time.now()
                t.child_frame_id = robot_name + 'goal'
                t.transform.translation.x = imposterPaths[robot_name][1][0]
                t.transform.translation.y = imposterPaths[robot_name][1][1]
                t.transform.translation.z = 0.0
                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = 0.0
                t.transform.rotation.w = 1.0
                tfm = tf2_msgs.msg.TFMessage([t])
                pub0.publish(tfm)

                pub_update = rospy.Publisher(robot_name + '/taskUpdate', RobotTaskUpdate, queue_size=10)
                updateMsg = RobotTaskUpdate()
                updateMsg.robot_name = robot_name
                updateMsg.need_task_update = False
                updateMsg.need_path_update = False
                pub_update.publish(updateMsg)
                r.sleep()
            return

        else:
            return


    except Exception as e:
        print(e)
        if not targets[robot_name]:
            target = find_nearest_robot(robot_name, alive_crewmates)
            targets[robot_name] = target
        else:
            target = targets[robot_name]

        # imposter_msg = rospy.wait_for_message("/" + robot_name + "/odom", Odometry, timeout)
        # target_msg = rospy.wait_for_message("/" + target + "/odom", Odometry, timeout)

        # i_x = imposter_msg.pose.pose.position.x
        # i_y = imposter_msg.pose.pose.position.y

        # t_x = target_msg.pose.pose.position.x
        # t_y = target_msg.pose.pose.position.y
        # print(i_x, i_y, t_x, t_y)
        # path = a_star_function(i_x, i_y, t_x, t_y)
        # imposterPaths[robot_name] = path
        # if not path:
        #     targets.pop(robot_name)
        #     alive_crewmates.remove(target)
        #     return
        pub0 = rospy.Publisher('/tf', tf2_msgs.msg.TFMessage, queue_size = 50)

        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = "map_static"
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = robot_name + 'goal'
        t.transform.translation.x = imposterPaths[robot_name][1][0]
        t.transform.translation.y = imposterPaths[robot_name][1][1]
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        tfm = tf2_msgs.msg.TFMessage([t])
        pub0.publish(tfm)
        return

if __name__ == '__main__':
    #Run this program as a new node in the ROS computation graph
    #called /listener_<id>, where <id> is a randomly generated numeric
    #string. This randomly generated name means we can start multiple
    #copies of this node without having multiple nodes with the same
    #name, which ROS doesn't allow.
    print("ImposterMaster initiated.")


    alive_crewmates = ["robot0", "robot1", "robot2", "robot3", "robot4", "robot5"]

    targets = {"robot6": None, "robot7": None}

    initialize = True

    imposterPaths = {}

    rospy.init_node('impostermaster', anonymous=True)

    r = rospy.Rate(10) # 10hz

    imposter_master()