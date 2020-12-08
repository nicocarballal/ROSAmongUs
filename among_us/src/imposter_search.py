import rospy
import sys
import tf2_ros
import tf2_msgs.msg
import numpy as np

import geometry_msgs
from geometry_msgs.msg import Twist
from among_us.msg import RobotTaskUpdate
from visualization_msgs.msg import Marker, MarkerArray
import time
from time import sleep
from a_star_function import a_star_function
from nav_msgs.msg import Odometry

def find_nearest_robot(imposter, crewmates):
    print(crewmates)
    timeout = 5
    imposter_msg = rospy.wait_for_message("/" + imposter + "/odom", Odometry, timeout)
    ix = imposter_msg.pose.pose.position.x
    iy = imposter_msg.pose.pose.position.y

    smallest_distance = float('inf')
    closest_robot = None
    for c in crewmates:

        crewmate_msg = rospy.wait_for_message("/" + c + "/odom", Odometry, timeout)
        cx = crewmate_msg.pose.pose.position.x
        cy = crewmate_msg.pose.pose.position.y
        dist = np.sqrt((ix - cx)**2 + (iy - cy)**2)

        if dist < smallest_distance:
            smallest_distance = dist
            closest_robot = c
    print(closest_robot)
    return closest_robot


def kill_nearest_robot():
    return