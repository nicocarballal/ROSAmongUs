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

def find_nearest_robot(imposter):
    crewmates = rospy.get_param('alive_crewmates')
    crewmates = crewmates.split()
    timeout = 1
    time1 = time.time()
    ix = rospy.get_param(imposter + "/positionX")
    iy = rospy.get_param(imposter + "/positionY")

    smallest_distance = float('inf')
    closest_robot = 'None'
    for c in crewmates:
        cx = rospy.get_param(c + "/positionX")
        cy  = rospy.get_param(c + "/positionY")
        dist = np.sqrt((ix - cx)**2 + (iy - cy)**2)

        if dist < smallest_distance:
            if imposter == 'robot6':
                otherImpostorTarget = rospy.get_param('imposter2/target')
            else:
                otherImpostorTarget = rospy.get_param('imposter1/target')
            if c != otherImpostorTarget:
                smallest_distance = dist
                closest_robot = c
    time2 = time.time()

    return closest_robot


def kill_nearest_robot():
    return