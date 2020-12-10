
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
import numpy as np
from utils import listToString
        
def initialize():
    print('Give the crewmates a 30 second head start')
    sleep(30)
    while not rospy.is_shutdown():
        kill_checker()

def kill_checker():
    alive_crewmates = rospy.get_param('alive_crewmates')

    alive_crewmates = alive_crewmates.split()

    imposter6X = rospy.get_param('robot6/positionX')
    imposter6Y = rospy.get_param('robot6/positionY') 
    imposter7X = rospy.get_param('robot7/positionX')
    imposter7Y = rospy.get_param('robot7/positionY')
    for crewmate in alive_crewmates:
        X = rospy.get_param(crewmate + '/positionX')
        Y = rospy.get_param(crewmate + '/positionY')
        dist6 = np.sqrt((X - imposter6X)**2 + (Y - imposter6Y)**2)
        dist7 = np.sqrt((X - imposter7X)**2 + (Y - imposter7Y)**2)
        if dist6 < 1 or dist7 < 1:
            print("KILLLLLLLLLLLLL")
            alive_crewmates.remove(crewmate)
            rospy.set_param('alive_crewmates', listToString(alive_crewmates))
    sleep(3)








if __name__ == '__main__':
    print('Initialized kill_checker.')
    rospy.init_node('kill_checker', anonymous=True)
    initialize()