import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Path
import time
from time import sleep
import sys 
import re
from ast import literal_eval

def path_publisher(robot_name):
    

    while not rospy.is_shutdown():
   
        robotPath = rospy.get_param(robot_name + '/path')

        robotPath = literal_eval(robotPath)


        if len(robotPath) == 0:
            sleep(1)
            continue
		
	
        pub = rospy.Publisher(robot_name + '/path', Path, queue_size=10)

        i = 1

        path = Path()
        for tup in robotPath:
            newPoint = Point()
            newPoint.x = tup[0]
            newPoint.y = tup[1]
            newPoint.z = 0
            newRot = Quaternion()
            newRot.x = 0
            newRot.y = 0
            newRot.z = 0
            newRot.w = 1
            newPose = Pose()
            newPose.position = newPoint
            newPose.orientation = newRot
            newPoseStamped = PoseStamped()
            newHeader = Header()
            newHeader.seq = i
            Header()
            i+=1
            newHeader.stamp = rospy.Time()
            newHeader.frame_id = "world"
            newPoseStamped.pose = newPose
            newPoseStamped.header = newHeader
            path.poses.append(newPoseStamped)

        newPathHeader = Header()
        newPathHeader.seq = i
        newPathHeader.stamp = rospy.Time()
        newPathHeader.frame_id = "world"
        path.header = newPathHeader

        pub.publish(path)

        sleep(1)

if __name__ == '__main__':
    rospy.init_node('path_publisher')
    robot_name = sys.argv[1]
    path_publisher(robot_name)