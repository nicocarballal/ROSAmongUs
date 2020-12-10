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

        try:
            robotPath = rospy.get_param(robot_name + '/path')

            robotPath = literal_eval(robotPath)

         
            if len(robotPath) == 0:
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

            '''
            path.color.a = colorLibrary[robot_name][0]# Don't forget to set the alpha!
            path.color.r = colorLibrary[robot_name][1]
            path.color.g = colorLibrary[robot_name][2]
            path.color.b = colorLibrary[robot_name][3]
            '''
            path.header = newPathHeader
            pub.publish(path)
            sleep(5)

        except Exception as e:
            print(e)

        
if __name__ == '__main__':
    print("initiated path publisher for: " + sys.argv[1])

    colorLibrary = {'robot0': [1, 0, 0, .7], 'robot1': [1, .1, 1, .1],'robot2': [1, 0, 0, 0],'robot3':[1, .2, 1, 1],'robot4': [1,0.9,0,0],
    'robot5': [1,.1,0.5,0.1],'robot6': [1,0.8,0.3,0.8], 'robot7':  [1,0.4,0.2,0]}
    robot_name = sys.argv[1]
    rospy.init_node('path_publisher_' + robot_name)

    path_publisher(robot_name)