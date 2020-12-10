from gridmap import OccupancyGridMap
import matplotlib.pyplot as plt
from a_star import a_star
from utils import plot_path
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from among_us.msg import OccupancyGridUpdate
import numpy as np
import time 
def path_cleaner(array):
    path = []
    last_waypoint = array[0]
    path. append(last_waypoint)
    for i in range(1, len(array) - 1):
        waypoint = array[i]
        if (waypoint[0] == array[i-1][0]):
            if (waypoint[0] != array[i+1][0]):
                path.append(waypoint)
        if (waypoint[1] == array[i-1][1]):
            if (waypoint[1] != array[i+1][1]):
                path.append(waypoint)

    path.append(array[len(array)- 1])
    return path




def a_star_function(X, Y, taskX, taskY, robotName):
    # load the map\

    global path 
    global robot_name 

    try:
        print('waiting for VIS/MAP')
        timeout = 100
        time1 = time.time()
        gmap = rospy.wait_for_message("map/occupancy_grid", OccupancyGridUpdate, timeout)


        #cell_size = gmap.resolution
        cell_size = .25 ## (7, 12.5)

        ### Need to fix how it's being reshaped

        data_array = np.reshape(gmap.occupancy_grid, (-1, gmap.height))

        data_array = np.fliplr(np.rot90(data_array, -1))

        gmap = OccupancyGridMap(data_array, cell_size)

        #OccupancyGridMap.from_png('among-us-edges-fixed-ai1.png', .05)
        # set a start and an end node (in meters)
        start_node = (X, Y) ##AMONG US: Where we are
        goal_node = (taskX, taskY) ##AMONG US: Where the task is :)
 
        # run A*


        path, path_px = a_star(start_node, goal_node, gmap, movement='4N')
        
        
        
 
        gmap.plot()
        if path:
            # plot resulting path in pixels over the map
            plot_path(path_px)
        else:
            print('Goal is not reachable')

            # plot start and goal points over the map (in pixels)
            start_node_px = gmap.get_index_from_coordinates(start_node[0], start_node[1])
            goal_node_px = gmap.get_index_from_coordinates(goal_node[0], goal_node[1])

        #     plt.plot(start_node_px[0], start_node_px[1], 'ro')
        #     plt.plot(goal_node_px[0], goal_node_px[1], 'go')

        # plt.show()
        

        
        path = path_cleaner(path)

        time2 = time.time()

        print('A Star Time:' + str(time2 - time1))
        robot_name = robotName





        return path
    except Exception as e:
        print(e)

        gmap = OccupancyGridMap.from_png('among-us-edges-fixed-ai1.png', .05)

        # set a start and an end node (in meters)
        start_node = (X, Y) ##AMONG US: Where we are
        goal_node = (taskX, taskY) ##AMONG US: Where the task is :)

        # run A*


        path, path_px = a_star(start_node, goal_node, gmap, movement='4N')

        
        
        gmap.plot()
        

        if path:
            # plot resulting path in pixels over the map
            plot_path(path_px)
        else:
            print('Goal is not reachable')

            # plot start and goal points over the map (in pixels)
            start_node_px = gmap.get_index_from_coordinates(start_node[0], start_node[1])
            goal_node_px = gmap.get_index_from_coordinates(goal_node[0], goal_node[1])

        #     plt.plot(start_node_px[0], start_node_px[1], 'ro')
        #     plt.plot(goal_node_px[0], goal_node_px[1], 'go')

        # plt.show()
        
        
        
        path = path_cleaner(path)

        return path



def publish_path():
    global robot_name
    global path 
    while not rospy.is_shutdown():
        try: 
            pub = rospy.Publisher(robot_name + 'Path', nav_msgs.msg.Path, queue_size=10)

            i = 1
            newPath = Path()
            for tup in path:
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
                i+=1
                newHeader.stamp = rospy.Time()
                newHeader.frame_id = ""
                newPoseStamped.pose = newPose
                newPoseStamped.header = newHeader
                newPath.poses.append(newPoseStamped)
            
            newPathHeader = Header()
            newPathHeader.seq = i
            newPathHeader.stamp = rospy.Time()
            newPathHeader.frame_id = ""
            newPath.header = newPathHeader

            pub.publish(newPath)
        except Exception as e:
            print(robot_name + ': path not published yet')


#Python's syntax for a main() method
if __name__ == '__main__':
    #Run this program as a new node in the ROS computation graph
    #called /listener_<id>, where <id> is a randomly generated numeric
    #string. This randomly generated name means we can start multiple
    #copies of this node without having multiple nodes with the same
    #name, which ROS doesn't allow.
    print("Path Publisher initiated.")
    
    global robot_name
    robot_name = 'robot0'
    global path
    path = []

    rospy.init_node('path_publisher', anonymous=True)

    r = rospy.Rate(10) # 10hz

    publish_path()