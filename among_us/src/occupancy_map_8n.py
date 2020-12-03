from gridmap import OccupancyGridMap
import matplotlib.pyplot as plt
from a_star import a_star
from utils import plot_path



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


if __name__ == '__main__':
    # load the map
    gmap = OccupancyGridMap.from_png('among-us-edges-fixed-ai1.png', .05)


    # set a start and an end node (in meters)
    start_node = (15, 12) ##AMONG US: Where we are
    goal_node = (8, 5) ##AMONG US: Where the task is :)

    # run A*
    path, path_px = a_star(start_node, goal_node, gmap, movement='4N')

    path = path_cleaner(path)

    print(path)