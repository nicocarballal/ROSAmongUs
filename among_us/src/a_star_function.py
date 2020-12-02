from gridmap import OccupancyGridMap
import matplotlib.pyplot as plt
from a_star import a_star
from utils import plot_path


def a_star_function(X, Y, taskX, taskY):
    # load the map
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

        plt.plot(start_node_px[0], start_node_px[1], 'ro')
        plt.plot(goal_node_px[0], goal_node_px[1], 'go')

    plt.show()

    return path
