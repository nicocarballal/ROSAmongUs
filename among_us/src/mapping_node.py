#!/usr/bin/python
################################################################################
#
# Node to wrap the OccupancyGrid2d class.
#
################################################################################

from occupancy_grid_2d import OccupancyGrid2d

import rospy
import sys

if __name__ == "__main__":
	print('Occupancy Grid initiated')
	fixed_frames = ['~frames/fixed0', '~frames/fixed1', '~frames/fixed2', '~frames/fixed3','~frames/fixed4','~frames/fixed5','~frames/fixed6','~frames/fixed7']
	sensor_frames = ['~frames/sensor0', '~frames/sensor1', '~frames/sensor2', '~frames/sensor3','~frames/sensor4','~frames/sensor5','~frames/sensor6','~frames/sensor7']
	vis_topics = ['~topics/vis0', '~topics/vis1', '~topics/vis2', '~topics/vis3','~topics/vis4','~topics/vis5','~topics/vis6','~topics/vis7']
	sensor_topics = ['~topics/sensor0', '~topics/sensor1', '~topics/sensor2', '~topics/sensor3','~topics/sensor4','~topics/sensor5','~topics/sensor6','~topics/sensor7']

	for i in range(8):

	    rospy.init_node("mapping_node")
	    og = OccupancyGrid2d(fixed_frames[i], sensor_frames[i], sensor_topics[i], vis_topics[i])
	    if not og.Initialize():
	        rospy.logerr("Failed to initialize the mapping node.")
	        sys.exit(1)

	rospy.spin()
