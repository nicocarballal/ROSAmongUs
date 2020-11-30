#!/usr/bin/env python

import rospy 

from geometry_msgs.srv import Twist


class RobotMonitor(object):
	def __init__(self):
		pass

	def get_closest(self, req):
		rospy.loginfo('getClosest called')
		response = getClosestResponse()
		return response

	def get_distance(self, req):
		rospy.loginfo('GetDistance called with {}'.format(req.name))
		response = getDistanceResponse()
		return response


def main():
	rospy.init_noe('robot_server')
	monitor = LandmarkMonitor()
	get_closest = rospy.Service('get_closest', getClosest, )
	get_distance = rospy.Service('get_distance', getDistance, )
	rospy.spin()

if __name__ == '__main__':
	main()