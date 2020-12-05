################################################################################
#
# OccupancyGrid2d class listens for LaserScans and builds an occupancy grid.
#
################################################################################

import rospy
import tf2_ros
import tf

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

import numpy as np
import math

class OccupancyGrid2d(object):
    def __init__(self):
        self._intialized = False

        # Set up tf buffer and listener.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

    # Initialization and loading parameters.
    def Initialize(self):
        self._name = rospy.get_name() + "/grid_map_2d"

        # Load parameters.
        if not self.LoadParameters():
            rospy.logerr("%s: Error loading parameters.", self._name)
            return False

        # Register callbacks.
        if not self.RegisterCallbacks():
            rospy.logerr("%s: Error registering callbacks.", self._name)
            return False

        # Set up the map.
        self._map = np.zeros((self._x_num, self._y_num))

        self._initialized = True
        return True

    def LoadParameters(self):
        # Random downsampling fraction, i.e. only keep this fraction of rays.
        if not rospy.has_param("~random_downsample"):
            return False
        self._random_downsample = rospy.get_param("~random_downsample")

        # Dimensions and bounds.
        # TODO! You'll need to set values for class variables called:
        if not rospy.has_param("~x/num"):
            return False
        self._x_num = rospy.get_param("~x/num")
        if not rospy.has_param("~x/min"):
            return False
        self._x_min = rospy.get_param("~x/min")
        if not rospy.has_param("~x/max"):
            return False
        self._x_max = rospy.get_param("~x/max")
        if not rospy.has_param("~y/num"):
            return False
        self._y_num = rospy.get_param("~y/num")
        if not rospy.has_param("~y/min"):
            return False
        self._y_min = rospy.get_param("~y/min")
        if not rospy.has_param("~y/max"):
            return False
        self._y_max = rospy.get_param("~y/max")
        self._x_res = float((self._x_max-self._x_min)/float(self._x_num))
        self._y_res = float((self._y_max-self._y_min)/float(self._y_num))
        
        # Update parameters.
        if not rospy.has_param("~update/occupied"):
            return False
        self._occupied_update = self.ProbabilityToLogOdds(
            rospy.get_param("~update/occupied"))

        if not rospy.has_param("~update/occupied_threshold"):
            return False
        self._occupied_threshold = self.ProbabilityToLogOdds(
            rospy.get_param("~update/occupied_threshold"))

        if not rospy.has_param("~update/free"):
            return False
        self._free_update = self.ProbabilityToLogOdds(
            rospy.get_param("~update/free"))

        if not rospy.has_param("~update/free_threshold"):
            return False
        self._free_threshold = self.ProbabilityToLogOdds(
            rospy.get_param("~update/free_threshold"))

        # Topics.
        # TODO! You'll need to set values for class variables called:
        if not rospy.has_param("~topics/sensor0"):
            return False
        self._sensor0_topic = rospy.get_param("~topics/sensor0")

        if not rospy.has_param("~topics/sensor1"):
            return False
        self._sensor1_topic = rospy.get_param("~topics/sensor1")

        if not rospy.has_param("~topics/sensor2"):
            return False
        self._sensor2_topic = rospy.get_param("~topics/sensor2")

        if not rospy.has_param("~topics/sensor3"):
            return False
        self._sensor3_topic = rospy.get_param("~topics/sensor3")

        if not rospy.has_param("~topics/sensor4"):
            return False
        self._sensor4_topic = rospy.get_param("~topics/sensor4")

        if not rospy.has_param("~topics/sensor5"):
            return False
        self._sensor5_topic = rospy.get_param("~topics/sensor5")

        if not rospy.has_param("~topics/sensor6"):
            return False
        self._sensor6_topic = rospy.get_param("~topics/sensor6")

        if not rospy.has_param("~topics/sensor7"):
            return False
        self._sensor7_topic = rospy.get_param("~topics/sensor7")

        if not rospy.has_param("~topics/vis"):
            return False
        self._vis_topic = rospy.get_param("~topics/vis")

        # Frames.
        # TODO! You'll need to set values for class variables called:
        if not rospy.has_param("~frames/sensor0"):
            return False
        self._sensor0_frame = rospy.get_param("~frames/sensor0")

        if not rospy.has_param("~frames/sensor1"):
            return False
        self._sensor1_frame = rospy.get_param("~frames/sensor1")

        if not rospy.has_param("~frames/sensor2"):
            return False
        self._sensor2_frame = rospy.get_param("~frames/sensor2")

        if not rospy.has_param("~frames/sensor3"):
            return False
        self._sensor3_frame = rospy.get_param("~frames/sensor3")

        if not rospy.has_param("~frames/sensor4"):
            return False
        self._sensor4_frame = rospy.get_param("~frames/sensor4")

        if not rospy.has_param("~frames/sensor5"):
            return False
        self._sensor5_frame = rospy.get_param("~frames/sensor5")

        if not rospy.has_param("~frames/sensor6"):
            return False
        self._sensor6_frame = rospy.get_param("~frames/sensor6")

        if not rospy.has_param("~frames/sensor7"):
            return False
        self._sensor7_frame = rospy.get_param("~frames/sensor7")

        if not rospy.has_param("~frames/fixed"):
            return False
        self._fixed_frame = rospy.get_param("~frames/fixed")

        return True

    def RegisterCallbacks(self):
        # Subscriber.
        self._sensor0_sub = rospy.Subscriber(self._sensor0_topic,
                                            LaserScan,
                                            self.SensorCallback, (self._sensor0_frame),
                                            queue_size=1)
        self._sensor1_sub = rospy.Subscriber(self._sensor1_topic,
                                            LaserScan,
                                            self.SensorCallback, (self._sensor1_frame),
                                            queue_size=1)
        self._sensor2_sub = rospy.Subscriber(self._sensor2_topic,
                                            LaserScan,
                                            self.SensorCallback, (self._sensor2_frame),
                                            queue_size=1)
        self._sensor3_sub = rospy.Subscriber(self._sensor3_topic,
                                            LaserScan,
                                            self.SensorCallback, (self._sensor3_frame),
                                            queue_size=1)
        self._sensor4_sub = rospy.Subscriber(self._sensor4_topic,
                                            LaserScan,
                                            self.SensorCallback, (self._sensor4_frame),
                                            queue_size=1)
        self._sensor5_sub = rospy.Subscriber(self._sensor5_topic,
                                            LaserScan,
                                            self.SensorCallback, (self._sensor5_frame),
                                            queue_size=1)
        self._sensor6_sub = rospy.Subscriber(self._sensor6_topic,
                                            LaserScan,
                                            self.SensorCallback, (self._sensor6_frame),
                                            queue_size=1)
        self._sensor7_sub = rospy.Subscriber(self._sensor7_topic,
                                            LaserScan,
                                            self.SensorCallback, (self._sensor7_frame),
                                            queue_size=1)


        # Publisher.
        self._vis_pub = rospy.Publisher(self._vis_topic,
                                        Marker,
                                        queue_size=10)

        return True

    # Callback to process sensor measurements.
    def SensorCallback(self, msg, args):
        sensor_frame = args
        print(sensor_frame)
        if not self._initialized:
            rospy.logerr("%s: Was not initialized.", self._name)
            return

        # Get our current pose from TF.
        try:
            pose = self._tf_buffer.lookup_transform(
                self._fixed_frame, sensor_frame, rospy.Time())
            print(pose)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            # Writes an error message to the ROS log but does not raise an exception
            rospy.logerr("%s: Could not extract pose from TF.", self._name)
            return

        # Extract x, y coordinates and heading (yaw) angle of the turtlebot, 
        # assuming that the turtlebot is on the ground plane.
        sensor_x = pose.transform.translation.x
        sensor_y = pose.transform.translation.y
        if abs(pose.transform.translation.z) > 0.05:
            rospy.logwarn("%s: Turtlebot is not on ground plane.", self._name)

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [pose.transform.rotation.x, pose.transform.rotation.y,
             pose.transform.rotation.z, pose.transform.rotation.w])
        if abs(roll) > 0.1 or abs(pitch) > 0.1:
            rospy.logwarn("%s: Turtlebot roll/pitch is too large.", self._name)

        # Loop over all ranges in the LaserScan.
        for idx, r in enumerate(msg.ranges):
            
            # Randomly throw out some rays to speed this up.
            if np.random.rand() > self._random_downsample:
                continue
            elif np.isnan(r):
                continue

            # Get angle of this ray in fixed frame.
            # TODO!
            angle = msg.angle_min + idx * msg.angle_increment

            far = False
            # Throw out this point if it is too close or too far away.
            if r > msg.range_max:
                far = True
                r = msg.range_max
            if r < msg.range_min:
                rospy.logwarn("%s: Range %f < %f was too small.",
                              self._name, r, msg.range_min)

                continue
       
            for i in np.arange(0, r +self._x_res, self._x_res):
                x = sensor_x + i * math.cos(yaw + angle)
                y = sensor_y + i * math.sin(yaw + angle)
                (idx_x, idx_y) = self.PointToVoxel(x, y)


                
                if far != True and (i >= r) and (self._map[idx_x][idx_y] < self._occupied_threshold) :
                    self._map[idx_x][idx_y] = self._map[idx_x][idx_y] + self._occupied_update
                elif (i < r) and (self._map[idx_x][idx_y] > self._free_threshold):
                    self._map[idx_x][idx_y] = self._map[idx_x][idx_y] + self._free_update
                else:
                    self._map[idx_x][idx_y] = self._map[idx_x][idx_y] + self._free_update
            # Walk along this ray from the scan point to the sensor.
            # Update log-odds at each voxel along the way.
            # Only update each voxel once. 
            # The occupancy grid is stored in self._map
            # TODO!

        # Visualize.
        self.Visualize()



    
    # Convert (x, y) coordinates in fixed frame to grid coordinates.
    def PointToVoxel(self, x, y):
        grid_x = int((x - self._x_min) / self._x_res)
        grid_y = int((y - self._y_min) / self._y_res)

        return (grid_x, grid_y)

    # Get the center point (x, y) corresponding to the given voxel.
    def VoxelCenter(self, ii, jj):
        center_x = self._x_min + (0.5 + ii) * self._x_res
        center_y = self._y_min + (0.5 + jj) * self._y_res

        return (center_x, center_y)

    # Convert between probabity and log-odds.
    def ProbabilityToLogOdds(self, p):
        return np.log(p / (1.0 - p))

    def LogOddsToProbability(self, l):
        return 1.0 / (1.0 + np.exp(-l))

    # Colormap to take log odds at a voxel to a RGBA color.
    def Colormap(self, ii, jj):
        p = self.LogOddsToProbability(self._map[ii, jj])

        c = ColorRGBA()
        c.r = p
        c.g = 0.1
        c.b = 1.0 - p
        c.a = 0.75
        return c

    # Visualize the map as a collection of flat cubes instead of
    # as a built-in OccupancyGrid message, since that gives us more
    # flexibility for things like color maps and stuff.
    # See http://wiki.ros.org/rviz/DisplayTypes/Marker for a brief tutorial.
    def Visualize(self):
        m = Marker()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = self._fixed_frame
        m.ns = "map"
        m.id = 0
        m.type = Marker.CUBE_LIST
        m.action = Marker.ADD
        m.scale.x = self._x_res
        m.scale.y = self._y_res
        m.scale.z = 0.01

        for ii in range(self._x_num):
            for jj in range(self._y_num):
                p = Point(0.0, 0.0, 0.0)
                (p.x, p.y) = self.VoxelCenter(ii, jj)

                m.points.append(p)
                m.colors.append(self.Colormap(ii, jj))

        self._vis_pub.publish(m)