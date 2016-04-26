""" Provides a goal pose based on sensor data, trying to follow the challenge route """
import rospy
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion
import whoami
import math
import tf.transformations
import numpy as np
from helpers import param

class Navigator(object):

    STRAIGHT_AHEAD = [15, 0, 0]

    def __init__(self, viz):
        self.viz = viz
        self.goal = self.STRAIGHT_AHEAD

        # Each wall is a list of 2 points [[x0, y0], [x1, y1]]
        self.walls = []
        # Corridors are derived from walls (the spaces inbetween)
        self.corridors = []

    def laser_update(self, laser_data):
        DISCONTINUITY_THRESHOLD = 1. # meters
        INF = 1000.
        ranges = np.array(laser_data.ranges)
        # Set range_max points to be really far, to force a discontinuity
        # (this might be only needed for the simulator)
        ranges[ranges + 0.1 > laser_data.range_max] = INF
        angles = (np.arange(ranges.shape[0]) * laser_data.angle_increment) + laser_data.angle_min
        include_indices = np.where((angles > -math.pi/2.) & (angles < math.pi/2.))
        ranges = ranges[include_indices]
        angles = angles[include_indices]
        discontinuities = np.hstack(([False],
                np.abs(ranges[:-1] - ranges[1:]) > DISCONTINUITY_THRESHOLD))
        labeled = np.cumsum(discontinuities)
        labels = np.unique(labeled)
        new_walls = []
        for label in labels:
            indices = np.where(labeled == label)[0]
            i_left = indices[0]
            i_right = indices[-1]
            if i_right - i_left < 5:
                # Skip really short "walls"
                continue
            if ranges[i_left] == INF:
                # Skip adding a "wall" when it's the edge of the vision range
                continue
            left_point  = self._polar_to_point(ranges[i_left],  angles[i_left])
            right_point = self._polar_to_point(ranges[i_right], angles[i_right])
            new_walls.append([left_point, right_point])
        self.walls = new_walls
        self.corridors = np.array([[lw[1], rw[0]] for lw, rw in zip(self.walls, self.walls[1:])])
        # Only include wide enough corridors
        self.corridors = filter(lambda c: self._length(c) > 1, self.corridors)
        # TODO: the corridor is not best described by the endpoints of each wall,
        # but rather the endpoint of one wall and the closest point on the other wall
        # (whichever combo comes out shorter). This would capture the "doorway".

        if len(self.corridors) > 0:
            # Set goal to the rightmost corridor
            [[x0, y0], [x1, y1]] = self.corridors[0] # [[x0, y0], [x1, y1]]
            centerpoint_x = (x0 + x1)/2.
            centerpoint_y = (y0 + y1)/2.
            centerpoint_heading = math.atan2(y1 - y0, x1 - x0) - math.pi/2.
            self.goal = [centerpoint_x, centerpoint_y, centerpoint_heading]
        else:
            self.goal = self.STRAIGHT_AHEAD

    def _polar_to_point(self, distance, angle):
        """ returns [x,y] """
        return [distance*math.cos(angle), distance*math.sin(angle)]

    def _length(self, segment):
        [[x0, y0],[x1, y1]] = segment
        l = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)
        # print l
        return l

    def camera_update(self, camera_data):
        """
        Incorporates the camera data into its goal output
        ( This method doesn't return a value )
        """
        pass

    def goalpoint(self):
        """
        Returns the current goal point as a list [x, y, heading] in the base frame.
        """
        return self.goal[:]

    def visualize(self):
        """
        Output a marker for the goal point, and markers for the walls & corridors
        """

        if self.viz.should_visualize("goals.next_goal"):
            self.viz.publish("goals.next_goal", self._make_pose_marker(self.goal, ColorRGBA(0,1,1,1)))
        if self.viz.should_visualize("goals.walls"):
            self.viz.publish("goals.walls", self._make_segment_markers(self.walls, ColorRGBA(1,0,1,1)))
        if self.viz.should_visualize("goals.corridors"):
            self.viz.publish("goals.corridors", self._make_segment_markers(self.corridors, ColorRGBA(0,1,0,1)))

    def _init_marker(self):
        x = Marker()
        x.header = Header(
                stamp=rospy.Time.now(),
                frame_id="hokuyo_link")
        x.ns = "navigator"
        x.id = 0
        x.action = 0
        x.lifetime = rospy.Duration.from_sec(10.)
        x.points = []
        return x

    def _make_pose_marker(self, pose, color):
        """ pose is [x,y,heading] """
        goal = self._init_marker()
        goal.type = Marker.ARROW
        goal.color = color
        goal.pose = Pose()
        goal.pose.position.z = 0.
        goal.pose.position.x = pose[0]
        goal.pose.position.y = pose[1]
        goal.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,pose[2]))
        goal.scale = Vector3(.5, .1, .1)
        return goal

    def _make_segment_markers(self, segments, color):
        walls = self._init_marker()
        walls.type = Marker.LINE_LIST
        walls.color = color
        for wall in segments:
            walls.points.append(Point(wall[0][0], wall[0][1], 0))
            walls.points.append(Point(wall[1][0], wall[1][1], 0))
        walls.scale.x = .1
        return walls


