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
from helpers import *
import warnings
from profilehooks import profile

class Navigator(object):

    STRAIGHT_AHEAD = (Point2D(param("navigator.straight_ahead_distance"), 0), 0)
    MIN_WALL_LENGTH = param("navigator.min_wall_length")
    MIN_CORRIDOR_WIDTH_2 = param("navigator.min_corridor_width")**2

    def __init__(self, viz):
        self.viz = viz
        self.goal = self.STRAIGHT_AHEAD # (Point2D, heading)
        self.sm = SplitMerge()

        # These are saved for viz
        self.walls = [] # list of Segments
        self.imagined_wall = None # Segment
        self.corridor = None # Segment

    def laser_update(self, laser_data):
        ranges = np.array(laser_data.ranges)
        angles = (np.arange(ranges.shape[0]) * laser_data.angle_increment) + laser_data.angle_min
        # Exclude out-of-bounds ranges by distance too far and by angle too deep
        include_indices = np.where((ranges + 0.1 < laser_data.range_max) &
                (angles > -math.pi/2.) & (angles < math.pi/2.))
        ranges = ranges[include_indices]
        angles = angles[include_indices]
        self.find_walls(ranges, angles)
        self.find_good_corridors()
        #self.find_right_corridor()

        if len(self.corridors) > 0:
            # Set goal to the rightmost corridor
            [[x0, y0], [x1, y1]] = self.corridors[0] # [[x0, y0], [x1, y1]]
            centerpoint_x = (x0 + x1)/2.
            centerpoint_y = (y0 + y1)/2.
            centerpoint_heading = math.atan2(y1 - y0, x1 - x0) - math.pi/2.
            self.goal = (Point2D(centerpoint_x, centerpoint_y), centerpoint_heading)
            if distance2(self.goal[0], Point2D(0,0)) < 8:
                # Move the goal point farr away
                x = self.goal[0].x + math.cos(self.goal[1])*5
                y = self.goal[0].y + math.sin(self.goal[1])*5
                self.goal = (Point2D(x,y), self.goal[1])
        else:
            self.goal = self.STRAIGHT_AHEAD

    def find_walls(self, ranges, angles):
        """ sets self.walls. """
        #print "started Split-Merge"
        points = [polar_to_point(r,a) for r,a in zip(ranges, angles)]
        walls = self.sm.run(points)
        self.walls = filter(lambda wall: length(wall) > self.MIN_WALL_LENGTH, walls)
        #print "finished Split-Merge", len(walls)

    def find_good_corridors(self):
        """ Find corridors """
        corridors = []
        for w2, w1 in zip(self.walls, self.walls[1:]):
            p1 = w1.p2
            p2 = w2.p1
            improved_p1 = closest_point_segment(w2, p1)
            improved_p2 = closest_point_segment(w1, p2)
            if distance2(improved_p1, improved_p2) > self.MIN_CORRIDOR_WIDTH_2:
                corridors.append(Segment(improved_p1, improved_p2))
        self.corridors = corridors


    def find_right_corridor(self):
        """ Sets self.corridors (to contain only 1 corridor) based on self.walls. 
        Algorithm:
        * Rightmost wall = w1
        * Endpoint of w1 = p1
        * Next wall = w2
        * Point on w2 closest to p1 = p2
        * if distance(p1, p2) < min start over with w2 as w1
          ( That excludes cases where this is not a discontinuity )
        * If discontinuity is right of center or over center:
          * w3 = line through p1 parallel to w2
          * p3 = point on w3 closest to p2
        * If discontinuity is left of center:
          * w3 = line through p2 parallel to w1
          * p3 = point on w3 closest to p1
        * Corridor = p2--p3
        """
        walls = self.walls
        while len(walls) > 2: # Can try to place a corridor
            w1 = walls[0]
            p1 = w1.p1
            w2 = walls[1]
            if point_in_segment(w2, p1):
                p2 = closest_point(segment_to_line(w2), p1)
            else:
                p2 = min((w2.p1, w2.p2), key=lambda point: distance2(point, p1))
            if distance2(p1, p2)< self.MIN_CORRIDOR_WIDTH_2:
                walls = walls[1:]
                continue
            if p1.x < 0: # This is a corridor on the right (or center)
                delta = Point2D(p1.x - p2.x, p1.y - p2.y)
                w3_p1 = Point2D(w2.p1.x + delta.x, w2.p1.y + delta.y)
                w3_p2 = Point2D(w2.p2.x + delta.x, w2.p2.y + delta.y)
                w3_seg = Segment(w3_p1, w3_p2)
                w3_line = segment_to_line(w3_seg)
                p3 = closest_point(w3_line, p2)
                self.corridors = [Segment(p3, p2)]
                self.imagined_wall = w3_seg
                return
            else:
                delta = Point2D(p2.x - p1.x, p2.y - p1.y)
                w3_p1 = Point2D(w1.p1.x + delta.x, w1.p1.y + delta.y)
                w3_p2 = Point2D(w1.p2.x + delta.x, w1.p2.y + delta.y)
                w3_seg = Segment(w3_p1, w3_p2)
                w3_line = segment_to_line(w3_seg)
                p3 = closest_point(w3_line, p1)
                self.corridors = [Segment(p1, p3)]
                self.imagined_wall = w3_seg
                return
        self.corridors = []
        self.imagined_wall = None


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
        return (self.goal[0].x, self.goal[0].y, self.goal[1])

    def visualize(self):
        """
        Output a marker for the goal point, and markers for the walls & corridors
        """

        if self.viz.should_visualize("goals.next_goal"):
            self.viz.publish("goals.next_goal", self._make_pose_marker(self.goal, ColorRGBA(0,1,1,1)))
        if self.viz.should_visualize("goals.walls"):
            self.viz.publish("goals.walls", self._make_segment_markers(self.walls, ColorRGBA(1,0,1,1)))
            #self.viz.publish("goals.imagined_wall", self._make_segment_markers([self.imagined_wall], ColorRGBA(0,1,1,1)))
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
        """ pose is (Point2D(x,y),heading) """
        goal = self._init_marker()
        goal.type = Marker.ARROW
        goal.color = color
        goal.pose = Pose()
        goal.pose.position.z = 0.
        goal.pose.position.x = pose[0].x
        goal.pose.position.y = pose[0].y
        goal.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,pose[1]))
        goal.scale = Vector3(.5, .1, .1)
        return goal

    def _make_segment_markers(self, segments, color):
        walls = self._init_marker()
        walls.type = Marker.LINE_LIST
        walls.color = color
        for wall in segments:
            walls.points.append(Point(wall.p1.x, wall.p1.y, 0))
            walls.points.append(Point(wall.p2.x, wall.p2.y, 0))
        walls.scale.x = .1
        return walls


class SplitMerge(object):
    """ Split-and-Merge algorithm """
    def __init__(self):
        # This is the *squared* discontinuity
        self.discontinuity_threshold_2 = param("navigator.min_corridor_width")
        self.angle_error = param("navigator.same_wall_error_angle")
        self.distance_error = param("navigator.same_wall_error_distance")
        self.point_in_segment_error = self.distance_error
        warnings.simplefilter('ignore', np.RankWarning)
        self.downsample = param("navigator.downsample_every")

    #@profile(sort='tottime')
    def run(self, points):
        """ Main entry point to the algorithm.
        points is a list of Point2Ds
        Returns a list of Walls
        """
        #return self.merge(self.split(points[::5]))
        all_split = []
        for spoints in self.presplit(points[::self.downsample]):
            all_split.extend(self.split(spoints))
            #all_split.append(points_to_segment(spoints))
        return self.merge(all_split)

    def presplit(self, points):
        """
        Input: orderd list of Point2Ds
        Output: list of list of Point2Ds
        """
        splits = [-1]
        for i, (p1, p2) in enumerate(zip(points, points[1:])):
            if distance2(p1, p2) > self.discontinuity_threshold_2:
                splits.append(i)
        splits.append(len(points)-1)
        output = []
        for i,j in zip(splits, splits[1:]):
            output.append(points[i+1:j+1])
        #print len(output), "presplit segments"
        return output

    def split(self, points, iters=0):
        """
        Input: ordered list of Point2Ds
        Output: list of Segments
        """
        #if iters == 0:
        #    print "splitting", len(points), "points"
        if points == []:
            return []
        line = fit_line(points)
        def error2(point):
            return distance2(point, closest_point(line, point))
        outliers = filter(lambda point: error2(point) > self.point_in_segment_error**2, points)
        if outliers == []:
            return [points_to_segment(points)]
        # Worst is the closest point to the middle which is worse than the threshold
        # (This is a modification to basic split-merge which fixes an edge case)
        worst = min(outliers, key=lambda point: abs(points.index(point)-len(points)/2.))
        i = points.index(worst)
        left = points[:i]
        right = points[i+1:]
        #if iters % 100 == 0:
        #    print "split depth", iters
        if error2(worst) > self.point_in_segment_error**2:
            r = self.split(left, iters+1) + self.split(right, iters+1)
            #if iters == 0:
                #print "FINISHED SPLIT", len(r)
            return r
        else:
            #if iters == 0:
            #    print "FINISH SPLIT WITH NO ERROR"
            return [points_to_segment(points)]

    def merge(self, segments, iters=0):
        """
        Input: List of Segments
        Output: List of Segments
        """
        #if iters % 50 == 1:
        #    print "split depth", iters
        for i1, (l1, l2) in enumerate(zip(segments, segments[1:])):
            if collinear(l1, l2, self.angle_error, self.distance_error):
                new_segment = merge_segments(l1,l2)
                new_segments = segments[:i1] + [new_segment] + segments[i1+2:] # preserve order
                r = self.merge(new_segments, iters+1)
                #if iters == 0:
                #    print "FINISHED MERGE", len(r)
                return r
        #if iters == 0:
        #    print "FINISHED MERGE WITH NO STEPS", segments
        return segments

