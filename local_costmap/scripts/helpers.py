#!/usr/bin/env python
from __future__ import print_function
import rospy

import numpy as np
import math, collections, recordclass
from scipy import ndimage
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, Quaternion, Pose
import whoami

memo_table = {}

Point2D = collections.namedtuple("Point2D", ["x", "y"])
Segment = collections.namedtuple("Segment", ["p1", "p2"])
Line = collections.namedtuple("Line", ["m", "b"]) # y = mx + b
State = collections.namedtuple("State", 
    ["x", "y", "theta", "steering_angle", "speed"])
AccelerationState = collections.namedtuple("AccelerationState", 
    ["control_states", "steering_velocity", "linear_accel"])
DynamicAccelerationState = collections.namedtuple("DynamicAccelerationState", 
    ["control_states", "steering_velocity", "linear_accel", "step_size", "integration_steps"])
Circle = collections.namedtuple("Circle", ["radius", "x", "y", "deflection"])
CirclePathState = collections.namedtuple("CirclePathState", ["circle", "dist_goal"])

Path  = collections.namedtuple("Path", ["states"])
# min and max are both States, with values set to the min/max of each field
StateRange = collections.namedtuple("StateRange", ["min", "max"])
# wraps a state with additional information for search
SearchNode = collections.namedtuple("SearchNode", ["state", "cost", "heuristic", "parent", "tree_node"])
# used for recreating the search tree in visualization
TreeNode = recordclass.recordclass("TreeNode", ["state", "children"])

def spline_circle_intersection(circle, spline):
    # find upper bound on t for binary search
    L = circle.radius
    t_min = spline.min
    t_max = spline.max

    def binary_search(low, high, func, count=0):
        # count += 1
        check = (high + low) / 2.0
        val = func(check)
        if abs(val) < 0.01:
            return check
        elif abs(low-high) < 0.0001:
            return None
        elif val < 0:
            return binary_search(low, check, func, count)
        elif val > 0:
            return binary_search(check, high, func, count)

    def path_dist2(t):
        p = spline(t)
        x_diff = p[0] - circle.x
        y_diff = p[1] - circle.y
        return L - math.sqrt(x_diff*x_diff+y_diff*y_diff)

    intersection = binary_search(t_min, t_max, path_dist2)
    if intersection == None:
        return None
    else:
        p = spline(intersection)
        return Point2D(x=p[0], y=p[1])
    # print(count)
    # if intersection:
    #     p = spline(intersection)
    #     print(intersection, p, math.sqrt(p[0]*p[0]+p[1]*p[1]))
    #     print(path_dist2(intersection))
    
    # print("test", t_max, spline(t), intersection)


def circle_segment_intersection(circle, start_point, end_point):
    start_point = np.array([start_point.x, start_point.y])
    end_point = np.array([end_point.x, end_point.y])
    circle_center = np.array([circle.x, circle.y])

    d = end_point - start_point
    f = start_point - circle_center

    a = np.dot(d,d)
    b = 2.0 * np.dot(f,d)
    c = np.dot(f,f) - circle.radius * circle.radius

    discriminant = b*b - 4.0*a*c
    if discriminant < 0:
        # no intersection
        return None
    else:
        # ray didn't totally miss sphere,
        # so there is a solution to
        # the equation.
        discriminant = np.sqrt(discriminant)

        t1 = (-b - discriminant) / (2.0 * a)
        t2 = (-b + discriminant) / (2.0 * a)

        if t1 >= 0 and t1 <= 1:
            p = d * t1 + start_point
            return Point2D(x=p[0], y=p[1])
        if t2 >= 0 and t2 <= 1:
            p = d * t2 + start_point
            return Point2D(x=p[0], y=p[1])
        
        return None

# fetch the info from ros for a given dot separated path in the yaml config file
def param(raw_name):
    # cache params
    if raw_name in memo_table:
        return memo_table[raw_name]

    name = raw_name.split(".")
    # print("PARAM QUERY:", name)

    # grab param object from param server
    if name[0] == "dynamics":
        name.pop(0)
        param = rospy.get_param("/dynamics/" + name[0])
    else:
        param = rospy.get_param("/local_costmap/" + name[0])
        
    def drill(key, p):
        try:
            return p[key]
        except:
            rospy.logerr("Could not find key in config: %s", raw_name)

        return p

    if name[0] == "runtime_specific":
        param = drill("racecar", param) if whoami.is_racecar() else drill("not_racecar", param)
    
    name.pop(0)

    # drill into param objet if necessary
    while name:
        # extract next level
        param = drill(name[0], param)
        # pop namespace specifier
        name.pop(0)

    # print ("RETURNING:", param)
    memo_table[raw_name] = param
    return param

def polar_to_euclid(angles, ranges):
    xs = ranges * np.cos(angles)
    ys = ranges * np.sin(angles)
    return (xs, ys)

def polar_to_point(distance, angle):
    """ returns Point2D(x,y) """
    x,y = polar_to_euclid(angle, distance)
    return Point2D(x,y)

def distance2(p1, p2):
    dx = p1.x - p2.x
    dy = p1.y - p2.y
    return dx*dx+dy*dy


def euclidean_distance(p1, p2):
    return math.sqrt(distance2(p1, p2))

def length(segment):
    return euclidean_distance(segment.p1, segment.p2)

def length2(segment):
    return distance2(segment.p1, segment.p2)

def fit_line(points):
    """ Returns a Line which best fits the Point2Ds """
    xs = [point.x for point in points]
    ys = [point.y for point in points]
    m,b = np.polyfit(xs, ys, 1)
    return Line(m, b)

def segment_to_line(segment):
    p1, p2 = segment
    m = (p1.y - p2.y)/float(p1.x - p2.x)
    b = p1.y - m * p1.x
    return Line(m, b)

def point_in_segment(segment, point):
    """ True iff the closest point to point on the segment is not an end point of the segment. """
    def subtract(p1, p2):
        """ p1 - p2 """
        return Point2D(p1.x - p2.x, p1.y - p2.y)
    def dot(p1, p2):
        """ p1 dot p2 """
        return p1.x*p2.x + p1.y*p2.y
    return 0 < dot(subtract(point, segment.p1), subtract(segment.p2, segment.p1)) < length2(segment)

def closest_point_segment(segment, point):
    if point_in_segment(segment, point):
        return closest_point(segment_to_line(segment), point)
    return min(segment.p1, segment.p2, key=lambda p: distance2(p, point))

def closest_point(line, point):
    """
    Find the closest point on a Line to a Point2D.
    Returns: Point2D(on_point_x, on_point_y)
    """
    main_line_slope, main_line_intercept = line
    off_point_x, off_point_y = point
    if main_line_slope == 0:
        # If main_line is horizontal, then the result can quickly be determined.
        return Point2D(off_point_x, main_line_intercept)
    else:
        # perp_line slope is main_line slope rotated 90deg.
        perp_line_slope = - 1 / float(main_line_slope)
    # perp_line crosses off_point.
    perp_line_intercept = off_point_y - perp_line_slope * off_point_x

    # on_point is the intersection of main_line and perp_line.
    on_point_x = (main_line_intercept - perp_line_intercept) / float(perp_line_slope - main_line_slope)
    # Plug on_point_x into main_line.
    on_point_y = main_line_slope * on_point_x + main_line_intercept

    return Point2D(on_point_x, on_point_y)

def points_to_segment(points):
    """ List of Point2Ds to a Segment using best fit line (so endpoints might not be in points) """
    line = fit_line(points)
    p1 = closest_point(line, points[0])
    p2 = closest_point(line, points[-1])
    return Segment(p1, p2)

def merge_segments(s1, s2):
    """ Create a single Segment from these 2 Segments. Assumes they're IN ORDER """
    return points_to_segment([s1.p1, s1.p2, s2.p1, s2.p2])

def angdiff(a,b):
    """Smallest difference between two angles."""
    return abs(math.atan2(math.sin(a-b), math.cos(a-b)))

def segment_angle(s):
    """Get the angle between two points."""
    return math.atan2(s.p2.y - s.p1.y, s.p2.x - s.p1.y)

def collinear(s1, s2, angle_error, distance_error):
    """ True iff the Lines of each Segment differ by less than the error Line """
    if angdiff(segment_angle(s1), segment_angle(s2)) > angle_error:
        return False
    for p1 in s1:
        for p2 in s2:
            if p1 != p2 and euclidean_distance(p1, p2) < distance_error:
                return True
    return False

class FPSCounter(object):
    """docstring for FPSCounter"""
    def __init__(self, enabled=True, size=10):
        self.buffer = np.zeros(size)
        self.index = 0
        self.last_step = rospy.get_rostime().to_sec()
        self.enabled = enabled

    def step(self):
        if self.enabled:
            t = rospy.get_rostime().to_sec()
            delta = t - self.last_step
            self.last_step = t
            self.buffer[self.index % len(self.buffer)] = delta
            self.index += 1

    def fps(self):
        m = np.mean(self.buffer)
        if m > 0:
            fps = 1.0 / m
        else:
            fps = 30.0
        
        if fps < 2.0 or fps > 40.0:
            fps = 30.0

        return fps

        # if m == 0:
        #     return 0

        # fps = 1.0 / m
        # return round(float(self.count) / (rospy.get_rostime().to_sec() - self.start_time), 2)

class FrameBuffer:
    # bins/meter, (meters), (meters)
    def __init__(self, resolution=10, x_range=(-1,6), y_range=(-4,4)):
        self.discretization = resolution # bins per meter
        self.max_x = x_range[1]
        self.max_y = y_range[1]
        self.min_x = x_range[0]
        self.min_y = y_range[0]

        self.x0 = 0
        self.y0 = 0
        self.distmap = None
        self.seq = 0

        self.find_center()
        self.clear()

    def find_center(self):
        self.x0 = abs(int(self.min_x * self.discretization))
        self.y0 = abs(int(self.min_y * self.discretization))

    # def world_to_ind(self, x, y):
    #     """ Maybe incorrect now. """
    #     if (self.min_x < x < self.max_x) and (self.min_y < y < self.max_y):
    #         return (int(y * self.discretization) + self.y0, int(x * self.discretization) + self.x0)
    #     return (None, None)

    # def ind_to_world(self, x_ind, y_ind):
    #     return 

    def clear(self):
        width = (self.max_x - self.min_x) * self.discretization
        height = (self.max_y - self.min_y) * self.discretization

        self.buffer = np.ones((width, height))

    # def add_sample(self, x, y):
    #     if (self.min_x < x < self.max_x) and (self.min_y < y < self.max_y):
    #         xind = int(x * self.discretization)
    #         yind = int(y * self.discretization)
    #         # add obstacle to the buffer
    #         self.buffer[xind + self.x0][yind + self.y0] = False

    def add_samples(self, xs, ys):
        """Add many samples to the buffer."""
        # Filter out out of bounds samples.
        include_indices = np.where(
            (xs > self.min_x) &
            (xs < self.max_x) &
            (ys > self.min_y) &
            (ys < self.max_y))
        xs = xs[include_indices]
        ys = ys[include_indices]

        # Translate into discretized index space.
        xinds = (xs * self.discretization + self.x0).astype(int)
        yinds = (ys * self.discretization + self.y0).astype(int)

        # Add obstacle to the buffer.
        self.buffer[(xinds, yinds)] = False

    def dist_transform(self):
        # Show a stripe to visualize coordinate system.
        # self.buffer[2,:] = False
        # self.buffer[:,10] = False
        # self.buffer[:,20] = False

        self.distmap = ndimage.distance_transform_edt(self.buffer)
        return self.distmap

    def dist_at(self, x, y):
        if (self.min_x < x < self.max_x) and (self.min_y < y < self.max_y) and not self.distmap == None:
            xind = int(x * self.discretization)
            yind = int(y * self.discretization)
            # add obstacle to the buffer
            return self.distmap[xind + self.x0][yind + self.y0] / self.discretization
        return 1000000

    def get_map(self, transfer_fxn):
        # return nav_msgs/OccupancyGrid of the contained buffer
        omap = np.copy(self.distmap)
        it = np.nditer(omap, flags=['multi_index'], op_flags=['writeonly'])

        # apply the given transfer function, and return the resultant costmap
        while not it.finished:
            x = float(it.multi_index[0]-self.x0+2) / self.discretization * .999
            y = float(it.multi_index[1]-self.y0+2) / self.discretization * .999

            it[0] = transfer_fxn(x, y)

            it.iternext()

        return self.serialize_numpy_map(omap)

    def serialize_numpy_map(self, omap):
        og = OccupancyGrid()
        
        og.header.stamp = rospy.Time.now()
        og.header.frame_id = "base_link"
        og.header.seq = self.seq

        origin = Point(-self.x0/self.discretization, -self.y0/self.discretization, 0.0)
        og.info.origin = Pose(origin, angle_to_quaternion(0))
        og.info.resolution = 1.0 / float(self.discretization) # meters/cell
        og.info.width  = (self.max_x - self.min_x)*self.discretization # number of cells
        og.info.height = (self.max_y - self.min_y)*self.discretization # number of cells

        flat_grid = (omap.flatten(order='F')*100).clip(0,100)
        og.data = list(np.round(flat_grid))
        return og
