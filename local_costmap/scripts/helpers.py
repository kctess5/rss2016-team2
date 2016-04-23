#!/usr/bin/env python
from __future__ import print_function
import rospy

import numpy as np
import math
from scipy import ndimage
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, Quaternion, Pose
import whoami

memo_table = {}

# fetch the info from ros for a given dot separated path in the yaml config file
def param(raw_name):
    # cache params
    if raw_name in memo_table:
        return memo_table[raw_name]

    name = raw_name.split(".")
    # print("PARAM QUERY:", name)

    # grab param object from param server
    param = rospy.get_param("/local_costmap/" + name[0])

    def drill(key, p):
        try:
            # returns the value corresponding to the given key from the param object p
            p = filter(lambda x: x.keys()[0] == key, p)[0]
            k = p.keys()[0]
            p = p[k]
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

def is_fucked(param):
    return type(param) is list and type(param[0]) is dict

# unpack the horrible packing of nested ros params into a dict
def unfuck_params(param):
    obj = {}
    for entry in param:
        for k in entry.keys():
            obj[k] = unfuck_params(entry[k]) if is_fucked(entry[k]) else entry[k]
    return obj

def polar_to_euclid(angles, ranges):
    x = np.cos(angles)
    y = np.sin(angles)
    x = x * ranges
    y = y * ranges
    return (x,y)

def euclidean_distance(p1, p2):
    dx = p1.x - p2.x
    dy = p1.y - p2.y
    return math.sqrt(dx*dx+dy*dy)

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

    def world_to_ind(self, x, y):
        """ Maybe incorrect now. """
        if (self.min_x < x < self.max_x) and (self.min_y < y < self.max_y):
            return (int(y * self.discretization) + self.y0, int(x * self.discretization) + self.x0)
        return (None, None)

    def ind_to_world(self, x_ind, y_ind):
        return 

    def clear(self):
        width = (self.max_x - self.min_x) * self.discretization
        height = (self.max_y - self.min_y) * self.discretization

        self.buffer = np.ones((width, height))

    def add_sample(self, x, y):
        if (self.min_x < x < self.max_x) and (self.min_y < y < self.max_y):
            xind = int(x * self.discretization)
            yind = int(y * self.discretization)
            # add obstacle to the buffer
            self.buffer[xind + self.x0][yind + self.y0] = False

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