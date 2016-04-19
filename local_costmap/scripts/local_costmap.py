#!/usr/bin/env python
from __future__ import print_function
import rospy
from scipy import ndimage
from nav_msgs.msg import OccupancyGrid, MapMetaData
import tf.transformations
import collections
import time
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion, Pose

from helper_functions import nondeterministic_weighted_index, exploration_weighted_index
import whoami
import pathlib
from pathlib import Path
import pathsearch
import whinytimer
import navigator

import threading
import numpy as np
import math
import matplotlib.pyplot as plt

from visualization_driver import VisualizationDriver
from car_controller.control_module import ControlModule

# Coarse lock.
# Used to serialize incoming asynchronous actions like timers and topic subscriptions.
FILE_LOCK = threading.RLock()

def param(name):
    return rospy.get_param("/local_costmap/" + name)

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

"""
    ================================================================================
    MSG: nav_msgs/OccupancyGrid
        # This represents a 2-D grid map, in which each cell represents the probability of
        # occupancy.

        Header header 

        #MetaData for the map
        MapMetaData info

        # The map data, in row-major order, starting with (0,0).  Occupancy
        # probabilities are in the range [0,1  Unknown is -1.
        int8[] data

    ================================================================================
    MSG: std_msgs/Header
        # Standard metadata for higher-level stamped data types.
        # This is generally used to communicate timestamped data 
        # in a particular coordinate frame.
        # 
        # sequence ID: consecutively increasing ID 
        uint32 seq
        #Two-integer timestamp that is expressed as:
        # * stamp.secs: seconds (stamp_secs) since epoch
        # * stamp.nsecs: nanoseconds since stamp_secs
        # time-handling sugar is provided by the client library
        time stamp
        #Frame this data is associated with
        # 0: no frame
        # 1: global frame
        string frame_id

    ================================================================================
    MSG: nav_msgs/MapMetaData
        # This hold basic information about the characterists of the OccupancyGrid

        # The time at which the map was loaded
        time map_load_time
        # The map resolution [m/cell]
        float32 resolution
        # Map width [cells]
        uint32 width
        # Map height [cells]
        uint32 height
        # The origin of the map [m, m, rad].  This is the real-world pose of the
        # cell (0,0) in the map.
        geometry_msgs/Pose origin
        ================================================================================
        MSG: geometry_msgs/Pose
        # A representation of pose in free space, composed of postion and orientation. 
        Point position
        Quaternion orientation

    ================================================================================
    MSG: geometry_msgs/Point
        # This contains the position of a point in free space
        float64 x
        float64 y
        float64 z

    ================================================================================
    MSG: geometry_msgs/Quaternion
        # This represents an orientation in free space in quaternion form.

        float64 x
        float64 y
        float64 z
        float64 w
"""

def polar_to_euclid(angles, ranges):
    x = np.cos(angles)
    y = np.sin(angles)
    x = x * ranges
    y = y * ranges
    return (x,y)

# plt.ion()

class LocalCostmap(object):
    """  Generates a costmap based off of sensor data from the car, without any global sense of map """
    def __init__(self, VISUALIZE=False):

        self.visualize = VISUALIZE

        # the class which manages the local costmap buffer
        # bins/meter, (meters), (meters)
        self.buffer = FrameBuffer(5, (-3,10), (-8,8))
        self.first_laser_recieved = False
        self.im = None
        self.dirty = False

        self.LASER_SCAN_TOPIC = "/scan" if whoami.is_racecar() else "/racecar/laser/scan"
        self.scan_subscriber = rospy.Subscriber(\
                self.LASER_SCAN_TOPIC, numpy_msg(LaserScan), self.scan_callback, queue_size=1)

        # self.pub_costmap = rospy.Publisher('~costmap', OccupancyGrid, queue_size=1)

    def filter_lasers(self, angles, ranges, range_min, range_max):
        # do nothing
        # return (angles, ranges)
        # remove the data on the edges, since they are noisy
        l = round(angles.shape[0] / 10)
        
        return (angles[l:-l], ranges[l:-l])

    def scan_callback(self, data):
        # print ("received laser scan")
        start = time.clock()

        if not self.first_laser_recieved:
            print(("first laser received: "
                    "angle_min: %f angle_max %f angle_incr: %f "
                    "ranges_len: %d range_min: %.2f range_max_ %2.2f") % (
                    data.angle_min, data.angle_max, data.angle_increment, 
                        data.ranges.shape[0], data.range_min, data.range_max))

        laser_angles = np.linspace(data.angle_min, data.angle_max, math.ceil(\
                (data.angle_max - data.angle_min) / data.angle_increment))
        laser_ranges = data.ranges

        laser_angles, laser_ranges  = self.filter_lasers(\
                laser_angles, laser_ranges, data.range_min, data.range_max)
        laser_x, laser_y =  polar_to_euclid(laser_angles, laser_ranges)

        # compute the distance transform from the laser scanner data
        with FILE_LOCK:
            self.buffer.clear()
            for x, y in zip(laser_x, laser_y):
                self.buffer.add_sample(x,y)
            self.buffer.dist_transform()

        # print ("Done computing distance map in:", time.clock() - start, "seconds")

        # if self.visualize == True:
        #     if self.im == None:
        #         self.im = plt.imshow(self.buffer.distmap)
        #         plt.colorbar(self.im)
        #     else:
        #         self.im.set_array(self.buffer.distmap)
        #     plt.draw()

        self.first_laser_recieved = True  
        self.mark_clean()

    def cost_at(self, x, y):
        xp = [0, 0.5, 10]
        fp = [1.0, 0.1, 0]
        z = np.interp(self.buffer.dist_at(x, y), xp, fp)
        return z * z

    def dist_at(self, x, y):
        return self.buffer.dist_at(x, y)

    # used to ensure that each costmap is only used once, to avoid wasted compute
    def mark_clean(self):
        self.dirty = False
    def is_dirty(self):
        return self.dirty
    def mark_dirty(self):
        self.dirty = True

    def get_map(self):
        return self.buffer.get_map(self.cost_at)

'''
    def generate_candidate_paths(self):
        curvatures = np.linspace(0, self.max_curve, num=self.path_candidates)

        paths = []

        ys = np.linspace(0, self.path_length, num=self.path_discretization)
        xs = np.zeros(self.path_discretization)

        paths.append(Path(0, self.max_speed, zip(xs,ys)))

        for curvature in curvatures[1:]:
            # set the speed profile
            if self.FAST_MODE:
                xp = [0,  .13, .2, .3,  self.max_curve]
                fp = [1,   1,  .6, .4,  .2]
            else:
                xp = [0,  .15, .2,  self.max_curve]
                fp = [1,   1,  .75,   .4]
            
            multiplier = np.interp(curvature, xp, fp)

            r = self.radius(curvature)
            # theta = self.path_length * math.sqrt(multiplier) / r
            theta = self.path_length / r
            thetas = np.linspace(0, theta, num=self.path_discretization)

            ys = r * np.sin(thetas)
            xs = r * np.cos(thetas) - r

            paths.append(Path(curvature, self.max_speed * multiplier, zip(xs, ys)))
            paths.append(Path(-1*curvature, self.max_speed * multiplier, zip(-1*xs, ys)))

        return paths
'''

class PathGenerator(object):
    """Generate paths to evaluate.
    Assumes a coordinate system in which the robot is at the origin and +X is forward.
    """

    def __init__(self):
        # The maximum length of a path in meters.
        self.PATH_LENGTH = param("max_path_length")
        # How long each leg of the path is in meters.
        self.FIXED_SPEED = param("fixed_speed")
        self.PATH_CANDIDATES = param("path_candidates")
        self.MAX_CURVE = param("max_curve") # maximum path curvature in radians
        self.PATH_DISCRETIZATION = param("path_discretization") # num points to evaluate for each path
        self.WHEEL_BASE = param("wheel_base")

    def generate_paths(self, costmap):
        # Return only one simple candidate path.
        # path = pathlib.path_constant_curve(
        #     wheel_base=self.WHEEL_BASE,
        #     steering_angle=.1,
        #     travel_distance=self.PATH_LENGTH, npoints=self.PATH_DISCRETIZATION,
        #     start_x=0, start_y=0, start_heading=0)
        # return [path]

        # Return a fan of constant curvature arcs.
        # steering_angle_max = self.MAX_CURVE
        # paths = pathlib.paths_fan(
        #     wheel_base=self.WHEEL_BASE,
        #     steering_angle_min=-steering_angle_max, steering_angle_max=steering_angle_max,
        #     npaths=10, travel_distance=2., npoints_perpath=30,
        #     start_x=0, start_y=0, start_heading=0)
        # return paths

        # Return a forking tree of paths.
        # steering_angle_max = self.MAX_CURVE
        # paths = pathlib.paths_forking(
        #     wheel_base=self.WHEEL_BASE,
        #     steering_angle_min=-steering_angle_max, steering_angle_max=steering_angle_max,
        #     npaths=10, nfork=4, step_distance=.1, fork_distance=1.4, travel_distance=self.PATH_LENGTH,
        #     start_x=0, start_y=0, start_heading=0)
        # return paths

        def heuristic_fn(state):
            """Function used to guide search.
            A lower return values means better.
            """
            # Prefer lower cost areas.
            x, y, heading = state.waypoints[-1]
            cost = costmap.cost_at(x, y)
            # Prefer positive x movement.
            forward = min(x, 2.)
            # Prefer lower curvature per distance traveled.
            curvyness = np.sum(np.square(np.absolute(state.steering_angles))) / state.length
            return cost + 0.001 * -forward + 0.001 * curvyness

        IMPASSIBLE_THRESHOLD = param("impassible_threshold")
        def cull_fn(x, y, heading):
            # Allow paths which are not too close to obstacles.
            obstacle_dist = costmap.dist_at(x, y)
            return obstacle_dist > IMPASSIBLE_THRESHOLD

        search = pathsearch.PathSearch(cull_fn, heuristic_fn)
        search.crunch(credits=25)

        return search.best(n=1)

    # def radius(self, curve):
    #     return self.WHEEL_BASE / np.tan(curve)

    # def generate_paths(self):
    #     ''' Return a list of Path namedtuples for later evaluation.
    #     '''

    #     curvatures = np.linspace(0, self.MAX_CURVE, num=self.PATH_CANDIDATES)
    #     paths = []

    #     ys = np.linspace(0, self.PATH_LENGTH, num=self.PATH_DISCRETIZATION)
    #     xs = np.zeros(self.PATH_DISCRETIZATION)

    #     paths.append(Path(steering_angle=0, waypoints=zip(xs,ys)[1:], speed=self.FIXED_SPEED))

    #     for curvature in curvatures[1:]:
    #         r = self.radius(curvature)
    #         theta = self.PATH_LENGTH / r
    #         thetas = np.linspace(0, theta, num=self.PATH_DISCRETIZATION)

    #         ys = r * np.sin(thetas)
    #         xs = r * np.cos(thetas) - r

    #         paths.append(Path(steering_angle=-1*curvature, waypoints=zip(xs,ys)[1:], speed=self.FIXED_SPEED))
    #         paths.append(Path(steering_angle=curvature, waypoints=zip(-1*xs,ys)[1:], speed=self.FIXED_SPEED))

    #     return paths

class PathEvaluator(object):
    def __init__(self):
        self.IMPASSIBLE_THRESHOLD = param("impassible_threshold")
        pass

    def path_cost(self, path, costmap):
        cost = 0
        impassible = 0
        # print (path.waypoints[0], path.waypoints[1])
        for waypoint in path.waypoints:
            c = costmap.cost_at(waypoint[0], waypoint[1])
            # print(c)
            if c > self.IMPASSIBLE_THRESHOLD:
                impassible += 1
            cost += c
        # print (impassible)
        if impassible >= 1:
            return None
        return cost

    def evaluate_paths(self, paths, costmap):
        ''' Return cost metrics for each path. 
            The lowest cost path will be chosen for execution.
        '''
        return [self.path_cost(i, costmap) for i in paths]

class LocalExplorer(ControlModule):
    def __init__(self, VISUALIZE=False):
        super(LocalExplorer, self).__init__("local_costmap_explorer")

        # TODO inherit from controller thing
        self.PLANNING_FREQ = param("planning_freq")
        self.VISUALIZE = VISUALIZE
        self.visualize_timer = whinytimer.EveryN(0)
        self.BACKUP_SPEED = param("backup_speed")
        self.BACKUP_DURATION = param("backup_duration")

        self.MIN_COST_PICK = param("min_cost_pick")
        self.COST_WEIGHTED_PROB = param("cost_weighted_prob")
        self.COST_WEIGHTED_PROB_TUNABLE = param("cost_weighted_prob_tunable")
        self.INIT_ALPHA = 1./param("exploration_level")

        self.started_backup = 0
        
        self.costmap = LocalCostmap(VISUALIZE)
        self.path_gen = PathGenerator()
        self.path_eval = PathEvaluator()
        self.navigator = navigator.Navigator(VISUALIZE)
        self.visualization_driver = VisualizationDriver()
        self.costmap_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)

	# self.path_pick = self.COST_WEIGHTED_PROB_TUNABLE # play with this
	self.path_pick = self.MIN_COST_PICK # play with this
	self.alpha = self.INIT_ALPHA # exploration param, also play with this

        whinytimer.WhinyTimer(rospy.Duration(1.0 / self.PLANNING_FREQ), self.timer_callback)
        rospy.on_shutdown(lambda: self.on_shutdown())

    def start_backing_up(self):
        self.started_backup = time.time()

    def is_backing_up(self):
        return abs(time.time() - self.started_backup) < self.BACKUP_DURATION

    def back_up(self):
        control_msg = self.make_message("direct_drive")
        control_msg.drive_msg.speed = -1 * self.BACKUP_SPEED
        control_msg.drive_msg.steering_angle = 0

        self.control_pub.publish(control_msg)
        self.costmap.mark_dirty()

    def timer_callback(self, event=None):
        with FILE_LOCK:
            self._timer_callback(event=event)

    def _timer_callback(self, event=None):
        # prevents recomputing the same control from an old costmap
        if self.costmap.is_dirty():
            print("dirty costmap")
            return

        if self.is_backing_up():
            return self.back_up()

        # print("timer timer_callback")
        rospy.logdebug("Computing control in LocalExplorer")

        paths = self.path_gen.generate_paths(self.costmap)
        costs = self.path_eval.evaluate_paths(paths, self.costmap)

        assert len(paths) == len(costs)

        # best_path = paths[min(range(len(costs)), key=lambda i: costs[i])]

        viable_paths = []
        for i in xrange(len(costs)):
            if not costs[i] == None:
                viable_paths.append((costs[i], paths[i]))

        print(len(viable_paths), "viable paths")

        if len(viable_paths) == 0:
            self.start_backing_up()
            return self.back_up()
            # best_path = Path(steering_angle=0, waypoints=[], speed=0)
        else:
            if self.path_pick == self.MIN_COST_PICK:
                # Choose the lowest cost of the viable paths.
                best_path = min(viable_paths, key=lambda (cost,path): cost)[1]
            else:
                weights = [1./(path[0]+.01) for path in viable_paths]
                if self.path_pick == self.COST_WEIGHTED_PROB:
                    # tries out choosing path by inverse of cost
                    i = nondeterministic_weighted_index(weights)
                elif self.path_pick == self.COST_WEIGHTED_PROB_TUNABLE:
                    # Introduces tunable exploration parameter (alpha)
                    i = exploration_weighted_index(weights, self.alpha)
                best_path = viable_paths[i][1]

        # Visualizations.
        if self.VISUALIZE and self.visualize_timer.step(auto_reset=True):
            # self.visualization_driver.publish_candidate_waypoints([v[1] for v in viable_paths], costmap=self.costmap)
            # self.visualization_driver.publish_candidate_waypoints(paths, costmap=self.costmap)
            # self.visualization_driver.publish_best_waypoints(best_path, costmap=self.costmap)
            self.navigator.visualize()
            self.visualization_driver.publish_best_path(best_path, costmap=self.costmap)
            # self.visualization_driver.publish_candidate_paths([vp[1] for vp in viable_paths], costmap=self.costmap)
            # print()
            # print(self.costmap.get_map())
            # self.costmap_pub.publish(self.costmap.get_map())

        control_msg = self.make_message("direct_drive")
        control_msg.drive_msg.speed = best_path.speed
        control_msg.drive_msg.steering_angle = best_path.steering_angle

        self.control_pub.publish(control_msg)
        self.costmap.mark_dirty()

    def on_shutdown(self):
        """Stop the car."""
        control_msg = self.make_message("direct_drive")
        control_msg.drive_msg.speed = 0
        control_msg.drive_msg.steering_angle = 0
        self.control_pub.publish(control_msg)


def angle_to_quaternion(angle):
    """Convert an angle in radians into a quaternion _message_."""
    return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))


if __name__ == '__main__':
    try:
        viz = param("racecar_visualize") if whoami.is_racecar() else param("vm_visualize")
        LocalExplorer(viz)
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
