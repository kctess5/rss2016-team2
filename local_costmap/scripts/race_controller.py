#!/usr/bin/env python
from __future__ import print_function
import rospy
import threading, time, collections, heapq, itertools, math, sys, recordclass
import numpy as np
import os

# message related imports
from sensor_msgs.msg import LaserScan
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point as Point_msg

# code from other files in this repo
import navigator2 as navigator
from visualization_driver import VisualizationDriver
from helpers import param, euclidean_distance, FrameBuffer, polar_to_euclid
from helpers import State, AccelerationState, Circle, CirclePathState, Path, StateRange, SearchNode, TreeNode, FPSCounter, circle_segment_intersection, spline_circle_intersection
from helpers import Point2D as Point
from pathlib import arc_step, ackerman_radius
from pathlib2 import arc_step_fast
from car_controller.direct_control_module import DirectControlModule
import whinytimer
from profilehooks import profile, timecall
from heapq import nsmallest
from whoami import is_racecar
import matplotlib.pyplot as plt
from navigator3 import ScanMaximaNavigator
from curve_lib import C_factory

# from shapely.geometry import Point as GeoPoint
# from shapely.ops import union
# from dynamical import DynamicModel

# Check that the dubins library supports path_length_segs.
try:
    import dubins
except ImportError:
    raise RuntimeError("Package 'dubins' does not support dubins.DubinsPath().path_length_segs.\n"
                       "Run $ sudo pip install git+https://github.com/mlsteele/pydubins.git")
try:
    dubins.DubinsPath((0,0,0), (0,0,0), 1.).path_length_segs
except AttributeError:
    raise RuntimeError("Package 'dubins' does not support dubins.DubinsPath().path_length_segs.\n"
                       "Run $ sudo pip install git+https://github.com/mlsteele/pydubins.git")

"""
Notes: 
  - Relies on the following external files:
      - navigator.py
      - helpers.py
      - pathlib.py 
          - very handy :D
      - whinytimer.py
          - death be to ros_time_moved_backwards error
  - Can be further split into different files if necessary:
      - DynamicsModel -> dynamics.py
      - ObstacleMap -> obstacles.py
      - HeuristicSearch -> pathsearch.py
      - I think everything else is sufficiently short and important at the moment to stay
        and I personally think that having most important logic in one file is good when 
        possible, because code folding is super effective
  - Info about states/paths:
      - States contain:
          x, y, theta - position/orientation local coordinates, based off of predicted motion
          speed, steering_angle - control commands for the car
      - I found that speed settings on the car generally correspond to real speed of the car
        so long as the maximal acceleration bounds are observed, so it does not have to be
        decoupled from x,y,theta does. Steering angle is not directly related to changes in
        theta, so the dynamic model is required to estimate where the car will end up after
        applying a chain of control states
      - Paths are just a list of states, they are applied consecutively. For now, the first
        state in the path defines the origin of the local coordinate space for that time step
        eventually we might want to use a "global" local coorinate space, where rather than
        resetting the origin at each timestep, move the believed "car position" according to 
        the full chain of states previously committed. This might make continious tree 
        construction more straightforward as we would not have to transform each node in the 
        search tree into the new local coorinate space between each iteration. It would 
        also be useful for visualizing how well the dynamical open-loop controller predicts motion
  - The emergency stop control is implemented as follows:
      - in the case where there are no remaining states to execute in the last committed 
        best path, the car will reuse it's previous control state while stopping as fast as possible
      - in the case where there is a path committed, but the path planner is unable to find
        a new path, it will continue the last committed path while stopping as fast as possible
      - if there are remaining control states then the path will continue as planned
  - Visualization
      - Subjects include:
          - goals
              - next goal (1)
              - corridor detector
                  - corridor (1)
                  - walls (2)
          - obstacles
              - in-admissible region (3) (NOT IMPLEMENTED)
          - path search
              - best path (1)
              - viable paths (1 or 2)
              - speed curves (1)
              - steering curves (1)
      - Notes:
          - Each topic does not advertise unless there are subscribers
 
Todos:
  - Accumulate the x, y, theta positions in the state history so that the percieved path may be 
    reconstructed relative to the start state
      - might also make continuous tree refinement easier, if that is necessary
  - Finish Dynamical model
      - still need to include the data I collected, planning on doing this before Sunday
      - I think that this will naturally improve search a lot, because faster paths will have
        lower curvature
  - Better heuristics
  - Potentially different path cost metrics
  - Continiously refining a single tree, rather than building a whole new one on every timestep
  - Decoupling path planning and path execution so that more than one path segment is executed per plan
  - Vision based goal integration
  - Cooridor detector should detect dead ends and put a goal point behind the robot if one is found

Citations:
    [1] C. Chen, M. Rickert, and A. Knoll, "Combining space exploration and
        heuristic search in online motion planning for nonholonomic vehicles",
        in Proc. IEEE Intelligent Vehicles Symposium, 2013, pp. 1307-1312.
        http://www6.in.tum.de/Main/Publications/ChenChao2013a.pdf

    [2] O. Brock and L. E. Kavraki, "Decomposition-based motion planning:
        Towards real-time planning for robots with many degrees of freedom,"
        Rice University, Houston, TX, USA, Tech. Rep. TR00-367, Aug. 2000.

"""


class DynamicPlot():
    def initialize(self):
        plt.ion()
        #Set up plot
        self.fig = plt.figure(figsize=plt.figaspect(2.))
        
        self.ax0 = self.fig.add_subplot(1,1,1)
        self.ax1 = self.fig.add_subplot(1,2,2)

        self.redraw()
        
    def redraw(self):
        #We need to draw *and* flush
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


if not is_racecar() and param("obstacle_map.vizualize_distmap"):
    LOCAL_VIZ = DynamicPlot()
    LOCAL_VIZ.initialize()

class DynamicModel(object):
    """ Encapsulates the dynamics of the car """
    def __init__(self):
        # self.max_accel = float(param("dynamics.max_linear_accel")) / float(param("planning_freq"))
        # self.max_decel = float(param("dynamics.max_linear_decel")) / float(param("execution_freq"))
        self.wheelbase = param("dynamics.wheelbase")
        self.execution_freq = float(param("execution_freq"))

        self.raw_data = param("dynamics.steering_prediction")
        self.raw_data.sort(key=lambda x: x["steering_angle"])
        self.sorted_angles = map(lambda x: x["steering_angle"], self.raw_data)
        self.ackerman_transition = map(lambda x: x["ackerman_transition"], self.raw_data)

        self.min_dynamic_angle = self.sorted_angles[0]
        self.max_dynamic_angle = self.sorted_angles[-1]

        self.TABLE = {}

    def arc_estimate(self, sample_index, speed, steering_angle):
        if speed < self.ackerman_transition[sample_index]:
            # use ackerman steering
            return ackerman_radius(self.wheelbase, steering_angle)
        else:
            # estimate arc radius based off of nonlinear model
            coeffs = self.raw_data[sample_index]["polynomial_coefficients"]
            return coeffs[0] * speed * speed + coeffs[1] * speed + coeffs[2]
       
    def neighbor_samples(self, a):
        # find closest two entries for the given angle, along with interpolatoin weights.
        if a < self.min_dynamic_angle or a > self.max_dynamic_angle:
            return None
        elif self.min_dynamic_angle == a:
            return [(1, 0)]
        elif self.max_dynamic_angle == a:
            return [(1, len(self.raw_data)-1)]
        else:

            for i in xrange(len(self.sorted_angles)-1):
                if self.sorted_angles[i] == a:
                    return [(1,i)]
                elif self.sorted_angles[i] < a and a < self.sorted_angles[i+1]:
                    # find distance
                    w1 = a - self.sorted_angles[i]
                    w2 = self.sorted_angles[i+1] - a

                    # normalize
                    t = w1 + w2
                    w1 = 1 - w1 / t
                    w2 = 1 - w2 / t
                    return [(w1,i), (w2, i+1)]
        assert(False)
        return 

    def estimate_effective_arc(self, state):
        # return the expected path arc radius of the given state
        #   special cases: steering = 0 -> 0
        #                  steering < 0 -> -arc_radius
        
        sign = 1 if state.steering_angle >= 0 else -1

        if (state.speed, state.steering_angle) in self.TABLE:
            return sign*self.TABLE[(state.speed, abs(state.steering_angle))]
        else:
            # print("not")
            a = abs(state.steering_angle)

            effective_radius = 0

            neighbors = self.neighbor_samples(a)

            if a == 0:
                effective_radius = 0.0
            elif neighbors == None:
                # outside of good data range, use ackerman TODO: expand the data range with approximation
                effective_radius = ackerman_radius(self.wheelbase, a)
            elif len(neighbors) == 1:
                # compute value of given angle
                effective_radius = self.arc_estimate(neighbors[0][1], state.speed, state.steering_angle)
            elif len(neighbors) == 2:
                # interpolate between two nearest angles
                effective_radius = neighbors[0][0]*self.arc_estimate(neighbors[0][1], state.speed, state.steering_angle) \
                                 + neighbors[1][0]*self.arc_estimate(neighbors[1][1], state.speed, state.steering_angle)
            
            self.TABLE[(state.speed, abs(state.steering_angle))] = effective_radius

            return sign*effective_radius

    def propagate(self, state, t=1.0):
        """ Gives an estimate for the final pose after assuming a given state after a given time. t=1 is time time of the next control timestep
        """
        effective_radius = self.estimate_effective_arc(state)
            
        propagated = arc_step_fast(effective_radius, t * state.speed / self.execution_freq,
            state.x, state.y, state.theta)
        return State(x=propagated[0], y=propagated[1], theta=propagated[2], \
            steering_angle=state.steering_angle, speed=state.speed)

    def propagate_time(self, state, t=1.0):
        """ Gives an estimate for the final pose after assuming a given state after a given time. t=1 is time time of the next control timestep
        """
        effective_radius = self.estimate_effective_arc(state)
            
        propagated = arc_step_fast(effective_radius, t * state.speed, state.x, state.y, state.theta)
        return State(x=propagated[0], y=propagated[1], theta=propagated[2], \
            steering_angle=state.steering_angle, speed=state.speed)

    def propagate_accel(self, start_state, linear_accel, steering_velocity, num_segments):
        # NOTE: this function returns a list that DOES NOT include the given start state as 
        # it is assumed to be the last state in the previous path segment
        # given a start state and accelerations, this returns a list of control states that make up the action
        # control_states = [start_state]
        control_states = []
        ls = start_state

        # return DYNAMICS.propagate_accel(start)

        # [(x,y,theta,steering,speed),...]

        for i in xrange(num_segments):
            # bound the speed to [0, max_speed]
            next_speed = max(min(param("dynamics.max_speed"), ls.speed + linear_accel / self.execution_freq), 0)
            # bound the angle to [-max_deflection, max_deflection]
            next_steering = ls.steering_angle + steering_velocity / self.execution_freq
            next_steering = max(min(param("dynamics.max_deflection"), next_steering), -param("dynamics.max_deflection"))
            
            # fix small accumulated error
            if abs(next_steering) < param("epsilon"):
                next_steering = 0.0

            # add the new control and old position to new state
            ns = State(x=ls.x, y=ls.y, theta=ls.theta, speed=next_speed, steering_angle=next_steering)
            # propagate the old position forward in time
            propagated = self.propagate(ns)
            control_states.append(propagated)
            # set the most recent step as the last state for use in the next iteration
            ls = propagated

        return control_states

    def max_speed_at_turning_radius(self, turning_radius):
        """Return the maximum achievable speed at a given turning radius.
        Args:
            turning_radius: Turning radius to consider.
        Returns: Maximum speed in meters.
                 This will not exceed dynamics.max_speed.
        """
        if turning_radius > 5:
            return param("dynamics.max_speed")
        x = turning_radius
        curve = (-0.14*x*x + 1.38*x +1.151 ) * param("planner.speed_safety")
        return min(param("dynamics.max_speed"), curve)

    def dubins_time(self, q0, q1, turning_radii):
        """Shortest time to traverse from q0 to q1 along one of the dubins curves.
        Args:
            q0, q1: Pose tuples (x, y, heading) of start and end.
            turning_radii: Turning radii to consider.
                           A dubins curve is considered for each radius and the minimum time is returned.
        """
        assert(len(turning_radii) > 0)
        min_time = float("inf")

        # These are dubins curve types which have a straight middle section.
        # All other types have a curve in the middle.
        MID_STRAIGHT_TYPES = [dubins.LSL, dubins.LSR, dubins.RSL]

        max_straight_speed = param("dynamics.max_speed")

        for turning_radius in turning_radii:
            max_curved_speed = min(max_straight_speed, self.max_speed_at_turning_radius(turning_radius))
            path = dubins.DubinsPath(q0, q1, turning_radius)

            # Distances along the three sections of the dubins curve.
            # Tuple of 3 distances.
            distances = path.path_length_segs()

            # Time to traverse is the sum of the times to traverse each section.
            if path.path_type() in MID_STRAIGHT_TYPES:
                # Case where the middle section is straight.
                path_time = ((distances[0] + distances[2]) / max_curved_speed
                        + distances[1] / max_straight_speed)
            else:
                # Case where all 3 sections are curved.
                path_time = sum(distances) / max_curved_speed

            min_time = min(min_time, path_time)

        return min_time

DYNAMICS = DynamicModel()

# from skimage.morphology import medial_axis, skeletonize
# from scipy.ndimage.filters import sobel, gaussian_filter
# from skimage.feature import canny

class ObstacleMap(object):
    """ Manages obstacle information from the scanner to provide admissibility information """
    def __init__(self):
        self.buffer = FrameBuffer(param("obstacle_map.discretization"), \
            (param("obstacle_map.xmin"),param("obstacle_map.xmax")), \
            (param("obstacle_map.ymin"),param("obstacle_map.ymax")))
        self.first_laser_recieved = False
        self.lock = threading.Lock() # used to serialize accesses to the main buffer
        # origin of the costmap in the coordinate space that the car operates in
        self.origin = State(x=0,y=0,theta=0,steering_angle=0,speed=0)

    def filter_lasers(self, angles, ranges, laser_data):
        # Include only data which is in bounds.
        include_indices = np.where((ranges + 0.1 < laser_data.range_max))
        return angles[include_indices], ranges[include_indices]

    # @profile(sort="cumtime")
    def scan_callback(self, data):
        if not self.first_laser_recieved:
            print(("first laser received: "
                    "angle_min: %f angle_max %f angle_incr: %f "
                    "ranges_len: %d range_min: %.2f range_max_ %2.2f") % (
                    data.angle_min, data.angle_max, data.angle_increment, 
                        data.ranges.shape[0], data.range_min, data.range_max))
            self.laser_angles = np.linspace(data.angle_min, data.angle_max, math.ceil(\
                (data.angle_max - data.angle_min) / data.angle_increment))
        
        laser_angles, laser_ranges  = self.filter_lasers(self.laser_angles, data.ranges, data)
        laser_x, laser_y =  polar_to_euclid(laser_angles, laser_ranges)

        # compute the distance transform from the laser scanner data
        self.buffer.clear()
        self.buffer.add_samples(laser_x, laser_y)
        self.buffer.dist_transform()
        
        self.first_laser_recieved = True
        self.mark_clean()

    def is_admissible(self, state):
        # returns true if the given control state is admissible
        # steps through t values between 0 and 1 while checking that each point is 
        # above the minimum allowed distance
        if state.speed == 0:
            # only check the state position
            return self.buffer.dist_at(state.x, state.y) > param("obstacle_map.min_distance")

        t_step = param("obstacle_map.admissible_hop") / ( param("obstacle_map.discretization") * state.speed )
        t = t_step

        state_admissible = True

        # with self.lock:
        while t < 1.0+t_step:
            if t > 1.0:
                t = 1.0
            waypoint = DYNAMICS.propagate(state, t)
            if self.buffer.dist_at(waypoint.x, waypoint.y) < param("obstacle_map.min_distance"):
                state_admissible = False
                break
            
            t += t_step

        return state_admissible

    def dist_at(self, state):
        return self.buffer.dist_at(state.x, state.y)

    # used to ensure/sanity check that each costmap is only used once, to avoid wasted compute
    def mark_clean(self):
        self.dirty = False

    def is_dirty(self):
        return self.dirty

    def mark_dirty(self):
        self.dirty = True

    def viz(self):
        if not self.buffer.distmap == None and param("obstacle_map.vizualize_distmap"):        
            # dist = self.buffer.distmap
            # skel = gaussian_filter(dist, sigma=1)
            # skel = canny(skel)
            # skel = sobel(self.buffer.buffer)

            # dist /= (dist.max()/255.0)
            # skel /= (skel.max()/255.0)

            LOCAL_VIZ.ax0.imshow(self.buffer.distmap)
            # LOCAL_VIZ.ax0.imshow(skel)
            # LOCAL_VIZ.ax1.imshow(dist)
            LOCAL_VIZ.redraw()
        # self.viz.laser_euclid.set_data(self.expected_angles, self.expected_ranges)

class GoalManager(object):
    """ Manages and tracks the car's concept of purpose in life
            - This should wrap the corridor detector and the vision based goal detection
    """

    def __init__(self, viz, pass_rad=0.25, locality_rad=100, max_pts=5):
        """ Tunable parameters are kwargs above:
                - pass_rad = the radius threshold around the car for which a goal point
                    is considered "passed"
                - locality_rad = the radius threshold around each goal point for which a new
                    goal point is mapped as the same
                - max_pts = maximum number of points allowed in gp buffer
        """
        # self.navigator = navigator.Navigator(viz)
        self.navigator3 = ScanMaximaNavigator(viz)
        self.origin = None

        # TODO: Make these range-query trees/numpy arrays for larger max_pts value
        self.green_gps = []
        self.corr_gps = []

        self.pass_rad = pass_rad
        self.locality_rad = locality_rad
        self.green_sub = rospy.Subscriber(param("goal_manager.green_gp_topic"), Point_msg, self.cb_green)

    def cb_green(self, pt):
        """ Processes the goal points produced by Fernando's green patch detector
        """
        # Don't bother adding the default (0,0,0) "goal point"
        if pt.x == 0.0 and pt.y == 0.0 and pt.z == 0.0:
            return
        self.match((pt.x, pt.y, pt.z), "green")

    def match(self, new_pt, gp_type):
        """ Matches goal point with an existing one or appends to appropriate DS
                if new.
            - new_pt: The new point to integrate into managed points.
                Assumes point is in indexable (x,y,orientation) format
            - gp_type: either "green" or "corridor"
        """
        # NOTE: We L1 norm instead of L2 radius for fast calculation

        if new_pt == None:
            return
        x,y,orient = new_pt
        gp_list = self.green_gps if gp_type=="green" else self.corr_gps

        for i,gp in enumerate(gp_list):
            if abs(gp[0] - x) <= self.locality_rad and abs(gp[1] - y) <= self.locality_rad:
                # Update the point if match found
                gp_list[i] = new_pt
                return

        # Otherwise it's a new point, append
        gp_list.append(new_pt)
        return

    def next_goal(self):
        """ Return the position of the next goal in local coordinates
                - for now, this directly calls the corridor detector with no smoothing
        """
        new_gp = self.navigator3.goalpoint()
        self.match(new_gp, "corridor")

        # Prune out all passed points TODO: should this go before matching?
        self.check_passed()

        # Sort the individual goal point management lists
        # self.sort_gpls()

        # Car stops if no goal points
        if len(self.green_gps) == 0 and len(self.corr_gps) == 0:
            return None

        # For now, always prioritize green patches
        if len(self.green_gps) > 0:
            target_x, target_y, _ = self.green_gps[0]
        else:
            target_x, target_y, _ = self.corr_gps[0]

        # Publish target coordinates with direction of next goal point
        return State(x=target_x, y=target_y, theta=0, steering_angle=None, speed=None)

    def check_passed(self):
        """ Remove goal points if passed
        """
        # Criteria for passing a green patch
        for i,gp in enumerate(self.green_gps):
            if gp[0] <= self.pass_rad or gp[1] <= self.pass_rad:
                gpl.pop(i)

        # Criteria for passing a corridor
        for i,gp in enumerate(self.corr_gps):
            if gp[0] <= self.pass_rad or gp[1] <= self.pass_rad:
                gpl.pop(i)

    def sort_gpls(self):
        """ Sort goal points by Euclidean distance to car
        """
        self.green_gps.sort(key=lambda g: g[0]**2 + g[1]**2)
        self.corr_gps.sort(key=lambda g: g[0]**2 + g[1]**2)

    def scan_callback(self, data):
        self.navigator3.scan_callback(data)
        
        # self.navigator.laser_update(data)
        # self.navigator.visualize()

    def viz(self):
        self.navigator3.visualize()

class HeuristicSearch(object):
    """ Perform heuristic search on the provided set of cost/admissibility/heuristic/neighbor functions """
    def __init__(self):
        pass
        # self.reset()

    def bfs_tree(self, levels, start_state):
        if levels == 0:
            return TreeNode(state=start_state, children=[])
        else:
            children = self.neighbors(start_state)
            return TreeNode(state=start_state, children=map(lambda x: self.bfs_tree(levels-1, x), children))

    def reset(self, start_state):
        # Frontier is a priority queue.
        self.closed_set = []
        self.frontier = []
        self.found_paths = []
        self.goal_state = self.goal()
        self.step_count = 0
        self.tree_root = TreeNode(state=start_state, children=[])

        ss = SearchNode(state=start_state, parent=None, cost=0, tree_node=self.tree_root, \
                    heuristic=self.heuristic(start_state, self.goal_state))

        heapq.heappush(self.frontier, (ss.heuristic, ss))

    def search(self, time_limit):
        if self.goal_state == None:
            return None
        
        start_time = time.time()

        # extend nodes until the time limit is reached
        while time.time() - start_time < time_limit:
            if len(self.frontier) == 0:
                print("Search failed, bailing early")
                return
            self._step()

    def search_nsteps(self, step_limit):
        """Run a fixed number of steps.
        Using search is preferable. This exists to make profiling easier.
        """
        # extend nodes until the step limit is reached
        for _ in xrange(step_limit):
            if len(self.frontier) == 0:
                print("Search failed, bailing early")
                return
            self._step()

    def _step(self):
        """ Perform one iteration of heuristic search - extend a single node.
            Requires that self.frontier is not empty.
        """
        # Pop the best path to split.
        parent_score, parent_state = heapq.heappop(self.frontier)
        parent_tree_node = parent_state.tree_node

        for neighbor_state in self.neighbors(parent_state.state):
            # prune any path segments that are bound to fail
            if not self.should_bail(neighbor_state, self.goal_state) and self.is_admissible(neighbor_state):
                # build the tree representation of the search
                ntn = TreeNode(state=neighbor_state, children=[])
                parent_tree_node.children.append(ntn)
                
                nss = SearchNode(state=neighbor_state, parent=parent_state, tree_node=ntn, \
                    cost=parent_state.cost + self.cost(neighbor_state),  \
                    heuristic=self.heuristic(neighbor_state, self.goal_state))

                score = nss.cost + nss.heuristic
                if self.goal_met(neighbor_state, self.goal_state):
                    heapq.heappush(self.found_paths, (score, nss))
                else:
                    heapq.heappush(self.frontier, (score, nss))

        self.closed_set.append(parent_state.state)

        self.step_count += 1

    def best(self):
        # returns the best path found
        if len(self.frontier) == 0 and len(self.found_paths) == 0:
            return None
        elif len(self.found_paths) > 0:
            return self.make_path(self.found_paths[0][1])
        # elif len(self.frontier) == 0:
        #     return self.make_path(self.found_paths[0][1])
        elif len(self.found_paths) == 0:
            return self.make_path(self.frontier[0][1])

    def complete_paths(self):
        return map(lambda x: self.make_path(x[1]), self.found_paths)

    def make_path(self, end_node, add_goal=False, remove_start=True):
        # a path is a list of control states, in the order of traversal
        # builds it in reverse order by following parent pointers, then reverses result
        path = [end_node.state]
        while end_node.parent:
            path.append(end_node.parent.state)
            end_node = end_node.parent
        # don't execute the start state
        if remove_start:
            path.pop()
        path.reverse()

        if add_goal:
            path.append(self.goal_state)

        if type(end_node.state) == AccelerationState:
            path = reduce(lambda x,y: x+y, map(lambda x: x.control_states, path))

        return Path(states=path)

    """ The following functions should be overridden by the parent class
    """
    def cost(self, path):
        # estimate the cost of traversing a given path
        raise NotImplementedError("HeuristicSearch cost not specified")

    def heuristic(self, state, goal_state):
        # return an estimate for cost to go between the given state and the goal state
        raise NotImplementedError("HeuristicSearch heuristic not specified")

    def goal(self):
        # return the goal state
        raise NotImplementedError("HeuristicSearch goal not specified")

    def goal_met(self):
        # return true if the goal state is met
        raise NotImplementedError("HeuristicSearch goal not specified")

    def should_bail(self, state, goal_state):
        # return true if the node should be terminated
        return False

    def is_admissible(self, state):
        # return true if the given state is considered admissible - fulfills hard constraints
        raise NotImplementedError("HeuristicSearch is_admissible not specified")

    def neighbors(self, state):
        # return a discretized set of neighbors of the given state
        raise NotImplementedError("HeuristicSearch neighbors not specified")

class AccelerationPlanner(HeuristicSearch):
    """ Implements path planning by considering constant acceleration path segments
    """
    def __init__(self, obstacles, goals):
        self.circle_path = None
        # TODO parameterize this
        # self.steering_options = [[-1.0,0.0, 1.0], [-0.4, -.15, -0.04, 0.0, 0.04, 0.15, 0.4]]
        self.steering_options = [[-1.0,0.0, 1.0], [-1.0, -0.4, -.12, -0.04, 0.0, 0.04, 0.12, 0.4, 1.0]]
        self.steering_options = map(lambda l: map(lambda x: float(param("dynamics.max_angular_velocity"))*x, l), self.steering_options)

        self.obstacles = obstacles
        self.goals = goals

        # caching for performance
        cell_size = 1.0 / float(param("obstacle_map.discretization"))
        self.min_step = max(cell_size, param("obstacle_map.min_distance"))

        super(AccelerationPlanner, self).__init__()

    def reset(self, start_state, circle_path=None):
        self.circle_path = circle_path
        super(AccelerationPlanner, self).reset(start_state)
        
    def cost(self, accel_state):
        return len(accel_state.control_states) / float(param("execution_freq")) 

    # this big function is designed to check as few points as possible along the given path
    # it evaluates the distance function at the first point along the path, and then it steps
    # along the path by the maximum amount possible without completely stepping over an
    # inadmissible region. It does this by considering the value of the distance function
    # at each point along the way. Since this number is the distance to the nearest obstacle,
    # we can be sure to not step over an obstacle if we step by most that amount minus the
    # width of the inadmissible region around obstacles
    def is_admissible(self, accel_state):
        assert(len(accel_state.control_states) > 0)
        
        segment_lengths = map(lambda x: x.speed/float(param("execution_freq")), accel_state.control_states)
        dists = [0]
        for i in xrange(len(segment_lengths)):
            dists.append(segment_lengths[i] + dists[i])
        path_length = dists[-1]

        if path_length == 0:
            print("ZERO LENGTH PATH")
            # if the path is stationary, only need to check the first state
            p = accel_state.control_states[0]
            return self.obstacles.buffer.dist_at(p.x, p.y) > param("obstacle_map.min_distance")

        d_path = 0
        while d_path < path_length:
            # find next point along the path
            p = None
            for i in xrange(len(dists)-1):
                if dists[i] == d_path:
                    # distance falls directly on a control state
                    p = accel_state.control_states[i]
                    break
                elif dists[i] < d_path and d_path < dists[i+1]:
                    # distance falls in between path point, find intermediate state
                    # find t and generate intermediate state
                    start_state = accel_state.control_states[i]
                    seg_dist = d_path - dists[i]
                    seg_length = segment_lengths[i]
                    t = seg_dist / seg_length
                    p = DYNAMICS.propagate(start_state, t)
                    break

            # if not admissible, return false
            if not self.obstacles.buffer.dist_at(p.x, p.y) > param("obstacle_map.min_distance"):
                return False
            
            d_path += max(self.min_step, self.obstacles.buffer.dist_at(p.x, p.y)-param("obstacle_map.min_distance"))
        
        return True

    def heuristic_old(self, accel_state, goal_state):
        # return 0
        q0 = (accel_state.control_states[-1].x, accel_state.control_states[-1].y, accel_state.control_states[-1].theta)
        q1 = (goal_state.x, goal_state.y, goal_state.theta)
        turning_radius = param("dynamics.r_min")

        return param("planner.heuristic_bias")*dubins.path_length(q0, q1, turning_radius) / float(param("dynamics.max_speed"))

    def heuristic_old2(self, accel_state, goal_state):
        """Estimate the time it would take to get from state to goal_state.
        By taking a few dubins curves at different turning radii and returning the best time.
        """
        # return self.heuristic_old(accel_state, goal_state)
        q0 = (accel_state.control_states[-1].x, accel_state.control_states[-1].y, accel_state.control_states[-1].theta)
        q1 = (goal_state.x, goal_state.y, goal_state.theta)
        # Take the min of several dubins curves.
        # Turning radii to try.
        # TODO parameterize this curvature range.
        turning_radii = np.linspace(1.3,4.5, num=3)
        # time = DYNAMICS.dubins_time(q0, q1, [1.3])
        time = DYNAMICS.dubins_time(q0, q1, turning_radii)
        return time*param("planner.heuristic_bias")

    def heuristic(self, accel_state, goal_state):
        """Estimate the time it would take to get from state to goal_state.
        By taking a few dubins curves at different turning radii and returning the best time.
        """

        # if the final goal is nearby, we need to check every intermediate control state 
        # to get a better heuristic estimate, in case the chosen path overshoots
        # TODO might want to make the path terminate at the control state where the goal is reached
        q1 = (goal_state.x, goal_state.y, goal_state.theta)

        if euclidean_distance(accel_state.control_states[-1], goal_state) \
            < euclidean_distance(accel_state.control_states[-1], accel_state.control_states[0]):
            return param("planner.heuristic_bias")*min(map(lambda s: 
                DYNAMICS.dubins_time((s.x, s.y, s.theta), q1, [param("dynamics.r_min")]), accel_state.control_states))
        else:
            # only check the end state
            s = accel_state.control_states[-1]
            q0 = (s.x, s.y, s.theta)
            
            # Take the min of several dubins curves.
            # Turning radii to try.
            # TODO parameterize this curvature range.
            turning_radii = np.linspace(1.3,4.5, num=3)
            return param("planner.heuristic_bias") * DYNAMICS.dubins_time(q0, q1, turning_radii)

    def heuristic_circle(self, accel_state, goal_state):
        """Estimate the time it would take to get from state to goal_state.
        By taking a few dubins curves at different turning radii and returning the best time.
        """

        if self.circle_path:
            end_state = accel_state.control_states[-1]
            c = self.associated_circle(end_state)
            return param("planner.heuristic_bias") * ( euclidean_distance(c.circle, end_state) + c.dist_goal ) / param("dynamics.max_speed")
        else:
            return self.heuristic_new(accel_state, goal_state)

    def goal(self):
        # return the next goal state
        return self.goals.next_goal()

    def goal_met(self, accel_state, goal_state):
        # if the path length is large compared to the distance between the end and the goal
        # we check intermediate path points, otherwise we only check the end effector
        if not param("planner.test.enabled") and euclidean_distance(accel_state.control_states[-1], goal_state) \
            < euclidean_distance(accel_state.control_states[-1], accel_state.control_states[0]):
            return min(map(lambda x: 
                euclidean_distance(x, goal_state), accel_state.control_states)) < float(param("planner.goal_distance_threshold"))
        else:
            return euclidean_distance(accel_state.control_states[-1], goal_state) < float(param("planner.goal_distance_threshold"))

    def should_bail(self, state, goal_state):
        # bail if the end effector is further from the robot than the goal
        return euclidean_distance(state.control_states[-1], Point(x=0, y=0)) > euclidean_distance(goal_state, Point(x=0, y=0))

    def max_speed_given_dist(self, dist):
        # TODO might want to limit max speed depending on environmental factors
        return param("dynamics.max_speed")

    def associated_circle(self, control_state):
        closest = nsmallest(3, self.circle_path.states, key=lambda x: euclidean_distance(x.circle, control_state))
        # return whichever circle associates with the given control_state for the heuristic
        return min(closest, key=lambda x: x.dist_goal + euclidean_distance(x.circle, control_state))

    def step_size(self, state):
        if self.circle_path:
            c = self.associated_circle(state)
            return min(param("planner.alpha") * c.circle.radius, param("planner.beta") * c.dist_goal, param("planner.min_step"))
        else:
            return param("planner.min_step")

    def neighbors(self, accel_state):
        start_state = accel_state.control_states[-1]
        if start_state.speed == 0:
            num_segments = int(param("planner.max_segments"))
        else:
            step_size = self.step_size(start_state)
            num_segments = math.floor((step_size / abs(start_state.speed)) * float(param("execution_freq")))
            num_segments = int(min(max(num_segments, 1),  param("planner.max_segments")))
        # num_segments = param("planner.control_decisions_per_segment")
        # linear accel options: max accel, max decel, unity
        accel_options = []
        decel_options = []

        # limit the max speed at any given point in space to avoid going too fast near obstacles
        max_target_speed = self.max_speed_given_dist(self.obstacles.dist_at(start_state))
        current_speed = accel_state.control_states[-1].speed

        # this is positive if the car is over speed
        accel_target = param("execution_freq")*(max_target_speed - current_speed)/float(num_segments)

        if accel_target < param("dynamics.max_linear_decel"):
            # linear_accel_options = [param("dynamics.max_linear_decel"), param("dynamics.max_linear_accel")] 
            decel_options = [param("dynamics.max_linear_decel")] 
        elif accel_target > param("dynamics.max_linear_accel"):
            accel_options = [param("dynamics.max_linear_accel")]
            decel_options = [param("dynamics.max_linear_decel")]
        else:
            accel_options = [accel_target]
            decel_options = [param("dynamics.max_linear_decel")]
            # linear_accel_options = [param("dynamics.max_linear_decel"), accel_target]

        if start_state.speed < param("epsilon"):
            decel_options = []

        # NOTE: angular branch factor should be odd
        # only considers the admissible options
        steering_options = self.steering_options
        if abs(start_state.steering_angle + param("dynamics.max_deflection")) < param("epsilon"):
            steering_options = map(lambda x: filter(lambda opt: opt >= 0, x), steering_options)
        elif abs(start_state.steering_angle - param("dynamics.max_deflection")) < param("epsilon"):
            steering_options = map(lambda x: filter(lambda opt: opt <= 0, x), steering_options)
        

        candidate_controls = []
        if accel_options:
            candidate_controls += list(itertools.product(accel_options, steering_options[1]))
        if decel_options:
            candidate_controls += list(itertools.product(decel_options, steering_options[0]))

        # map each control choice to an actual search state, complete with intermediate control states
        return map( lambda cc: \
            AccelerationState(control_states=DYNAMICS.propagate_accel(start_state, cc[0], cc[1], num_segments), 
                steering_velocity=cc[1], linear_accel=cc[0]), candidate_controls)

class SpaceExploration(HeuristicSearch):
    """ This class implements lower dimensional search to provide a high quality
        heuristic for higher dimensional path planning. If this search fails to 
        provide a solution, then there is a good chance that no solution exists
        and the car should just stop

        See: [1,2]
    """
    def __init__(self, obstacles, goals):
        # cache reused values
        step = math.pi * 2.0 / param("space_explorer.branch_factor")
        self.thetas = np.linspace(0, math.pi * 2.0 - step, num=param("space_explorer.branch_factor"))
        # self.thetas = [0.5]
        bounds = param("space_explorer.sampling_bounds_initial")
        self.initial_thetas = np.linspace(bounds[0], bounds[1], num=param("space_explorer.branch_factor"))
        # self.thetas = [6.27]
        self.radii = np.empty(param("space_explorer.branch_factor"))
        
        self.goals = goals
        self.obstacles = obstacles

        super(SpaceExploration, self).__init__()

    def circle_radius(self, state):
        return min(param("space_explorer.max_circle_size"), self.obstacles.dist_at(state) - param("obstacle_map.min_distance"))

    def reset(self, start_state):
        # Frontier is a priority queue.
        start_state = Circle(x=start_state.x, y=start_state.y, radius=self.circle_radius(start_state), deflection=0)
        super(SpaceExploration, self).reset(start_state)

    def cost(self, state):
        if state.radius < param("space_explorer.min_radius"):
            return state.radius * param("space_explorer.soft_limit_penalty")
        return state.radius#*(1.0 + param("space_explorer.deflection_coeff")*abs(np.sin(state.deflection / 2.0)) )
    
    def heuristic(self, state, goal_state):
        if goal_state == None:
            return np.inf
        return (euclidean_distance(state,goal_state) - state.radius)*param("space_explorer.heuristic_bias")
    
    def overlap(self, s1, s2, percentage=.15):
        # TODO/NOTE: this is a bit of a hack to go faster, percentage overlap not accurate
        if euclidean_distance(s1, s2) > (s1.radius + s2.radius)*(1.0-percentage):
            return False
        else:
            return True

    def goal(self):
        g=self.goals.next_goal()
        if g == None:
            return None
        r = self.circle_radius(g)
        # if the goal is out of bounds it returns a very large value, fix that
        if r > 100:
            r = param("space_explorer.max_circle_size")
        return Circle(x=g.x, y=g.y, radius=r, deflection=0)

    def goal_met(self, state, goal_state):
        return self.overlap(state, goal_state, float(param("space_explorer.overlap_percentage_goal")))

    def is_admissible(self, state):
        return self.obstacles.dist_at(state) > param("obstacle_map.min_distance")

    def should_bail(self, state, goal_state):
        # too_far
        # if np.linalg.norm(np.array([state.x, state.y]))
        if euclidean_distance(state, Point(0,0)) > param("space_explorer.max_distance"):
            return True
        # too big
        if state.radius > param("space_explorer.max_distance"):
            return True
        # too behind
        if state.x < param("space_explorer.back_limit"):
            return True

        # return False
        # overlaps with another node
        return True in map(lambda x: euclidean_distance(x,state) < x.radius, self.closed_set)

    def search(self, time_limit):
        # same as before, but this returns early if paths are found
        start_time = time.time()

        # extend nodes until the time limit is reached
        while time.time() - start_time < time_limit and not len(self.found_paths):
            if len(self.frontier) == 0:
                print("Search failed, bailing early")
                return
            self._step()

        return time.time() - start_time

    def neighbors(self, state):
        self.radii.fill(state.radius)
        if state == self.tree_root.state:
            thetas = self.initial_thetas
        else:
            thetas = self.thetas
        
        xs, ys = polar_to_euclid(thetas, self.radii)

        def deflection(angle):
            if angle > math.pi:
                return angle - 2.0*math.pi
            return angle

        neighbors = map(lambda x: Circle( \
            x=state.x+x[0], y=state.y+x[1],  \
            radius=self.circle_radius(Point(x=state.x+x[0], y=state.y+x[1])), \
            deflection=deflection(x[2])),
                zip(xs,ys, thetas))

        # use the hard constraint here
        potential_neighbors = filter(lambda x: x.radius > param("space_explorer.hard_min_radius"), neighbors)

        return potential_neighbors

    def best(self):
        if len(self.found_paths) > 0:
            return self.make_path(self.found_paths[0][1], True, False)
        return None
    
class ChallengeController(DirectControlModule):
    """ Top level car control for the 6.141 Challenge"""
    def __init__(self):
        print(param("runtime_specific.racecar_env"))
        super(ChallengeController, self).__init__("challenge_controller", racecar_env=param("runtime_specific.racecar_env"))

        # initialize the control state management
        self.state_history = [State(x=0, y=0, theta=0, steering_angle=0, speed=0)]
        self.current_path = None
        self.state_index = 0

        # counter for tracking fps of control
        self.control_monitor = FPSCounter(True)
        self.execution_monitor = FPSCounter(False)

        self.execution_time_estimate = 1.0 / 25.0

        self.lookahead_circle = None

        # self.search_time_limit = 1.0/float(param("planning_freq"))
        self.computing_control = False
        self.since_scan_recieved = []

        # used for one off path tests
        self.test_started = False
        self.test_path = None
        self.control_steps_per_plan_interval = 1 # int(math.ceil(float(param("execution_freq")) * self.search_time_limit))

        # initialize peripheral algorithms
        self.viz = VisualizationDriver()

        self.goals = GoalManager(self.viz)
        self.obstacles = ObstacleMap()
            
        self.space_explorer = SpaceExploration(self.obstacles, self.goals)
        self.scan_subscriber = rospy.Subscriber(param("runtime_specific.scan_topic"),
            numpy_msg(LaserScan), self.scan_callback, queue_size=1)

        rospy.on_shutdown(lambda: self.on_shutdown())

        self.new_data = False
        self.scanner_data = None

        control_thread = threading.Thread(target=lambda: self._control_loop())
        control_thread.daemon = True
        control_thread.start()

        if param("obstacle_map.vizualize_distmap") or param("navigator3.show_local_viz"):
            while not rospy.is_shutdown():
                self.loop()
                rospy.sleep(0.05)

    def test_control(self):
        if not self.test_started:
            self.test_started = True
            start_state = State(x=0, y=0, theta=0, steering_angle=0, speed=0)
            start_accel_state = AccelerationState(control_states=[start_state], linear_accel=0, steering_velocity=0)
            
            self.path_planner.reset(start_state=start_accel_state)
            
            goal_state = param("planner.test.goal")
            self.path_planner.goal_state = State(x=goal_state[0], y=goal_state[1], theta=goal_state[2], steering_angle=0, speed=0)
            self.path_planner.search(time_limit=1.0)

            self.test_path = self.path_planner.best()

            # begin executing the new path. if no path is found, the path will be set to 
            # none, and the emergency planner can take over
            self.commit_path(self.test_path) 

            if self.test_path:
                print("FOUND TEST PATH, committing.")
                speeds = map(lambda x: round(x.speed,2), self.test_path.states)
                angles = map(lambda x: round(x.steering_angle,2), self.test_path.states)
                print("speeds:", speeds)
                print("angles:", angles)
                print(self.test_path.states[-1])

        if self.test_path:
            # visualize paths if necessary
            if self.viz.should_visualize("path_search.test_goal"):
                self.viz.publish_test_goal(self.path_planner.goal_state, ColorRGBA(1,1,1,1))
            if self.viz.should_visualize("path_search.best_path"):
                self.viz.publish_best_path(self.test_path)
            if self.viz.should_visualize("path_search.complete_paths"):
                self.viz.publish_complete_path(self.path_planner.complete_paths())
            if self.viz.should_visualize("path_search.viable_paths"):
                self.viz.publish_viable_paths(self.path_planner.tree_root)
                # self.viz.publish_viable_accel_paths(self.path_planner.tree_root)

    def optimize_time_limit(self):

        self.execution_time_estimate = 1.0 / self.control_monitor.fps()
        # this function will decrease the search time limit if the planning is taking too long
        # otherwise it decrease 
        # place reasonable bounds on the time to prevent crazy results
        # max_time = 1.2/float(param("planning_freq"))
        # min_time = 1.0/float(param("planning_freq"))

        # percentage_slower_than_goal = (float(param("planning_freq")) - self.planning_fps())/float(param("planning_freq")) 
        # self.search_time_limit = np.clip(self.search_time_limit * (1.0 - percentage_slower_than_goal), min_time, max_time)

        # update the expected number of steps per plan interval so that the emergency planners
        # can use the coorect number of control steps
        # self.control_steps_per_plan_interval = int(math.ceil(float(param("execution_freq")) * self.search_time_limit))

    def integrate_forward(self, time_delta):
        # TODO fix this to actually integrate forward, given the controller fps 
        
        start_state = State(x=0, y=0, theta=0, steering_angle=self.state_history[-1].steering_angle, speed=max(0, self.state_history[-1].speed))
        # start_state = DYNAMICS.propagate_time(start_state, time_delta)
        # print(start_state.x, start_state.y)

        return start_state

    def _control_loop(self):
        while True:
            if self.new_data:
                self.computing_control = True
                
                self.scan_time = self.scanner_data.header.stamp.to_sec()

                start_time = rospy.get_rostime().to_sec()
                # update data sources
                self.obstacles.scan_callback(self.scanner_data)
                obstacle_time = rospy.get_rostime().to_sec()
                self.goals.scan_callback(self.scanner_data)
                goal_time = rospy.get_rostime().to_sec()
                # compute control actions
                self.compute_control()
                control_time = rospy.get_rostime().to_sec()

                if self.control_monitor.index % 10 == 1:
                    print()
                    print ("average planning fps: ", self.control_monitor.fps())
                    self.optimize_time_limit()
                    print("last cycle time: total:", round(control_time - start_time,4), 's')
                    print("   - obstacles :", round(obstacle_time - start_time,4), 's')
                    print("   - goals     :", round(goal_time-obstacle_time,4), 's')
                    print("   - control   :", round(control_time-goal_time,4), 's')


                # house keeping
                self.control_monitor.step()
                self.new_data = False
                self.computing_control = False
            else:
                time.sleep(0.001)

    def scan_callback(self, data):
        if self.computing_control:
            return

        self.new_data = True
        self.scanner_data = data
        self.since_scan_recieved = []

    def speed_bounds(self):
        # returns the min/max speed that the car can be set to when called
        # find how much acceleration we can apply to the car considering the rate of control
        fps = self.control_monitor.fps()

        max_decel = param("dynamics.max_linear_decel") / fps
        max_accel = param("dynamics.max_linear_accel") / fps

        max_speed = min(param("dynamics.max_speed"), max(0.0, self.state_history[-1].speed)+max_accel)
        min_speed = max(param("planner.backup_speed"), self.state_history[-1].speed+max_decel)

        return (min_speed, max_speed)

    def pure_pursuit_control(self, circle_path, spline_path=None):
        if circle_path == None:
            return None

        deflections = map(lambda x: abs(x.deflection), circle_path.states)
        max_deflection = max(deflections)

        start_state = circle_path.states[0]

        #TODO: modify this depending on path curvature
        L = param("space_explorer.max_pursuit_radius")*np.cos(max_deflection)
        L = max(L, param("space_explorer.min_pursuit_radius"))
        # print(L, max_deflection, np.cos(max_deflection))
        pursuit_circle = Circle(radius=L, x=start_state.x, y=start_state.y, deflection=0)

        self.lookahead_circle = pursuit_circle

        # if not spline_path == None:
        #     # print("test")
        #     intersection = spline_circle_intersection(pursuit_circle, spline_path)
        # else:
        # if intersection == None:
            # find intersection between the pure pursuit circle and the path
        intersection = None
        for i in xrange(1,len(circle_path.states)):
            intersection = circle_segment_intersection(pursuit_circle, circle_path.states[i-1], circle_path.states[i])
            if not intersection == None:
                break

        if intersection == None:
            return None

        np_intersection = np.array([intersection.x, intersection.y])
        np_start_state = np.array([start_state.x, start_state.y])

        local_intersection = np_intersection - np_start_state

        # find control required to reach the intersection point
        if abs(local_intersection[1]) < param("epsilon"):
            arc_radius = np.inf
        else:
            arc_radius = L * L / (2.0*local_intersection[1])

        arc_radius = max(param("dynamics.r_min"), arc_radius)

        bounds = self.speed_bounds()

        # TODO use the dynamical model to find an optimal steering angle
        speed = DYNAMICS.max_speed_at_turning_radius(abs(arc_radius))
        speed = np.clip(speed, max(0.0, bounds[0]), bounds[1])

        # steering = DYNAMICS.steering_angle(target_arc_radius=arc_radius, target_speed=speed)
        steering_angle = np.arctan2(param("dynamics.wheelbase"), arc_radius) * np.sign(local_intersection[1])

        # print(round(steering_angle,2), round(speed,2))

        return State(x=0,y=0,theta=0,steering_angle=steering_angle,speed=speed)

    @profile(sort="cumtime")
    def compute_control(self, event=None):
        if not self.obstacles.first_laser_recieved:
            print("Waiting for laser data...")
            return False

        # if the test is set to occur, this should pass control to that function by returning
        if param("planner.test.enabled"):
            self.test_control()
            return 

        if self.goals.next_goal() == None:
            # stop if no goal point
            next_path = self.make_stop_path(steering_angle=self.state_history[-1].steering_angle, num_segments=1)
            self.execute_state(next_path.states[0])
            return

        def parameterize_path(circle_path):
            # create a parametric curve approximating the control points
            if circle_path == None:
                return None
            # finds the cubic bezier spline described by the circle path
            n = 3 # order of the curve
            control_points = map(lambda x: (x.x, x.y), reversed(circle_path.states))
            len_orig = len(control_points)

            if param("planner.prune_distance") > 0:
                
                last = circle_path.states[-1]
                downsampled_control = []
                for i in circle_path.states:
                    if euclidean_distance(last, i) > param("planner.prune_distance") \
                    or abs(i.deflection) > param("planner.max_prune_deflection"):
                        downsampled_control.append((i.x, i.y))
                        last = i
                control_points = downsampled_control

            return C_factory(control_points, n)

        #TODO use FPS instead
        dt = rospy.get_rostime().to_sec() - self.scan_time
        start_state = self.integrate_forward(dt)
        # print(start_state.x, start_state.y)
        # start_state = State(x=0, y=0, theta=0, steering_angle=self.state_history[-1].steering_angle, speed=max(0, self.state_history[-1].speed))
        self.space_explorer.reset(start_state=start_state)

        t = self.space_explorer.search(time_limit=param("space_explorer.time_limit"))

        circle_path = self.space_explorer.best()
        spline_path = parameterize_path(circle_path)
        next_control_state = self.pure_pursuit_control(circle_path, spline_path)

        if self.viz.should_visualize("path_search.curve_path") and not spline_path == None:
            self.viz.publish_path_curve(spline_path)

        if self.viz.should_visualize("space_explorer.explored"):
            self.viz.publish_exploration_circles(self.space_explorer.tree_root)

        if self.viz.should_visualize("space_explorer.path") and self.space_explorer.best():
            self.viz.publish_path_circles(circle_path)

        if self.viz.should_visualize("space_explorer.center_path") and self.space_explorer.best():
            self.viz.publish_path_line(circle_path)

        if self.viz.should_visualize("path_search.lookahead_circle") and self.lookahead_circle:
            self.viz.publish_lookahead_circle(self.lookahead_circle)

        if next_control_state == None:
            # commit emergency plan
            print("No path found with", self.space_explorer.step_count, "extensions")
            self.execute_stuck_control()
        else:
            # begin executing the new path. if no path is found, the path will be set to 
            # self.commit_path(best_path) 
            self.execute_state(next_control_state)

    def execute_stuck_control(self):
        if (self.state_history[-1].speed > 0):
            next_path = self.make_stop_path(steering_angle=self.state_history[-1].steering_angle, num_segments=1)
        else:
            next_path = self.make_back_up_path(num_segments=1)

        self.execute_state(next_path.states[0])

    def make_stop_path(self, steering_angle=0, num_segments=None):
        if num_segments == None:
            num_segments = self.control_steps_per_plan_interval+10

        fps = self.control_monitor.fps()

        max_decel = param("dynamics.max_linear_decel") / fps
        last_state = self.state_history[-1]
        next_steering = steering_angle
        stop_path = []
        for i in xrange(num_segments):
            next_speed = max(0.0, last_state.speed + max_decel)
            if next_speed == 0.0:
                next_steering = 0.0
            next_state = State(x=last_state.x, y=last_state.y, theta=last_state.y, \
                speed=next_speed, steering_angle=next_steering)
            next_state = DYNAMICS.propagate(next_state)
            stop_path.append(next_state)
            last_state = next_state

        return Path(states=stop_path)

    def make_back_up_path(self, num_segments=None):
        """ Computes a path which backs the robot up for one planner timestep
        """
        if num_segments == None:
            num_segments = self.control_steps_per_plan_interval+10

        last_state = self.state_history[-1]
        backup_path = []
        for i in xrange(num_segments):
            next_state = State(x=last_state.x, y=last_state.y, theta=last_state.y, \
                speed=param("planner.backup_speed"), steering_angle=0)
            next_state = DYNAMICS.propagate(next_state)
            backup_path.append(next_state)
            last_state = next_state

        return Path(states=backup_path)

    def execute_state(self, state):
        if not self.is_enabled():
            return

        if self.execution_monitor.index == 1:
            print ("execution fps: ", self.execution_monitor.fps())

        # apply the given path to the car, continue it until told otherwise
        self.state_history.append(state)
        self.since_scan_recieved.append(state)

        # send the message to the car
        self.direct_set(speed=state.speed, steering_angle=state.steering_angle)

        if self.viz.should_visualize("path_search.speed"):
            self.viz.publish("path_search.speed", state.speed)
        if self.viz.should_visualize("path_search.speed"):
            self.viz.publish("path_search.speed", state.speed)

        self.execution_monitor.step()

    # callback for when the car is disabled
    def disabled(self):
        self.direct_set(speed=0, steering_angle=0)
        self.state_history.append(State(x=0, y=0, theta=0, steering_angle=0, speed=0))

    def on_shutdown(self):
        """Stop the car."""
        self.direct_set(speed=0, steering_angle=0)
        self.state_history.append(State(x=0, y=0, theta=0, steering_angle=0, speed=0))

    def loop(self):
        # pass
        self.obstacles.viz()
        self.goals.viz()

def make_flamegraph(filter=None):
    import flamegraph
    perf_log_path = os.path.join(os.path.dirname(__file__), "../tmp/perf.log")
    flamegraph.start_profile_thread(fd=open(perf_log_path, "w"),
                                    filter=r"compute_control",
                                    interval=0.001)

if __name__ == '__main__':
    # a=Point(x=0,y=0)
    # b=Point(x=2,y=2)
    # c = Circle(x=0,y=0,radius=0.5,deflection=0)
    # print("intersects", circle_segment_intersection(c, a,b))
    # print(dynamics("steering_prediction"))

    # make_flamegraph(r"compute_control")

    try:
        ChallengeController()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()

