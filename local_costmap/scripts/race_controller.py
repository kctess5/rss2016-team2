#!/usr/bin/env python
from __future__ import print_function
import rospy
import threading, time, collections, heapq, itertools, math, sys, recordclass
import numpy as np

# message related imports
from sensor_msgs.msg import LaserScan
from rospy.numpy_msg import numpy_msg

# code from other files in this repo
import navigator
from visualization_driver import VisualizationDriver
from helpers import param, euclidean_distance, FrameBuffer, polar_to_euclid
from pathlib import ack_step
from car_controller.control_module import ControlModule
import whinytimer

# Notes: 
#   - Relies on the following external files:
#       - navigator.py
#       - helpers.py
#       - pathlib.py 
#           - very handy :D
#       - whinytimer.py
#           - death be to ros_time_moved_backwards error
#   - Can be further split into different files if necessary:
#       - DynamicsModel -> dynamics.py
#       - ObstacleMap -> obstacles.py
#       - HeuristicSearch -> pathsearch.py
#       - I think everything else is sufficiently short and important at the moment to stay
#         and I personally think that having most important logic in one file is good when 
#         possible, because code folding is super effective
#   - Info about states/paths:
#       - States contain:
#           x, y, theta - position/orientation local coordinates, based off of predicted motion
#           speed, steering_angle - control commands for the car
#       - I found that speed settings on the car generally correspond to real speed of the car
#         so long as the maximal acceleration bounds are observed, so it does not have to be
#         decoupled from x,y,theta does. Steering angle is not directly related to changes in
#         theta, so the dynamic model is required to estimate where the car will end up after
#         applying a chain of control states
#       - Paths are just a list of states, they are applied consecutively. For now, the first
#         state in the path defines the origin of the local coordinate space for that time step
#         eventually we might want to use a "global" local coorinate space, where rather than
#         resetting the origin at each timestep, move the believed "car position" according to 
#         the full chain of states previously committed. This might make continious tree 
#         construction more straightforward as we would not have to transform each node in the 
#         search tree into the new local coorinate space between each iteration. It would 
#         also be useful for visualizing how well the dynamical open-loop controller predicts motion
#   - The emergency stop control is implemented as follows:
#       - in the case where there are no remaining states to execute in the last committed 
#         best path, the car will reuse it's previous control state while stopping as fast as possible
#       - in the case where there is a path committed, but the path planner is unable to find
#         a new path, it will continue the last committed path while stopping as fast as possible
#       - if there are remaining control states then the path will continue as planned
#   - Visualization
#       - Subjects include:
#           - goals
#               - next goal (1)
#               - corridor detector
#                   - corridor (1)
#                   - walls (2)
#           - obstacles
#               - in-admissible region (3) (NOT IMPLEMENTED)
#           - path search
#               - best path (1)
#               - viable paths (1 or 2)
#               - speed curves (1)
#               - steering curves (1)
#       - Notes:
#           - Each topic does not advertise unless there are subscribers
 
# Todos:
#   - Accumulate the x, y, theta positions in the state history so that the percieved path may be 
#     reconstructed relative to the start state
#       - might also make continuous tree refinement easier, if that is necessary
#   - Finish Dynamical model
#       - still need to include the data I collected, planning on doing this before Sunday
#       - I think that this will naturally improve search a lot, because faster paths will have
#         lower curvature
#   - Better heuristics
#   - Potentially different path cost metrics
#   - Continiously refining a single tree, rather than building a whole new one on every timestep
#   - Decoupling path planning and path execution so that more than one path segment is executed per plan
#   - Vision based goal integration
#   - Cooridor detector should detect dead ends and put a goal point behind the robot if one is found

State = collections.namedtuple("State", ["x", "y", "theta", "steering_angle", "speed"])
Path  = collections.namedtuple("Path", ["states"])
# min and max are both States, with values set to the min/max of each field
StateRange = collections.namedtuple("StateRange", ["min", "max"])
# wraps a state with additional information for search
SearchNode = collections.namedtuple("SearchNode", ["state", "cost", "heuristic", "parent", "tree_node"])
# used for recreating the search tree in visualization
TreeNode = recordclass.recordclass("TreeNode", ["state", "children"])

class DynamicModel(object):
    """ Encapsulates the dynamics of the car """
    def __init__(self):
        self.max_accel = float(param("dynamics.max_linear_accel")) / float(param("planning_freq"))
        self.max_decel = float(param("dynamics.max_linear_decel")) / float(param("planning_freq"))
        self.max_angular_accel = float(param("dynamics.max_angular_accel")) / float(param("planning_freq"))

    def path_statistics(self, path):
        """ Provides an estimate of:
                - travel time
                - end state
                - distance traveled
                - max speed
                - min speed
                - average speed
            Note: this is currently not used anywhere, but feel free to implement it if you want it
        """
        pass

    def reachable_neighbors(self, state):
        """ Provides state ranges that the car could feasibly transition to given an input state.
                heading: based off of max angular acceleration model
                speed: based off of max linear acceleration model
                x, y: CURRENTLY WRONG based off of speed, controller refresh rate, and max curvature bounds
                TODO: x,y are not correctly calculated, fix if needed
        """
        range_min = State(steering_angle=max(state.steering_angle-self.max_angular_accel, -1*param("dynamics.max_deflection")), \
                              speed=max(state.speed+self.max_decel, 0), x=0, y=0, theta=0)

        range_max = State(steering_angle=min(state.steering_angle+self.max_angular_accel, param("dynamics.max_deflection")), \
                              speed=min(state.speed+self.max_accel, param("dynamics.max_speed")), x=0, y=0, theta=0)

        return StateRange(min=range_min, max=range_max)

    def propagate(self, state, t=1.0):
        """ Gives an estimate for the final pose after assuming a given state after a given time. t=1 is time time of the next control timestep
        """

        # need to find:
        #   - translation
        #   - rotation
        # dynamical model gives
        #   - effective path arc radius
        #   - speed along the path
        # pathlib can probably be easily be modified to use a given arc radius and speed

        # for now, use ackermann model to propagate the model. TODO: use collected dynamic model
        propagated = ack_step(param("wheel_base"), state.steering_angle, t * state.speed / param("planning_freq"),
            state.x, state.y, state.theta)

        return State(x=propagated[0], y=propagated[1], theta=propagated[2], \
            steering_angle=state.steering_angle, speed=state.speed)

DYNAMICS = DynamicModel()

class ObjectManager(object):
    """ Manages all inbound ROS communications
            - this prevents unnecessary data flow
    """
    def __init__(self):
        self.scan_subscriber = rospy.Subscriber(param("runtime_specific.scan_topic"), \
            numpy_msg(LaserScan), self.scan_callback, queue_size=1)
        self.scan_callbacks = []

    def scan_callback(self, data):
        for cb in self.scan_callbacks:
            cb(data)

    def register_scan_callback(self, callback):
        self.scan_callbacks.append(callback)

class ObstacleMap(object):
    """ Manages obstacle information from the scanner to provide admissibility information """
    def __init__(self):
        self.buffer = FrameBuffer(param("obstacle_map.discretization"), \
            (param("obstacle_map.xmin"),param("obstacle_map.xmax")), \
            (param("obstacle_map.ymin"),param("obstacle_map.ymax")))
        self.first_laser_recieved = False
        # self.lock = threading.Lock() # used to serialize accesses to the main buffer

    def filter_lasers(self, angles, ranges):
        # remove the data on the edges, since they are noisy
        l = round(angles.shape[0] / 10)
        return (angles[l:-l], ranges[l:-l])

    def scan_callback(self, data):
        if not self.first_laser_recieved:
            print(("first laser received: "
                    "angle_min: %f angle_max %f angle_incr: %f "
                    "ranges_len: %d range_min: %.2f range_max_ %2.2f") % (
                    data.angle_min, data.angle_max, data.angle_increment, 
                        data.ranges.shape[0], data.range_min, data.range_max))
            self.laser_angles = np.linspace(data.angle_min, data.angle_max, math.ceil(\
                (data.angle_max - data.angle_min) / data.angle_increment))
        
        laser_angles, laser_ranges  = self.filter_lasers(self.laser_angles, data.ranges)
        laser_x, laser_y =  polar_to_euclid(laser_angles, laser_ranges)

        # compute the distance transform from the laser scanner data
        # with self.lock:
        self.buffer.clear()
        for x, y, r in zip(laser_x, laser_y, laser_ranges):
            if r < data.range_max-0.1:
                self.buffer.add_sample(x,y)
        self.buffer.dist_transform()
        
        self.first_laser_recieved = True
        self.mark_clean()
         
    def is_admissible(self, state):
        # returns true if the given control state is admissible
        # steps through t values between 0 and 1 while checking that each point is 
        # above the minimum allowed distance
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

class GoalManager(object):
    """ Manages and tracks the car's concept of purpose in life
            - This should wrap the corridor detector and the vision based goal detection
            - Shraman, you should improve this with goal tracking, etc
    """
    def __init__(self, viz):
        self.navigator = navigator.Navigator(viz)

    def next_goal(self):
        """ Return the position of the next goal in local coordinates
                - for now, this directly calls the corridor detector with no smoothing
        """
        gp = self.navigator.goalpoint()
        return State(x=gp[0], y=gp[1], theta=gp[2], steering_angle=None, speed=None)

    def scan_callback(self, data):
        self.navigator.laser_update(data)
        self.navigator.visualize()

class HeuristicSearch(object):
    """ Perform heuristic search on the provided set of cost/admissibility/heuristic/neighbor functions """
    def __init__(self):
        self.reset()

    def reset(self, start_state=State(x=0,y=0,theta=0,speed=0,steering_angle=0)):
        # Frontier is a priority queue.
        self.frontier = []
        self.found_paths = []
        self.goal_state = self.goal()
        self.step_count = 0
        self.tree_root = TreeNode(state=start_state, children=[])

        ss = SearchNode(state=start_state, parent=None, cost=0, tree_node=self.tree_root, \
                    heuristic=self.heuristic(start_state, self.goal_state))

        heapq.heappush(self.frontier, (ss.heuristic, ss))

    def search(self, time_limit):
        start_time = time.time()

        # extend nodes until the time limit is reached
        while time.time() - start_time < time_limit:
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
            # print ("testing neighbor", neighbor_state)
            # prune any path segments that are bound to fail
            if self.is_admissible(neighbor_state):
                # print(neighbor_state.theta)

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



        self.step_count += 1

    def best(self):
        # returns the best path found
        if len(self.frontier) == 0:
            return None
        else:
            def tree_size(node):
                return len(node.children) + sum(map(tree_size, node.children))

            def tree_depth(node):
                if len(node.children) == 0:
                    return 1
                else:
                    return max(map(tree_depth, node.children)) + 1
            
            # print(tree_size(self.tree_root), tree_depth(self.tree_root))
            return self.make_path(self.frontier[0][1]) # CHECK: this might need to be further indexed

    def make_path(self, end_node):
        # a path is a list of control states, in the order of traversal
        # builds it in reverse order by following parent pointers, then reverses result
        path = [end_node.state]
        while end_node.parent:
            path.append(end_node.parent.state)
            end_node = end_node.parent
        # don't execute the start state
        path.pop()
        path.reverse()
        return Path(states=path)

    """ The following functions should be overridden by the parent class
    """
    def cost(self, path):
        # estimate the cost of traversing a given path
        raise NotImplementedError("HeuristicSearch cost not specified")

    def heuristic(self, state):
        # return an estimate for cost to go between the given state and the goal state
        raise NotImplementedError("HeuristicSearch heuristic not specified")

    def goal(self):
        # return the goal state
        raise NotImplementedError("HeuristicSearch goal not specified")

    def goal_met(self):
        # return true if the goal state is met
        raise NotImplementedError("HeuristicSearch goal not specified")

    def is_admissible(self, state):
        # return true if the given state is considered admissible - fulfills hard constraints
        raise NotImplementedError("HeuristicSearch is_admissible not specified")

    def neighbors(self, state):
        # return a discretized set of neighbors of the given state
        raise NotImplementedError("HeuristicSearch is_admissible not specified")
        
class PathPlanner(HeuristicSearch):
    """docstring for PathPlanner"""
    def __init__(self, obstacles, goals):
        self.obstacles = obstacles
        self.goals = goals

        super(PathPlanner, self).__init__()
        
    def cost(self, state):
        # time is our optimization target, so the cost of any given segment is constant
        return 1.0 / float(param("planning_freq"))

    def is_admissible(self, state):
        return self.obstacles.is_admissible(state)

    def heuristic(self, state, goal_state):
        print
        # print((int(state.x), int(state.y), state.theta), (int(goal_state.x), int(goal_state.y), goal_state.theta))
        # print(np.cos(abs(goal_state.theta - state.theta)))
        # return an estimate for cost to go between the given state and the goal state
        # TODO: Dubin's curves, or other better heuristic

        # heuristic is measure of cost to get to goal
        # high deflection -> high cost
        # 0 deflection, minimal cost
        # print(np.cos(abs(state.theta - goal_state.theta)), goal_state.theta, state.theta)
        # t = d / v
        
        xp = [0, 0.7]
        fp = [2.0, 1]

        obstacle_coeff = np.interp(self.obstacles.dist_at(state), xp, fp)

        # approx_d = euclidean_distance(state, goal_state) /  math.pow(np.cos(abs(state.theta - goal_state.theta)), 1)
        # approx_v = state.speed
        # approx_d = euclidean_distance(state, goal_state)

        # return (euclidean_distance(state, goal_state) * obstacle_coeff) / (state.speed+0.0001)
        # bad approximation of the deflection slowdown
        # return euclidean_distance(state, goal_state) /  np.cos(abs(state.theta - goal_state.theta))
        return euclidean_distance(state, goal_state)

    def goal(self):
        # return the next goal state
        return self.goals.next_goal()

    def goal_met(self, state, goal_state):
        return euclidean_distance(state, goal_state) < float(param("planner.goal_distance_threshold"))

    def neighbors(self, state):
        """ find a list of candidate control settings to consider
                - optimal control is assumed to entail setting speed to min or max possible values 
                  at any given timestep, considering that the car should either be quickly accelerating,
                  quickly decelerating, or traveling at the top speed at any given time
                - if the reachable angular control range includes steering_angle=0, then we 
                  need to make sure that 0 is one of the options so that the car doesn't
                  take an unnecessarily curvy path
        """ 
        # print()
        # print("neighbors", state)
        reachable_range = DYNAMICS.reachable_neighbors(state)
        angles = np.linspace(reachable_range.min.steering_angle, reachable_range.max.steering_angle, param("planner.angular_branch_factor"))
        speeds = [reachable_range.min.speed, reachable_range.max.speed]

        # Don't consider paths with zero speed - that's dumb
        if speeds[0] == 0:
            speeds = [speeds[1]]

        # make sure that steering_angle=0 is an option if the range includes it
        if reachable_range.min.steering_angle < 0 and reachable_range.max.steering_angle > 0:
            # print("CONTAINS ZERO")
            # find index of value closest to zero
            idx = min(xrange(len(angles)), key=lambda i: abs(angles[i]))
            # set to zero
            angles[idx] = 0.0

        # make all possible combinations of speeds and steering angles - [angle, speed]
        candidate_controls = list(itertools.product(angles, speeds))
        # convert into states
        candidate_controls = map(lambda x: State(x=state.x, y=state.y, theta=state.theta, \
            steering_angle=x[0], speed=x[1]), candidate_controls)

        # propagate the position and orientation forward one control timestep according to the dynamics model
        return map(DYNAMICS.propagate, candidate_controls)
        
class ChallengeController(ControlModule):
    """ Top level car control for the 6.141 Challenge"""
    def __init__(self):
        # print("test")
        super(ChallengeController, self).__init__("challenge_controller")

        # print("test")

        # initialize the control state management
        self.state_history = [State(x=0, y=0, theta=0, steering_angle=0, speed=0)]
        self.current_path = None
        self.state_index = 0

        # initialize peripheral algorithms
        self.resources = ObjectManager()
        self.viz = VisualizationDriver()

        self.goals = GoalManager(self.viz)
        self.obstacles = ObstacleMap()
        self.path_planner = PathPlanner(self.obstacles, self.goals)

        # hook up necessary data flow
        self.resources.register_scan_callback(self.obstacles.scan_callback)
        self.resources.register_scan_callback(self.goals.scan_callback)

        # whinytimer.WhinyTimer(rospy.Duration(1.0 / float(param("planning_freq"))), self.compute_control)
        rospy.Timer(rospy.Duration(1.0 / float(param("planning_freq"))), self.compute_control)
        rospy.on_shutdown(lambda: self.on_shutdown())

    def next_path_segment(self):
        # return the next state in the currently executing path if possible
        
        # we have a valid path if there is an available path and it has not been completed
        if self.current_path and self.state_index < len(self.current_path.states):
            next_state = self.current_path.states[self.state_index]
            self.state_index += 1
            return next_state
        else:
            # we do not have a currently commited path, apply the last action with damping
            ls = self.state_history[-1]
            # TODO: propagate the state forward according to the dynamic model
            return State(x=ls.x, y=ls.y, theta=ls.theta, \
                speed=ls.speed, steering_angle=ls.steering_angle)

    def stop_path(self, steering_damping=1):
        # continue the next path segment while stopping as fast as possible
        ps = self.next_path_segment()
        next_speed = max(0.0, self.state_history[-1].speed + DYNAMICS.max_decel)
        next_steering = ps.steering_angle * float(param("emergency.stop_path_steering_damp_factor"))

        # stop wiggling the wheels once the car is stopped
        if next_speed == 0.0:
            next_steering = 0.0

        # TODO: propagate the state forward according to the dynamic model
        next_state = State(x=ps.x, y=ps.y, theta=ps.theta, speed=next_speed, steering_angle=next_steering)
        self.execute_state(next_state)

    def continue_path(self):
        # continue the next path segment as planned
        self.execute_state(self.next_path_segment())

    def stuck_control(self):
        # what to do when the car is stuck according to the path planner
        # for now, just stop as fast as possible while continuing path TODO: maybe do something a bit fancier
        self.stop_path()
        
    def compute_control(self, event=None):
        if not self.obstacles.first_laser_recieved:
            print("Waiting for laser data...")
            return False

        start_state = State(x=0, y=0, theta=0, \
            steering_angle=self.state_history[-1].steering_angle, speed=self.state_history[-1].speed)
        self.path_planner.reset(start_state=start_state)
        # search for a viable path, taking at most half the available computation time
        self.path_planner.search(time_limit=0.5/float(param("planning_freq")))
        # choose the best path
        best_path = self.path_planner.best()

        print("Computed control with " + str(self.path_planner.step_count) + " graph extensions")

        if best_path == None:
            # perform default stuck control
            self.stuck_control()
        else:
            self.commit_path(best_path) # begin executing the new path
            # TODO: decouple path execution from path planning so they can run at different hz
            self.continue_path()

            # visualize paths if necessary
            if self.viz.should_visualize("path_search.best_path"):
                self.viz.publish_best_path(best_path)
            if self.viz.should_visualize("path_search.viable_paths"):
                self.viz.publish_viable_paths(self.path_planner.tree_root)

    def commit_path(self, path):
        self.current_path = path
        self.state_index = 0

    def execute_state(self, state):
        # apply the given path to the car, continue it until told otherwise
        self.state_history.append(state)

        # send the message to the car
        # TODO: maybe an asynchronous state commit system will be more flexible
        control_msg = self.make_message("direct_drive")
        control_msg.drive_msg.speed = state.speed
        control_msg.drive_msg.steering_angle = state.steering_angle
        self.control_pub.publish(control_msg)

        if self.viz.should_visualize("path_search.speed"):
            self.viz.publish("path_search.speed", state.speed)
        if self.viz.should_visualize("path_search.speed"):
            self.viz.publish("path_search.speed", state.speed)

    def on_shutdown(self):
        """Stop the car."""
        control_msg = self.make_message("direct_drive")
        control_msg.drive_msg.speed = 0
        control_msg.drive_msg.steering_angle = 0
        self.control_pub.publish(control_msg)

if __name__ == '__main__':
    try:
        ChallengeController()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()

