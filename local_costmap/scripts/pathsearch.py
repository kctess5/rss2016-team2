"""
NOTE : Deprecated
    - use the classes in race_controller.py

"""


"""
Search for paths.
"""

import collections
import heapq
# https://docs.python.org/2/library/heapq.html
import numpy as np

import pathlib
from pathlib import Path, DEFAULT_SPEED, DEFAULT_WHEEL_BASE

import rospy
def param(name):
    return rospy.get_param("/local_costmap/" + name)


# Intermediate state of a search point.
State = collections.namedtuple("State",
    ["waypoints", "steering_angles", "length"])
# waypoints: List of decision points [[x, y, heading], ...]
# steering_angles: List of steering angles.
#                  Note that len(steering_angles) == len(waypoints)-1
# length: Total length of the path.


class PathSearch(object):
    """An object containing the state of a search.
    (not threadsafe)

    Searches for possible paths for the robot to take.
    The best choice so far can be fetched at any time.
    
    Note to self: Abstraction is the enemy of the simple and of the fast.
    
    Args:
        cull_fn: Function used to abort paths which collide.
                 (x, y, heading) -> bool (whether the pose is OK)
        heuristic_fn: Function used to guide search.
                 State -> numeric (lower means better)
    """
    def __init__(self, cull_fn, heuristic_fn):
        self.cull_fn = cull_fn
        self.heuristic_fn = heuristic_fn
        # TODO make these parameters
        self.steering_angle_min = -param("max_curve")
        self.steering_angle_max = -self.steering_angle_min
        # Number of paths stemming from each fork.
        self.nfork_initial = 15.
        self.nfork = 5.
        # Distance between forks.
        self.fork_step_distance = .7
        # Distance between cull_fn tests. Should be smaller than fork_distance.
        self.cull_step_distance = .1
        # Starting position for the search.
        self.start_x, self.start_y, self.start_heading = 0., 0., 0.

        # Frontier is a priority queue.
        self.frontier = []

        # Add the start nodes to the frontier.
        start_state = State(
            waypoints=[[self.start_x, self.start_y, self.start_heading]],
            steering_angles=[],
            length=0.,
        )
        heapq.heappush(self.frontier, (0, start_state))

    def reset(self):
        """Reset the entire search.
        Keeps the implementation functions attached.
        """
        raise NotImplementedError("PathSearch.reset not implemented.")

    def crunch(self, credits):
        """Work on the search.

        Args:
            credits: How many 'computational credits' to consume.
                     This is a loose analog to time.
        """
        for ci in xrange(credits):
            # print "Search using credit {}/{}".format(ci, credits)
            if len(self.frontier) == 0:
                return
            self._step()

    def _step(self):
        """Step one iteration of the search.
        Requires that self.frontier is not empty.
        """
        # Pop the best path to split.
        parent_score, parent_state = heapq.heappop(self.frontier)

        nfork = self.nfork if len(parent_state.steering_angles) == 0 else self.nfork_initial
        steering_angles = np.linspace(self.steering_angle_min,
                                      self.steering_angle_max,
                                      num=self.nfork)
        for steering_angle in steering_angles:
            x, y, heading = parent_state.waypoints[-1]
            # A list of distances spaced by cull_step in the range [cull_step, fork_step].
            intermediate_distances = (np.arange(
                self.cull_step_distance, self.fork_step_distance, step=self.cull_step_distance)
                .tolist() + [self.fork_step_distance])
            # Intermediate poses to consider for cull_fn.
            intermediates = [pathlib.ack_step(DEFAULT_WHEEL_BASE, steering_angle, d, x, y, heading)
                             for d in intermediate_distances]
            # TODO this conversion is dumb. Why is pathlib emitting short numpy arrays anyway?
            intermediates = [w.tolist() for w in intermediates]
            all_clear = all(self.cull_fn(x, y, heading) for [x, y, heading] in intermediates)
            is_starting_point = len(parent_state.steering_angles) == 0
            # Kill branches which do not pass the culling function.
            # Except always allow the starting point to branch.
            if not is_starting_point and not all_clear:
                # Kill this branch.
                continue

            # x, y, heading = intermediates[-1]
            new_state = parent_state._replace(
                # waypoints=parent_state.waypoints + [[x, y, heading]],
                # waypoints=parent_state.waypoints + intermediates,
                waypoints=parent_state.waypoints + intermediates[-1:],
                steering_angles=parent_state.steering_angles + [steering_angle],
                length=parent_state.length + self.fork_step_distance,
            )
            score = self.heuristic_fn(new_state)
            heapq.heappush(self.frontier, (score, new_state))

    def best_n(self, n):
        """Return up to the best n live paths.
        Returns: Up to n of the best Paths so far considered.
                 Note that this may be the empty list if no paths are live.
        """

    def best(self, n):
        """Get up to the n best paths.
        Args:
            n: The maximum number of paths to return.
               A value of -1 is special and means to return all paths, unsorted.
        Returns: Up to the best n paths Paths so far considered
                 Or an empty list if no paths are live.
        """
        if len(self.frontier) == 0:
            return []

        if n == 1:
            # Get the best state from the priority queue.
            entries = [self.frontier[0]]
        elif n == -1:
            entries = self.frontier
        else:
            entries = heapq.nsmallest(n, self.frontier)

        # The start state has no steering_angles, there's no point returning it.
        states = [state for (score, state) in entries if len(state.steering_angles) > 0]

        paths = [Path(steering_angle=state.steering_angles[0],
                      waypoints=np.array(state.waypoints),
                      speed=DEFAULT_SPEED)
                 for state in states]

        return paths
