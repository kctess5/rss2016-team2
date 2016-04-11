"""
Ackermann Path Generation Tools

Contains functions to make it easy to plan paths of an Ackermann-steering car.
Ackermann-steering cars travel in a circle when set to a constant steering angle.
"""

import numpy as np
import collections

Path = collections.namedtuple("Path", ["steering_angle", "waypoints", "speed"])
# steering_angle is the initial steering angle of the path.
# waypoints is 2d numpy array of shape (N, 2).
# path.waypoints[n,:] is [x, y, heading]
# path.waypoints[0] is always at the start point of the path.

DEFAULT_WHEEL_BASE = 0.325
DEFAULT_SPEED = 1.


def turning_radius(wheel_base, steering_angle):
    """Get the turning radius of the car at a certain steering angle.
    Args:
        wheel_base: Distance between front and back axel in meters.
        steering_angle: Steering angle in radians.
                        Should probably be limited to [-pi, pi].
    Returns:
        The turning radius in meters. Return inf if steering_angle is 0.
    """
    if steering_angle == 0.:
        return float("inf")
    return abs(wheel_base / np.tan(steering_angle))


def ack_step_origin(wheel_base, steering_angle, travel_distance):
    """Move a car starting at the origin along its path.
    Coordinate System:
        +X is "to the right" on paper and "forward" for the car.
        +Y is "up" on paper and "left turn" for the car.
        Angles proceed CCW.
    Args:
        wheel_base: Distance between front and back axel in meters.
        steering_angle: Steering angle in radians. Positive means left.
                        Should probably be limited to [-pi, pi].
        travel_distance: Distance the car should travel in meters.
    Returns:
        An np array of [x, y, heading] (shape=(3,)).
    """
    if steering_angle == 0.:
        x, y, heading = travel_distance, 0., 0.
        return (x, y, heading)
    radius = turning_radius(wheel_base, steering_angle)
    # Angle of the turning circle spanned by the arc that the car takes.
    travel_angle = travel_distance / radius
    # circumference = 2 * np.pi * radius
    # travel_angle2 = (travel_distance / circumference) * 2 * np.pi
    # assert np.isclose(travel_angle, travel_angle2), [travel_angle, travel_angle2]
    x = radius * np.sin(travel_angle)
    y = radius * (1 - np.cos(travel_angle))
    heading = travel_angle
    if steering_angle < 0:
        y *= -1
        heading *= -1
    return np.array([x, y, heading])
    

def ack_step(wheel_base, steering_angle, travel_distance, start_x, start_y, start_heading):
    """Move a car starting anywhere along its path.
    Works by calling ack_step_origin and then transforming to the start.
    Coordinate System: Same as ack_step_origin.
    Args:
        wheel_base: Distance between front and back axel in meters.
        steering_angle: Steering angle in radians.
                        Should probably be limited to [-pi, pi].
        travel_distance: Distance the car should travel in meters.
        startx, starty, startheading: Starting pose of the car.
    Returns: np array of [x, y, heading].
    """
    x_o, y_o, heading_o = ack_step_origin(wheel_base, steering_angle, travel_distance)
    # Rotate and then translate.
    x_1, y_1 = rotate2d(x_o, y_o, start_heading)
    x, y, heading = x_1 + start_x, y_1 + start_y, heading_o + start_heading
    return np.array([x, y, heading])


def path_constant_curve(wheel_base, steering_angle, travel_distance, npoints, start_x, start_y, start_heading):
    """Create a constant curvature path.
    This is done iteratively as an example.
    Args:
        npoints: Number of points in path. Must be > 2.
    npoints: How many points the result should contain.
    Returns: A Path.
np array array of waypoints  (shape=(N,3)).
             [[x1, y1, heading1], [x2, y2, heading2], ...]
    """
    assert(npoints > 2)
    travel_step = travel_distance / float(npoints-1)
    waypoints = []
    x, y, heading = start_x, start_y, start_heading
    waypoints.append((x, y, heading))
    for _ in xrange(npoints-1):
        x, y, heading = ack_step(wheel_base, steering_angle, travel_step, x, y, heading)
        waypoints.append((x, y, heading))
    return Path(steering_angle=steering_angle,
                waypoints=np.array(waypoints),
                speed=DEFAULT_SPEED)


def paths_fan(wheel_base, steering_angle_min, steering_angle_max,
              npaths, travel_distance, npoints_perpath,
              start_x, start_y, start_heading):
    """Create a bunch of constant curve paths.
    Returns: An array of np arrays of waypoints.
    """
    paths = []
    for steering_angle in np.linspace(steering_angle_min, steering_angle_max, num=npaths):
        paths.append(path_constant_curve(wheel_base,
                                        steering_angle, travel_distance, npoints_perpath,
                                        start_x, start_y, start_heading))
    return paths


def paths_forking(wheel_base, steering_angle_min, steering_angle_max,
              npaths, nfork, step_distance, fork_distance, travel_distance,
              start_x, start_y, start_heading):
    """Create a tree of paths that fork.
    Note that this blows up in runtime exponentially. Be careful with the parameters.
    Args:
        npaths: Number of paths to start with.
        nfork: Number of paths stemming from each fork.
        step_distance: Distance between each point.
        fork_distance: Distance between forks in meters.
        travel_distance: Length of each path in meters.
    Returns: An array of np arrays of waypoints.
    TODO update to return a Path
    """
    assert(step_distance > 0)
    assert(travel_distance > 0)
    assert(fork_distance > 0)
    State = collections.namedtuple("State",
        ["initial_steering_angle", "steering_angle", "waypoints", "length", "last_fork"])
    # State.waypoints is a list of waypoints.
    # A waypoint is [x, y, heading].
    # last_fork is the length at which the last fork was.

    # States which need to grow.
    states = []
    # States which are done.
    states_done = []
    
    # Start the first paths.
    for steering_angle in np.linspace(steering_angle_min, steering_angle_max, num=npaths):
        waypoints = np.array([[start_x, start_y, start_heading]])
        states.append(State(steering_angle, steering_angle, waypoints, length=0, last_fork=0))

    # Loop until all states have moved to states_done.
    while len(states):
        last_states = states
        states = []
        for state in last_states:
            done_now = state.length >= travel_distance
            fork_now = state.length - state.last_fork >= fork_distance
            if done_now:
                # Move the state to done.
                states_done.append(state)
            elif fork_now:
                # Fork into new paths with different steering angles.
                for steering_angle in np.linspace(steering_angle_min, steering_angle_max, num=nfork):
                    states.append(state._replace(
                            steering_angle=steering_angle,
                            last_fork=state.length))
            else:
                # Step the path by step_distance.
                x, y, heading = state.waypoints[-1]
                x, y, heading = ack_step(wheel_base, state.steering_angle, step_distance, x, y, heading)
                waypoints_new = np.append(state.waypoints, [[x, y, heading]], axis=0)
                state_new = state._replace(
                    waypoints=waypoints_new,
                    length=state.length + step_distance)
                states.append(state_new)
                
    return [Path(steering_angle=state.initial_steering_angle,
                waypoints=np.array(state.waypoints),
                speed=DEFAULT_SPEED)
            for state in states_done]


def rotate2d(x, y, angle):
    """Rotate a 2d coordinate around the origin.
    Returns: np array of [x', y']
    """
    return np.array([x * np.cos(angle) - y * np.sin(angle),
                     y * np.cos(angle) + x * np.sin(angle)])
