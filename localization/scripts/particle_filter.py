#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from cone_follower.msg import PolarPoints, PolarPoint
from rospy.numpy_msg import numpy_msg
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
from scipy import signal
import math
import numpy as np
import random
import collections

"""
Localizes Stuff.
"""
class Localizer(object):
    def __init__(self):
        rospy.loginfo("Initializing Monte Carlo Localization particle filter")

        rospy.init_node('localizer', anonymous=True)
        LASER_SCAN_TOPIC = "/scan"
        self.sub = rospy.Subscriber(LASER_SCAN_TOPIC, numpy_msg(LaserScan), self.scan_callback)
        self.odom = rospy.Subscriber("/vesc/odom", Odometry, self.odometry_callback)
        self.pub = rospy.Publisher('/cone/key_points', PolarPoints, queue_size=10)

        # 2D numpy array of occupancy floats in [0,1].
        self.omap = self.get_omap()

        # accumulated odometry delta - reset whenever the particle filter runs
        self.clear_odometry() # Delta(0,0,0)
        self.last_pose = None
        self.last_scan = None

        # container for the persistent particles
        self.particles = []

        # load all configuration variables
        self.configure()

        rospy.Timer(rospy.Duration(1.0 / self.LOCALIZATION_FREQUENCY), self.timer_callback)

    def configure(self):
        # the amount of noise to add for each component of the particle state
        # increasing this variable will make the particles diverge faster
        self.RANDOMNESS = Delta(1,1,0.1)
        self.NUM_PARTICLES = 100
        # number of times per second to attempt localization
        self.LOCALIZATION_FREQUENCY = 15.0
        # TODO - better initial pose management
        self.INITIAL_POSE = Particle(0,0,0)

    def scan_callback(self, data):
        rospy.logdebug("Storing scan data")
        self.last_scan = data

    def odometry_callback(self, data):
        rospy.logdebug("Storing odometry data")
        if self.last_pose == None:
            self.last_pose = data.pose.pose
            return

        pos = data.pose.pose.position
        orientation = data.pose.pose.orientation

        # calculate delta between this and last sensor reading
        x_d = pos.x - self.last_pose.position.x
        y_d = pos.y - self.last_pose.position.y
        heading_d = orientation.z - self.last_pose.orientation.z

        # store deltas
        aod = self.accumulated_odometry_delta
        self.accumulated_odometry_delta = (x_d + aod.x, y_d + aod.y, heading_d + aod.heading)

        # store this pose message for future use
        self.last_pose = data.pose.pose

    def clear_odometry(self):
        rospy.logdebug("Clearing accumulated odometry")
        self.accumulated_odometry_delta = Delta(0,0,0)

    def get_omap(self):
        """Get the map from the map service and return it as a numpy array.
        This blocks until the map service is available.
        """
        map_service_name = rospy.get_param("~static_map", "static_map")
        rospy.wait_for_service(map_service_name)
        map_msg = rospy.ServiceProxy(map_service_name, GetMap)()
        width, height = map_msg.info.width, map_msg.info.height
        array_255 = map_msg.data.reshape((height, width))
        array_float = array_255.astype(float) / 255.
        return array_float

    def motion_update(self, delta, particle):
        return Particle(particle.x + delta.x + self.RANDOMNESS.x * np.random.normal(), \
            particle.y + delta.y + self.RANDOMNESS.y * np.random.normal(), \
            particle.heading + delta.heading + self.RANDOMNESS.heading * np.random.normal())

    def sensor_update(self, omap, measurements, particles):
        """Calculate weights for particles given a map and sensor data.
        Basically the likelihood of the measurement at the location.
        Args:
            omap: np array of occupancy
            measurements: LaserScan message
            particles: np array of particles (positions)
        Returns:
            An array of weights for the particles.
        """
        return [self.sensor_update_individual(omap, sensors, particle) for particle in particles]

    def sensor_update_individual(self, omap, measurement, particle):
        """Whee, scan matching."""
        # Angles in the scan relative to the robot.
        relative_angles = (np.arange(msg.ranges.shape[0]) * msg.angle_increment) + msg.angle_min
        # Angles in map space.
        absolute_angles = particle.heading + angles
        # Ranges observed IRL are in measurement.ranges
        # Ranges we should've observed if this particle were correct.
        expected_ranges = [self.calc_range(omap, particle.x, particle.y, a, max_range)
                           for a in measurement_ranges]
        weight = ((expected_ranges - measurement.ranges) ** 2).mean()
        return weight

    def calc_range(self, omap, x, y, heading, max_range):
        """
        Calculate the expected laser reading at a given point and angle.
        Do this by walking along the ray until you reach something that the map thinks is filled.

        Copied from https://bitbucket.org/alexbuyval/ardroneum
        """
        robot_x, robot_y, robot_a = x, y, heading

        def is_valid(y, x):
            return (0 <= y < omap.shape[0] and
                    0 <= x < omap.shape[1])
        # What is scale?
        scale = 1

        x0, y0 = robot_x, robot_y
        x1, y1 = robot_x + max_range*math.cos(robot_a), robot_y + max_range*math.sin(robot_a)

        if abs(y1-y0) > abs(x1-x0):
            steep = True
        else:
            steep = False

        if steep:
            x0, y0 = y0, x0
            x1, y1 = y1, x1

        deltax = abs(x1-x0)
        deltay = abs(y1-y0)
        error = 0
        deltaerr = deltay

        x = x0
        y = y0

        if x0 < x1:
            xstep = 1
        else:
            xstep = -1
        if y0 < y1:
            ystep = 1
        else:
            ystep = -1

        while x != (x1 + xstep*1):
            x += xstep
            error += deltaerr
            if 2*error >= deltax:
                y += ystep
                error -= deltax

            # TODO: check
            # if steep:
            if not steep:
                if is_valid(y, x):
                    if omap[y][x]:
                        return (math.sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) / scale)
            else:
                if is_valid(x, y):
                    if omap[x][y]:
                        return (math.sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) / scale)

        return max_range

    def init_particle(self):
        rospy.logdebug("Initializing particle at: (%.2f, %.2f, %.2f)", \
            self.INITIAL_POSE.x, self.INITIAL_POSE.y, self.INITIAL_POSE.heading)

        self.particles.append(self.INITIAL_POSE)

    def timer_callback(self, event):
        # initialize particles if necessary
        while len(self.particles) < self.NUM_PARTICLES:
            self.init_particle()

        # don't start until scanner data is available
        if self.last_scan == None:
            return

        self.particles = \
            self.MCL(self.omap, self.particles, self.accumulated_odometry_delta, self.last_scan)

    def MCL(self, omap, previous_particles, odometry_delta, sensors):
        rospy.logdebug("Attempting Monte Carlo Localization")

        # update particle positions based on odometry readings
        particles = \
            map(lambda old_particle: self.motion_update(odometry_delta, old_particle), \
                previous_particles)

        # reset accumulated odometry
        self.clear_odometry()

        # update particle weights according to probability of recording the given sensor readings
        particle_weights = \
            map(lambda old_particle: self.sensor_update(omap, sensors, old_particle), previous_particles)

        # compute sum of all particle weights
        particle_mass = sum(particle_weights)

        # reweight particles - normalization of probability
        particle_weights = map(lambda x: x / particle_mass, particle_weights)

        # fill new_particles by sampling from the reweighted particle array with replacement
        new_particles = np.random.choice(particles, len(particles), True, particle_weights)

        return new_particles

Particle = collections.namedtuple("Particle", ["x", "y", "heading"])
Delta = collections.namedtuple("Delta", ["x", "y", "heading"])

if __name__ == '__main__':
    try:
        Localizer()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
