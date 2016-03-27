#!/usr/bin/env python
"""
Localizer node finds the robot's location in a known map given laser input.

Reads the map from static_map on startup.

Publishes:
    ~particles (PoseArray)   All active particles.
    ~guess     (PoseStamped) Best guess pose.
"""

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, Header
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, Quaternion
from cone_follower.msg import PolarPoints, PolarPoint
from rospy.numpy_msg import numpy_msg
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
from scipy import signal
import tf.transformations
import math
import numpy as np
import random
import collections

import matplotlib.pyplot as plt

def log_level(str_level):
    if str_level.upper() == "DEBUG":
        return rospy.DEBUG
    elif str_level.upper() == "INFO":
        return rospy.INFO
    elif str_level.upper() == "WARN":
        return rospy.WARN


'''
TODOs:
- publish necessary transforms based off the best particle
    map to odom
    odom to base_link is published by the odometer
        when we publish map to odom we should first subtract the odom to base link transform
        so that the odometer transform gives us the correct car position between localization
- publish synthetic laser scan data for visualization purposes
    publish in frame of base_link
- unit test:
    calc_range
- end to end test:
    given some simple map and known start state, does the algorithm converge to the correct solution within x timesteps
- Generate test data with a known start state
- make launch files that makes initialization not a total bitch
'''

"""
Localizes Stuff.
"""

class Localizer(object):
    def __init__(self):
        ll = rospy.get_param('log_level')
        rospy.init_node('localizer', anonymous=True, log_level=log_level(ll))
        rospy.loginfo("Initializing Monte Carlo Localization particle filter")

        # accumulated odometry delta - reset whenever the particle filter runs
        self.clear_odometry() # Delta(0,0,0)
        self.last_pose = None
        self.last_scan = None
        self.seq = 0

        # container for the persistent particles
        self.particles = []
        self.particle_weights = []

        # load all configuration variables
        self.configure()

        # create necessary ros channels

        LASER_SCAN_TOPIC = "/scan"
        self.sub = rospy.Subscriber(LASER_SCAN_TOPIC, numpy_msg(LaserScan), self.scan_callback)
        self.odom = rospy.Subscriber("/vesc/odom", Odometry, self.odometry_callback)
        self.pub_particles = rospy.Publisher('~particles', PoseArray, queue_size=1)
        self.pub_guess = rospy.Publisher('~guess', PoseStamped, queue_size=1)
        self.pub_tf = tf.TransformBroadcaster()

        # 2D numpy array of occupancy floats in [0,1].
        rospy.logdebug("Fetching map")
        self.omap, self.map_info = self.get_omap()
        rospy.logdebug("Got map")

        rospy.Timer(rospy.Duration(1.0 / self.LOCALIZATION_FREQUENCY), self.timer_callback)

    def configure(self):
        # the amount of noise to add for each component of the particle state
        # increasing this variable will make the particles diverge faster

        self.RANDOMNESS = Delta(0.1, 0.1, 0.05)
        self.NUM_PARTICLES = 10
        # number of times per second to attempt localization
        self.LOCALIZATION_FREQUENCY = 2.0
        # TODO - better initial pose management
        self.INITIAL_POSE = Particle(0,0,0)
        self.VISUALIZE = False

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
        heading_d = quaternion_to_angle(orientation) - quaternion_to_angle(self.last_pose.orientation)

        # store deltas
        aod = self.accumulated_odometry_delta
        self.accumulated_odometry_delta = Delta(x_d + aod.x, y_d + aod.y, heading_d + aod.heading)

        # store this pose message for future use
        self.last_pose = data.pose.pose

    def clear_odometry(self):
        rospy.logdebug("Clearing accumulated odometry")
        self.accumulated_odometry_delta = Delta(0,0,0)

    def get_omap(self):
        """Get the map from the map service and return it as a numpy array.
        This blocks until the map service is available.
        The _received_ map has cells which are probabilities in [0,100] with unknown as -1.
        The output map is a 2d numpy array of floats where:
        - [0, 1]: Probability of cell being filled.
        - np.nan: Unknown cell.
        """
        map_service_name = rospy.get_param("~static_map", "static_map")
        rospy.wait_for_service(map_service_name)
        map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map

        width, height = map_msg.info.width, map_msg.info.height

        # print(dir(map_msg.data))
        array_255 = np.array(map_msg.data).reshape((height, width))

        if self.VISUALIZE:
            imgplot = plt.imshow(array_255)
            plt.show()

        array_float = array_255.astype(np.float)
        # Set unknown cells (-1) to nan.
        array_float[array_float < 0] = np.nan
        # Divide [0,100] into [0,1]
        array_float /= 100.
        return array_float, map_msg.info

    def motion_update(self, delta, particle):
        x, y, heading = particle

        # Add delta
        # YO comment this out to disable odometry updating.
        x += delta.x
        y += delta.y
        heading += delta.heading

        # Add noise
        # YO comment this out to disable noise.
        x += self.RANDOMNESS.x * np.random.normal()
        y += self.RANDOMNESS.y * np.random.normal()
        heading += self.RANDOMNESS.heading * np.random.normal()

        return Particle(x, y, heading)

    def sensor_update(self, omap, scan_data, particle, angle_step=10):
        """Calculate weights for particles given a map and sensor data.
        Basically the likelihood of the scan_data at the location.
        Args:
            omap: np array of occupancy
            scan_data: LaserScan message
            particles: np array of particles (positions)
            angle_step: this determines how many laser scan points are generated and tested.
                        set to 1 to test every angle, 2 to test every other, and so on.
        Returns:
            Weight for the particle.
        """

        # compute the set of angles that should be checked
        num_samples = math.ceil((scan_data.angle_max - scan_data.angle_min) / (scan_data.angle_increment * angle_step))
        laser_angles = np.linspace(scan_data.angle_min, scan_data.angle_max, num_samples) + particle.heading

        # print(laser_angles)

        # compute expected ranges from the given particle's position and orientation
        expected_ranges = map(lambda angle: self.calc_range(omap, particle.x, particle.y, angle, scan_data.range_max), laser_angles)

        # sample the corresponding ground truth ranges from the laser scan measurements
        ground_ranges = scan_data.ranges[0::angle_step]

        # print(len(expected_ranges), len(ground_ranges))

        # compute MSE between computed ranges and ground truth measurements
        return ((expected_ranges - ground_ranges) ** 2).mean()


        # # Angles in the scan relative to the robot.
        # relative_angles = (np.arange(measurement.ranges.shape[0]) * measurement.angle_increment) + measurement.angle_min
        # # Angles in map space.
        # absolute_angles = particle.heading + relative_angles
        # # Ranges observed IRL are in measurement.ranges
        # # Ranges we should've observed if this particle were correct.
        # expected_ranges = [self.calc_range(omap, particle.x, particle.y, a, measurement.range_max)
        #                    for a in measurement.ranges]
        # weight = ((expected_ranges - measurement.ranges) ** 2).mean()
        # return weight

    def calc_range(self, omap, x, y, heading, max_range):
        """
        Calculate the expected laser reading at a given point and angle.
        Do this by walking along the ray until you reach something that the map thinks is filled.

        Copied from https://bitbucket.org/alexbuyval/ardroneum
        """

        robot_x, robot_y, robot_a = x, y, heading

        # Threshold value. Above this probability, the cell is expected filled.
        filled_threshold = 0.5

        # return true if the given index is within the map
        def is_valid(_y, _x):
            return (0 <= _y < omap.shape[0] and
                    0 <= _x < omap.shape[1])

        # given the robot's pose in the 'map' frame, compute the corresponding index in
        # the occupancy grid map
        def map_to_grid(map_x, map_y):
            grid_x = int((map_x - self.map_info.origin.position.x) / self.map_info.resolution)
            grid_y = int((map_y - self.map_info.origin.position.y) / self.map_info.resolution)

            return grid_x, grid_y

        x0, y0 = map_to_grid(robot_x, robot_y)
        x1, y1 = map_to_grid(robot_x + max_range*math.cos(robot_a),
                             robot_y + max_range*math.sin(robot_a))

        # compute the real world distance given a hit point which is a map grid index
        def _calc_range(_x, _y):
            xd = (_x - x0)
            yd = (_y - y0)
            d = math.sqrt(xd*xd + yd*yd)
            d_world = d * self.map_info.resolution
            return d_world

        # x0, y0 = robot_x, robot_y
        # x1, y1 = robot_x + max_range*math.cos(robot_a), robot_y + max_range*math.sin(robot_a)

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
                    if omap[y][x] > filled_threshold:
                        return _calc_range(x, y)
                        # return (math.sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) / scale)
            else:
                if is_valid(x, y):
                    if omap[x][y] > filled_threshold:
                        return _calc_range(x, y)
                        # return (math.sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) / scale)

        return max_range

    def init_particle(self):
        rospy.logdebug("Initializing particle at: (%.2f, %.2f, %.2f)", \
            self.INITIAL_POSE.x, self.INITIAL_POSE.y, self.INITIAL_POSE.heading)

        self.particles.append(self.INITIAL_POSE)
        self.particle_weights.append(0.)

    def timer_callback(self, event):
        rospy.logdebug("Stepping particle filter")
        # initialize particles if necessary
        while len(self.particles) < self.NUM_PARTICLES:
            self.init_particle()

        # don't start until scanner data is available
        if self.last_scan == None:
            return

        # update particles and weights
        self.particles, self.particle_weights = self.MCL(
            self.omap, self.particles, self.accumulated_odometry_delta, self.last_scan)

        self.publish_particles(self.particles, self.particle_weights)
        self.publish_tf(self.particles, self.particle_weights)

    def MCL(self, omap, previous_particles, odometry_delta, sensors):
        """Run one step of Monte Carlo localization."""
        rospy.logdebug("Attempting Monte Carlo Localization")

        # update particle positions based on odometry readings
        particles = \
            map(lambda old_particle: self.motion_update(odometry_delta, old_particle), \
                previous_particles)

        # reset accumulated odometry
        self.clear_odometry()
        # print()
        # print(particles)

        # update particle weights according to probability of recording the given sensor readings
        particle_weights = \
            map(lambda old_particle: self.sensor_update(omap, sensors, old_particle), previous_particles)


        # print(particle_weights)

        # compute sum of all particle weights
        particle_mass = sum(particle_weights)

        # reweight particles - normalization of probability
        particle_weights = map(lambda x: float(x) / float(particle_mass), particle_weights)

        # print ("particle weights:", particle_weights)

        # fill new_particles by sampling from the reweighted particle array with replacement
        new_particles_indices = np.random.choice(len(particles), len(particles), True, particle_weights)
        new_particles = map(lambda idx: particles[idx], new_particles_indices)
        new_weights   = map(lambda idx: particle_weights[idx], new_particles_indices)
        # print(new_particles)

        return new_particles, new_weights

    def publish_particles(self, particles, particle_weights):
        header = Header()
        header.seq = self.seq
        self.seq += 1
        header.stamp = rospy.Time.now()
        header.frame_id = "map"

        pa = PoseArray()
        pa.header = header
        pa.poses = map(particle_to_pose, particles)
        self.pub_particles.publish(pa)

        bestParticle = particles[np.argmax(particle_weights)]
        pb = PoseStamped()
        pb.header = header
        pb.pose = particle_to_pose(bestParticle)
        self.pub_guess.publish(pb)

    def publish_tf(self, particles, particle_weights):
        """Publish a tf from map to odom.
        The tf is such that base_link appears in the same place as the best particle."""
        # TODO not done.
        bestParticle = particles[np.argmax(particle_weights)]

        self.pub_tf.sendTransform(
            translation=(0, 0, 0),
            rotation=tf.transformations.quaternion_from_euler(0, 0, 0),
            time=rospy.Time.now(),
            child="odom",
            parent="map")

def particle_to_pose(particle):
    pose = Pose()
    pose.position.x = particle.x
    pose.position.y = particle.y
    pose.orientation = angle_to_quaternion(particle.heading)
    return pose

def angle_to_quaternion(angle):
    """Convert an angle in radians into a quaternion _message_."""
    return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))

def quaternion_to_angle(q):
    """Convert a quaternion _message_ into an angle in radians.
    The angle represents the yaw.
    This is not just the z component of the quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    roll, pitch, yaw = tf.transformations.euler_from_quaternion((x, y, z, w))
    return yaw

Particle = collections.namedtuple("Particle", ["x", "y", "heading"])
Delta = collections.namedtuple("Delta", ["x", "y", "heading"])

if __name__ == '__main__':
    try:
        Localizer()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
