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
import copy
import time
import fast_utils
import matplotlib.pyplot as plt

def time_millis():
    return time.time()*1000

def log_level(str_level):
    if str_level.upper() == "DEBUG":
        return rospy.DEBUG
    elif str_level.upper() == "INFO":
        return rospy.INFO
    elif str_level.upper() == "WARN":
        return rospy.WARN

class DynamicPlot():

    def initialize(self):
        plt.ion()
        #Set up plot
        self.fig = plt.figure(figsize=plt.figaspect(2.))
        
        self.ax0 = self.fig.add_subplot(2,1,1)

        self.laser_angular, = self.ax0.plot([],[], 'r.')
        self.laser_filtered, = self.ax0.plot([],[], 'r-')
        self.laser_eroded, = self.ax0.plot([],[], 'c-')
        self.laser_maxima, = self.ax0.plot([],[], 'wo')
        self.desired_heading_point, = self.ax0.plot([],[], 'ro')
        
        self.ax0.set_ylim(-1, 15)
        self.ax0.set_xlim(-math.pi, +math.pi)
        self.ax0.invert_xaxis()
        self.ax0.grid()
        
        self.ax1 = self.fig.add_subplot(2,1,2) 
        self.ax1.set_ylim(-1, 15)
        self.ax1.set_xlim(-math.pi, +math.pi)
        self.ax1.invert_xaxis()
        self.ax1.grid()
        self.laser_euclid, = self.ax1.plot([],[], '.')
        self.laser_euclid_eroded, = self.ax1.plot([],[], 'r.')
        self.ax1.plot(0,0, 'ko')
        self.current_heading_angle, = self.ax1.plot([0,0],[0,1], 'k-')
        self.desired_heading_angle, = self.ax1.plot([],[], 'g-')
        self.selected_maneuver, = self.ax1.plot([],[], 'c-')
        #self.costmap = self.ax1.pcolormesh(np.ones((10,10)))
        #
        # self.ax2 = self.fig.add_subplot(1,3,3) 
        
        self.redraw()
        
    def redraw(self):
        #Need both of these in order to rescale
        self.ax0.relim()
        self.ax0.autoscale_view()
        
        self.ax1.relim()
        self.ax1.autoscale_view()
        
        #We need to draw *and* flush
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


'''
TODOs:
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

ANGLE_MSE = 1
DISTANCE_HISTOGRAM = 2
MIXTURE_1 = 3

class Localizer(object):
    def __init__(self, visualize=False, simulated=False):

        self.SHOW_VIS = visualize

        ll = rospy.get_param('log_level')
        rospy.init_node('localizer', anonymous=True, log_level=log_level(ll))
        rospy.loginfo("Initializing Monte Carlo Localization particle filter")

        # Odometry data is saved in last_pose (Most recent) and prev_pose (last time MCL ran)
        self.last_pose = None
        self.prev_pose = None
        self.last_scan = None
        self.seq = 0
        self.expected_scan_seq = 0

        # container for the persistent particles
        self.particles = []
        self.particle_weights = []

        # load all configuration variables
        self.configure()

        self.first_laser_recieved = False
        self.laser_angles = None
        self.expected_angles = None
        self.expected_ranges = None
        self.current_best_err = 10000000

        # create necessary ros channels

        LASER_SCAN_TOPIC = rospy.get_param("~scan", "/scan")
        ODOM_TOPIC = "/vesc/odom"
        if simulated:
            LASER_SCAN_TOPIC = '/racecar/laser/scan'
            ODOM_TOPIC = "/odom"

        self.sub = rospy.Subscriber(LASER_SCAN_TOPIC, numpy_msg(LaserScan), self.scan_callback)
        self.odom = rospy.Subscriber(ODOM_TOPIC, Odometry, self.odometry_callback)
        self.pub_particles = rospy.Publisher('~particles', PoseArray, queue_size=1)
        self.pub_guess = rospy.Publisher('~guess', PoseStamped, queue_size=1)
        self.pub_tf = tf.TransformBroadcaster()
        self.pub_expected_scan = rospy.Publisher('~expected_scan', LaserScan, queue_size=1)

        # 2D numpy array of occupancy floats in [0,1].
        rospy.logdebug("Fetching map")
        self.omap, self.map_info = self.get_omap()
        rospy.logdebug("Got map")

        rospy.Timer(rospy.Duration(1.0 / self.LOCALIZATION_FREQUENCY), self.timer_callback)

        if self.SHOW_VIS:
            self.viz = DynamicPlot()
            self.viz.initialize()
            while not rospy.is_shutdown():
                self.loop()
                rospy.sleep(0.1)

    def configure(self):
        # the amount of noise to add for each component of the particle state
        # increasing this variable will make the particles diverge faster

        # self.RANDOMNESS = Delta(0.25, 0.25, 0.15)
        self.RANDOMNESS = Delta(0.06, 0.06, 0.06)
        self.NUM_PARTICLES = 50
        # number of times per second to attempt localization
        self.LOCALIZATION_FREQUENCY = 15.0
        # TODO - better initial pose management
        self.INITIAL_POSE = Particle(0.0,0,0.0)
        self.VISUALIZE = False
        self.ANGLE_STEP = 8

    def scan_callback(self, data):
        rospy.logdebug("Storing scan data")
        self.last_scan = data

        if self.laser_angles == None:
            self.laser_angles = np.linspace(data.angle_min, data.angle_max, math.ceil((data.angle_max - data.angle_min) / data.angle_increment))
            self.expected_angles = self.laser_angles[0::self.ANGLE_STEP]

    def odometry_callback(self, data):
        rospy.logdebug("Storing odometry data")
        self.last_pose = data.pose.pose
        if self.prev_pose == None:
            self.prev_pose = data.pose.pose

    def clear_odometry(self):
        rospy.logdebug("Clearing accumulated odometry")
        self.prev_pose = self.last_pose
        #self.accumulated_odometry_delta = Delta(0,0,0)

    def get_omap(self):
        """Get the map from the map service and return it as a numpy array.
        This blocks until the map service is available.
        The _received_ map has cells which are probabilities in [0,100] with unknown as -1.
        The output map is a 2d numpy array of floats where:
        - [0, 1]: Probability of cell being filled.
        - np.nan: Unknown cell.
        """
        map_service_name = rospy.get_param("~static_map", "static_map")
        print(map_service_name)
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

    def motion_update(self, delta, prev_particle):
        x, y, heading = prev_particle

        # Turn stored odometry Poses into Particles
        prev_odom = Particle(
                self.prev_pose.position.x,
                self.prev_pose.position.y,
                quaternion_to_angle(self.prev_pose.orientation))
        this_odom = Particle(
                self.last_pose.position.x,
                self.last_pose.position.y,
                quaternion_to_angle(self.last_pose.orientation))

        # The amount to rotate the delta point before updating
        # (If odometry were perfect and there were no noise, this would be 0)
        heading_error = prev_particle.heading - prev_odom.heading

        delta_odom = Delta(
                this_odom.x - prev_odom.x,
                this_odom.y - prev_odom.y,
                this_odom.heading - prev_odom.heading)

        # Rotate the new odometry point around the old point by heading_error
        rot_delta_x, rot_delta_y = rotatexy(heading_error, delta_odom.x, delta_odom.y)

        # Add delta
        # YO comment this out to disable odometry updating.
        x += rot_delta_x
        y += rot_delta_y
        heading += delta_odom.heading

        # Add noise
        # YO comment this out to disable noise.
        x += self.RANDOMNESS.x * np.random.normal()
        y += self.RANDOMNESS.y * np.random.normal()
        heading += self.RANDOMNESS.heading * np.random.normal()

        return Particle(x, y, heading)

    def sensor_update(self, omap, scan_data, particle, method=ANGLE_MSE):
        """Calculate weight for particles given a map and sensor data.
        Basically the likelihood of the scan_data at the location.
        Args:
            omap: np array of occupancy
            scan_data: LaserScan message
            particles: np array of particles (positions)
        Returns:
            Weight for the particle.
        """

        # compute the set of angles in map space that should be checked
        map_space_laser_angles = self.laser_angles[0::self.ANGLE_STEP] + particle.heading

        # compute expected ranges from the given particle's position and orientation
        expected_ranges = \
            map(lambda angle: \
                fast_utils.calc_range(self.map_info, omap, particle.x, particle.y, angle, scan_data.range_max), map_space_laser_angles)

        # sample the corresponding ground truth ranges from the laser scan measurements
        ground_ranges = scan_data.ranges[0::self.ANGLE_STEP]

        def histogram_error(n_bins=20):
            useful_ground = []
            useful_expected = []
            
            for i in xrange(len(expected_ranges)):
                # ignore synthetic ranges that are at the limit, because they are
                # more often than not a result of holes in the map data and are inaccurate
                if not expected_ranges[i] == scan_data.range_max:
                    useful_ground.append(ground_ranges[i])
                    useful_expected.append(expected_ranges[i])

            ground_hist = np.histogram(useful_ground, n_bins, (scan_data.range_min, scan_data.range_max))[0]
            expected_hist = np.histogram(useful_expected, n_bins, (scan_data.range_min, scan_data.range_max))[0]

            return sum(abs(ground_hist - expected_hist))
        def mse_error():
            errs = []
            for i in xrange(len(expected_ranges)):
                # ignore synthetic ranges that are at the limit, because they are
                # more often than not a result of holes in the map data and are inaccurate
                if not expected_ranges[i] == scan_data.range_max:
                    distance_error = expected_ranges[i] - ground_ranges[i]

                    # weights closer measurements more heavily because they are more likely
                    # to be accurate
                    err = distance_error / ground_ranges[i]
                    errs.append(err*err) 

            return reduce(lambda x, y: x + y, errs) / len(errs)

        if method == ANGLE_MSE:
            err = mse_error()
        elif method == DISTANCE_HISTOGRAM:
            err = histogram_error()
        elif method == MIXTURE_1:
            err = math.sqrt(mse_error())*50 + histogram_error()
        elif method == MIXTURE_2:
            err = math.sqrt(mse_error())*histogram_error()

        # print math.sqrt(mse_error())*100, histogram_error()

        if err < self.current_best_err:
            # ranges relative to the car that are expected at the given position
            self.expected_ranges = expected_ranges
            self.current_best_err = err

        if err == 0:
            return float("inf")
        return 1.0 / err

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

        # don't start until scanner and odometry data is available
        if self.last_scan == None or self.last_pose == None:
            print "insufficient data"
            return

        # update particles and weights
        self.particles, self.particle_weights = self.MCL(
            self.omap, self.particles, [], self.last_scan)

        self.publish_particles(self.particles, self.particle_weights)
        self.publish_tf(self.particles, self.particle_weights)
        self.publish_expected_scan()

    def MCL(self, omap, previous_particles, odometry_delta, sensors):
        """Run one step of Monte Carlo localization."""
        rospy.logdebug("Attempting Monte Carlo Localization")

        # update particle positions based on odometry readings
        particles = \
            map(lambda old_particle: self.motion_update(odometry_delta, old_particle), \
                previous_particles)

        # reset accumulated odometry
        self.clear_odometry()

        self.current_best_err = 10000000
        # update particle weights according to probability of recording the given sensor readings
        particle_weights = \
            map(lambda old_particle: self.sensor_update(omap, sensors, old_particle), particles)

        # compute sum of all particle weights
        particle_mass = sum(particle_weights)

        # reweight particles - normalization of probability
        particle_weights = map(lambda x: float(x) / float(particle_mass), particle_weights)

        # fill new_particles by sampling from the reweighted particle array with replacement
        new_particles_indices = np.random.choice(len(particles), len(particles), True, particle_weights)
        new_particles = map(lambda idx: particles[idx], new_particles_indices)
        new_weights   = map(lambda idx: particle_weights[idx], new_particles_indices)

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
        print(bestParticle)

        pb = PoseStamped()
        pb.header = header
        pb.pose = particle_to_pose(bestParticle)
        self.pub_guess.publish(pb)

    def publish_tf(self, particles, particle_weights):
        """Publish a tf from map to odom.
        The tf is such that base_link appears in the same place as the best particle."""
        bestParticle = particles[np.argmax(particle_weights)]

        if self.last_pose == None:
            return

        odom_xy = np.array([self.last_pose.position.x, self.last_pose.position.y])
        guess_xy = np.array([bestParticle.x, bestParticle.y])
        diff_yaw = bestParticle.heading - quaternion_to_angle(self.last_pose.orientation)
        # diff_yaw = np.deg2rad(45)

        rodom_xy = rotatexy(diff_yaw, *odom_xy)

        tf_xy = guess_xy - rodom_xy
        tf_yaw = diff_yaw

        self.pub_tf.sendTransform(
            translation=(tf_xy[0], tf_xy[1], 0),
            rotation=tf.transformations.quaternion_from_euler(0, 0, tf_yaw),
            time=rospy.Time.now(),
            child="odom",
            parent="map")
    
    def publish_expected_scan(self):
        """Publish a LaserScan of the expected ranges.
        From the vantage point of the best particle."""

        if self.last_scan == None:
            return

        new_msg = LaserScan()
        new_msg.header = Header()
        new_msg.header.seq = self.expected_scan_seq
        self.expected_scan_seq += 1
        new_msg.header.stamp = rospy.Time.now()
        new_msg.header.frame_id = self.last_scan.header.frame_id

        new_msg.angle_min = self.last_scan.angle_min
        new_msg.angle_max = self.last_scan.angle_max
        new_msg.angle_increment = self.last_scan.angle_increment * self.ANGLE_STEP
        new_msg.time_increment = self.last_scan.time_increment
        new_msg.scan_time = self.last_scan.scan_time

        new_msg.range_min = self.last_scan.range_min
        new_msg.range_max = self.last_scan.range_max

        new_msg.ranges = self.expected_ranges
        new_msg.intensities = np.ones_like(self.expected_ranges)

        self.pub_expected_scan.publish(new_msg)

    def loop(self):
        # update visualization 
        if self.SHOW_VIS and self.last_scan and not self.expected_ranges == None:
            self.viz.laser_angular.set_data(self.laser_angles, self.last_scan.ranges)
            self.viz.laser_euclid.set_data(self.expected_angles, self.expected_ranges)

            self.viz.redraw()


def rotatexy(angle, x, y):
    """Rotate xy coordinates by some angle in 2d."""
    rotater = tf.transformations.rotation_matrix(angle=angle, direction=(0, 0, 1))
    rx, ry, _, _ = rotater.dot([x, y, 0, 1])
    return np.array([rx, ry])

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
        Localizer(True, False)
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
