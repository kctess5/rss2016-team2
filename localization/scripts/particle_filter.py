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

import matplotlib.pyplot as plt

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

class Localizer(object):
    def __init__(self, visualize=False, simulated=False):

        self.SHOW_VIS = visualize

        ll = rospy.get_param('log_level')
        rospy.init_node('localizer', anonymous=True, log_level=log_level(ll))
        rospy.loginfo("Initializing Monte Carlo Localization particle filter")

        # accumulated odometry delta - reset whenever the particle filter runs
        # self.this_odometry = Particle(0,0,0)
        # self.clear_odometry()
        self.last_pose = None
        self.prev_pose = None
        self.last_scan = None
        self.seq = 0

        # container for the persistent particles
        self.particles = []
        self.particle_weights = []

        # load all configuration variables
        self.configure()

        self.first_laser_recieved = False
        self.laser_angles = None
        self.expected_angles = None

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

        self.RANDOMNESS = Delta(0.2, 0.2, 0.1)
        self.NUM_PARTICLES = 15
        # number of times per second to attempt localization
        self.LOCALIZATION_FREQUENCY = 20.0
        # TODO - better initial pose management
        self.INITIAL_POSE = Particle(0.0,0,0.0)
        self.VISUALIZE = False

    def scan_callback(self, data):
        rospy.logdebug("Storing scan data")
        self.last_scan = data

        if self.laser_angles == None:
            self.laser_angles = np.linspace(data.angle_min, data.angle_max, math.ceil((data.angle_max - data.angle_min) / data.angle_increment))

    def odometry_callback(self, data):
        rospy.logdebug("Storing odometry data")
        # if self.last_pose == None:
        #     self.last_pose = data.pose.pose
        #     return

        # pos = data.pose.pose.position
        # orientation = data.pose.pose.orientation

        # calculate delta between this and last sensor reading
        # x_d = pos.x - self.last_pose.position.x
        # y_d = pos.y - self.last_pose.position.y
        # heading_d = quaternion_to_angle(orientation) - quaternion_to_angle(self.last_pose.orientation)

        # store deltas
        # aod = self.accumulated_odometry_delta
        # self.accumulated_odometry_delta = Delta(x_d + aod.x, y_d + aod.y, heading_d + aod.heading)

        # store this pose message for future use
        self.last_pose = data.pose.pose

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
        prev_odom = Particle(
                self.prev_pose.position.x,
                self.prev_pose.position.y,
                quaternion_to_angle(self.prev_pose.orientation))
        this_odom = Particle(
                self.last_pose.position.x,
                self.last_pose.position.y,
                quaternion_to_angle(self.last_pose.orientation))

        heading_error = prev_particle.heading - prev_odom.heading

        delta_odom = Delta(
                this_odom.x - prev_odom.x,
                this_odom.y - prev_odom.y,
                this_odom.heading - prev_odom.heading)

        rot_delta_x, rot_delta_y = self.rotatexy(heading_error, delta_odom.x, delta_odom.y)

        # Add delta
        # YO comment this out to disable odometry updating.
        x += rot_delta_x
        y += rot_delta_y
        heading += delta_odom.heading


        # # Add delta
        # # YO comment this out to disable odometry updating.
        # x += delta.x
        # y += delta.y
        # heading += delta.heading

        # Add noise
        # YO comment this out to disable noise.
        x += self.RANDOMNESS.x * np.random.normal()
        y += self.RANDOMNESS.y * np.random.normal()
        heading += self.RANDOMNESS.heading * np.random.normal()

        return Particle(x, y, heading)

    def sensor_update(self, omap, scan_data, particle, angle_step=5):
        """Calculate weight for particles given a map and sensor data.
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

        # compute expected ranges from the given particle's position and orientation
        expected_ranges = map(lambda angle: self.calc_range(omap, particle.x, particle.y, angle, scan_data.range_max), laser_angles)

        self.expected_angles = laser_angles
        self.expected_ranges = expected_ranges

        # sample the corresponding ground truth ranges from the laser scan measurements
        ground_ranges = scan_data.ranges[0::angle_step]
        ground_intensites = scan_data.intensities[0::angle_step]

        # print(ground_intensites)

        # self.show_angles = laser_angles
        # self.show_ranges = expected_ranges

        self.show_angles = []
        self.show_ranges = []

        errs = []
        for i in xrange(len(expected_ranges)):
            # ignore synthetic ranges that are at the limit, because they are
            # more often than not a result of holes in the map data and are inaccurate
            if not expected_ranges[i] == scan_data.range_max:
                self.show_angles.append(self.expected_angles[i])
                self.show_ranges.append(self.expected_ranges[i])

                distance_error = expected_ranges[i] - ground_ranges[i]
                intensity = ground_intensites[i]
                abs_range = ground_ranges[i]

                err = distance_error / (abs_range)

                errs.append(abs(err))

                # errs.append(ground_intensites[i] * (expected_ranges[i] - ground_ranges[i]) / ground_ranges[i])
                # errs.append(ground_intensites[i] * (expected_ranges[i] - ground_ranges[i]) / ground_ranges[i])

        # avg_err = reduce(lambda x, y: x + abs(y), errs) / len(errs)
        # avg_err = reduce(lambda x, y: x + y, errs) / len(errs)
        avg_err = reduce(lambda x, y: x + y, errs) / len(errs)
        return 1.0 / avg_err
        # return (1.0 / avg_err)**2

        # return 1.0 / (errs ** 2).mean()
                # print "max"
            # pass

        # print(len(expected_ranges), len(ground_ranges))

        # compute MSE between computed ranges and ground truth measurements
        # return 1.0 / ((expected_ranges - ground_ranges) ** 2).mean()


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

        # print("optimal", self.sensor_update(self.omap, self.last_scan, Particle(0,0,0)))

        # update particles and weights
        self.particles, self.particle_weights = self.MCL(
            self.omap, self.particles, self.accumulated_odometry_delta, self.last_scan)

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

        # update particle weights according to probability of recording the given sensor readings
        particle_weights = \
            map(lambda old_particle: self.sensor_update(omap, sensors, old_particle), previous_particles)

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

        print (bestParticle, particle_weights[np.argmax(particle_weights)])

        pb = PoseStamped()
        pb.header = header
        pb.pose = particle_to_pose(bestParticle)
        self.pub_guess.publish(pb)

    def publish_tf(self, particles, particle_weights):
        """Publish a tf from map to odom.
        The tf is such that base_link appears in the same place as the best particle."""
        bestParticle = particles[np.argmax(particle_weights)]

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
        # TODO get best particle and use position and real particle

        if self.last_scan == None:
            return

        # Copy the last scan, as a template for which angles to show.
        msg2 = copy.copy(self.last_scan)
        msg2.ranges = msg2.ranges.copy()

        # Angles where 0 is forwards for the robot.
        angles = (np.arange(msg2.ranges.shape[0]) * msg2.angle_increment) + msg2.angle_min

        # This makes a fuzzy halo. Just to see something to make sure it works.
        # expected_ranges = np.array([2.4 + np.random.rand() * 0.1
        #                             for angle in angles])

        bestParticle = self.particles[np.argmax(self.particle_weights)]
        expected_ranges = [self.calc_range(self.omap, bestParticle.x, bestParticle.y,
                                           angle + bestParticle.heading, msg2.range_max)
                           for angle in angles]

        msg2.ranges[:] = expected_ranges

        self.pub_expected_scan.publish(msg2)

    def loop(self):
        # update visualization 
        if self.SHOW_VIS and self.last_scan and not self.expected_angles == None:

            self.viz.laser_angular.set_data(self.laser_angles, self.last_scan.ranges)
            # self.viz.laser_euclid.set_data(self.expected_angles, self.expected_ranges)
            self.viz.laser_euclid.set_data(self.show_angles, self.show_ranges)
            # self.viz.ax2.imshow(self.image)
            
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
        Localizer(True, True)
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
