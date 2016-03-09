#!/usr/bin/env python
from __future__ import print_function
import rospy
from geometry_msgs.msg import Quaternion
from car_controller.control_module import ControlModule
from car_controller.helpers import CoordinateHelpers, ColorThief
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from scipy import misc
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from rospy.numpy_msg import numpy_msg
import time
import matplotlib.pyplot as plt
import math
import message_filters
from cone_follower.msg import PolarPoints, PolarPoint

class DynamicPlot():

	def initialize(self):
		plt.ion()
		#Set up plot
		self.fig = plt.figure(figsize=plt.figaspect(2.))
		
		self.ax0 = self.fig.add_subplot(1,3,1)

		self.laser_angular, = self.ax0.plot([],[], 'r.')
		self.laser_filtered, = self.ax0.plot([],[], 'r-')
		self.laser_eroded, = self.ax0.plot([],[], 'c-')
		self.laser_maxima, = self.ax0.plot([],[], 'wo')
		self.desired_heading_point, = self.ax0.plot([],[], 'ro')
		
		self.ax0.set_ylim(-1, 15)
		self.ax0.set_xlim(-math.pi / 4, +math.pi / 4)
		self.ax0.invert_xaxis()
		self.ax0.grid()
		
		self.ax1 = self.fig.add_subplot(1,3,2) 
		self.ax1.set_ylim(-1, 10)
		self.ax1.set_xlim(-7, 7)
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
		self.ax2 = self.fig.add_subplot(1,3,3) 
		
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

CH = CoordinateHelpers()

RIGHT = 1
LEFT = 2
CENTER = 3

def dist(p1, p2):
	d = 0
	for i in xrange(0,len(p1)):
		d = d + (p1[i] - p2[i]) * (p1[i] - p2[i])
	return math.sqrt(d)

def distance_at(angle, data):
	delta = angle - data.angle_min
	index = int (delta / data.angle_increment)
	return data.ranges[index]

class ConeFollower(ControlModule):
	""" The controller for lab 4B - weave between cones """
	def __init__(self, turn_dir=RIGHT, visualize=False):
		super(ConeFollower, self).__init__("cone_weaver")

		self.SHOW_VIS = visualize

		self.node = rospy.init_node('cone_weaver', anonymous=True)

		image_sub = message_filters.Subscriber('/camera/rgb/image_rect_color', Image)
		scan_sub = message_filters.Subscriber('/scan', numpy_msg(LaserScan))
		keypoint_sub = message_filters.Subscriber('/cone/key_points', numpy_msg(PolarPoints))
		
		self.sync = message_filters.ApproximateTimeSynchronizer([scan_sub, image_sub, keypoint_sub], 1, 0.08)
		# self.sync = message_filters.TimeSynchronizer([image_sub], 1000)
		self.sync.registerCallback(self.process)

		self.first_laser_recieved = False 
		self.last_cone_position = (0,0)
		self.last_process = time.clock()
		self.last_steering_angle = 0
		self.CONE_DIST = 0.4 # desired cone distance on each side (meters)
		self.turn_direction = turn_dir # initial side of the cone to target

		# if consecutative cone positions are greater than this distance apart, they are 
		# considered to be different cones, and the turn direction is flipped to snake 
		# between the cones
		self.CONE_FLIP_DISTANCE = 0.8
		self.last_cone_flip = time.clock()

		self.CONE_COLOR = (10, 60, 175)
		# self.CONE_COLOR = (60, 100, 120)
		self.COLOR_DISTANCE_CUTTOFF = 50
		self.CONE_PERCENTAGE_CUTTOFF = .18
		self.CAR_WHEELBASE = 0.33 # (meters)
		self.CAR_DRIVE_SPEED = .4
		self.STRAIGHT_TIME = .5
		self.STOPPING_DISTANCE = 0.75

		self.bridge = CvBridge()
		
		if self.SHOW_VIS:
			self.viz = DynamicPlot()
			self.viz.initialize()
			while not rospy.is_shutdown():
				self.loop()
				rospy.sleep(0.1)

	def ros_to_cvimg(self, rosimg):
		ENC_GRAYSCALE = "mono8"
		ENC_COLOR = "bgr8"
		# return self.bridge.toCvShare(rosimg, ENC_GRAYSCALE)
		return self.bridge.imgmsg_to_cv2(rosimg, desired_encoding=ENC_COLOR)

	def cone_in_roi(self, roi, quality=2):
		width, height, depth = roi.shape
		dists = []
		num_pixels = float(width) * float(height) / float(quality * quality)

		if num_pixels == 0:
			return 0
			
		cone_pixels = 0

		main_r = 0
		main_g = 0
		main_b = 0

		for x in xrange(0, width, quality):
			for y in xrange(0, height, quality):
				data = roi[x, y]
				r, g, b = int(data[0]), int(data[1]), int(data[2])
				d = dist([r,g,b], self.CONE_COLOR)

				if (d < self.COLOR_DISTANCE_CUTTOFF):
					main_r = r + main_r
					main_b = b + main_b
					main_g = g + main_g
					cone_pixels += 1
		
		if (cone_pixels and (float(cone_pixels) / num_pixels) > self.CONE_PERCENTAGE_CUTTOFF):
			print((main_r) / cone_pixels, (main_g) / cone_pixels, (main_b) / cone_pixels)

		return (float(cone_pixels) / num_pixels) > self.CONE_PERCENTAGE_CUTTOFF

	def process(self, scan, image, keypoints):
		if time.clock() - self.last_process > 0.01 and scan and image and keypoints:
			self.last_process = time.clock()

			if self.SHOW_VIS:
				self.laser_angles = np.linspace(scan.angle_min, scan.angle_max, math.ceil((scan.angle_max - scan.angle_min) / scan.angle_increment))
				self.laser_ranges = scan.ranges
				self.laser_x, self.laser_y = CH.polar_to_euclid(self.laser_angles, self.laser_ranges)

			self.image = self.ros_to_cvimg(image)

			cones = []

			for keypoint in keypoints.points:
				image_x, image_y = CH.keypoint_to_image(keypoint.angle, keypoint.distance)

				height, width, depth = self.image.shape

				if (image_x >= 50 and image_x < width - 50 and \
					image_y >= 50 and image_y < height - 50):

					ROIheight = 90 / keypoint.distance
					ROIwidth = 90 / keypoint.distance

					ROI = self.image[image_y-ROIheight:image_y+ROIheight, image_x-ROIwidth:image_x+ROIwidth, 0:3]

					if self.cone_in_roi(ROI):
						# small_img = misc.imresize(ROI, (50,50))
						cones.append((image_x, image_y, keypoint))
						if self.SHOW_VIS:
							cv2.circle(self.image, (image_x, image_y), 10, (255,0,0), 20)
					elif self.SHOW_VIS:
						cv2.circle(self.image, (image_x, image_y), 10, (0,255,0), 10)

			cones.sort(key=lambda cone: cone[2].distance)
			if len(cones):
				cone_position = CH.polar_to_euclid(cones[0][2].angle, cones[0][2].distance)

				if (dist(cone_position, self.last_cone_position)) > self.CONE_FLIP_DISTANCE \
					and not self.turn_direction == CENTER:
					self.flip_cone_direction()

				self.last_cone_position = cone_position

				self.drive_to(cones[0], self.turn_direction)
			else:
				self.drive_to(None)
			print ("found: ", len(cones), "cones")

			self.first_laser_recieved = True
			# cones.sort(key=lambda cone: cone[2].distance)
			# if len(cones):
			# 	self.drive_to(cones[0], RIGHT)
			# else:
			# 	self.drive_to(None)
			# print (cones)

			# self.first_laser_recieved = True

	def flip_cone_direction(self):
		print("FLIPPING DIRECTION")
		if self.turn_direction == RIGHT:
			self.turn_direction = LEFT
		elif self.turn_direction == LEFT:
			self.turn_direction = RIGHT

		self.last_cone_flip = time.clock()

	def find_steer_goal(self, cone, side):
		cone_theta = cone[2].angle
		cone_d = cone[2].distance

		if side == RIGHT:
			dist = np.sqrt(cone_d * cone_d + (1 * self.CONE_DIST + 0.12) * self.CONE_DIST)
			theta = cone_theta - np.arcsin((1 * self.CONE_DIST + 0.12) / dist)
			# sign = 1
		elif side == LEFT:
			dist = np.sqrt(cone_d * cone_d + -1 * self.CONE_DIST * self.CONE_DIST)
			theta = cone_theta - np.arcsin(-1 * self.CONE_DIST / dist)
			# sign = -1
		elif side == CENTER:
			dist = cone_d
			theta = cone_theta
		
		# dist = np.sqrt(cone_d * cone_d + sign * self.CONE_DIST * self.CONE_DIST)
		# theta = cone_theta - np.arcsin(sign * self.CONE_DIST / dist)

		return (theta, dist)

	def find_steer_actuation(self, goal_theta, goal_d):
		return np.arctan2(self.CAR_WHEELBASE * np.sin(2.0 * goal_theta), \
						  goal_d * np.sin(math.pi / 2.0 - goal_theta))

	def drive_to(self, cone, side=RIGHT):
		control_msg = self.make_message("direct_drive")

		control_msg.drive_msg.speed = 0

		if cone == None:
			self.last_steering_angle = 0
			self.control_pub.publish(control_msg)
			return

		control_msg.drive_msg.speed = self.CAR_DRIVE_SPEED

		if time.clock() - self.last_cone_flip < self.STRAIGHT_TIME:
			print ("GOING STRAIGHT")
			control_msg.drive_msg.steering_angle = 0
			self.control_pub.publish(control_msg)
			return

		if side == CENTER and cone[2].distance < self.STOPPING_DISTANCE:
			control_msg.drive_msg.speed = 0
			control_msg.drive_msg.steering_angle = 0
			self.control_pub.publish(control_msg)
			return

		control_msg.drive_msg.steering_angle = self.find_steer_actuation(*self.find_steer_goal(cone, side))
		self.last_steering_angle = control_msg.drive_msg.steering_angle

		self.control_pub.publish(control_msg)

	def loop(self):
		# update visualization 
		if self.SHOW_VIS and self.first_laser_recieved:

			self.viz.laser_angular.set_data(self.laser_angles, self.laser_ranges)
			self.viz.laser_euclid.set_data(self.laser_x, self.laser_y)
			self.viz.ax2.imshow(self.image)
			
			self.viz.redraw()

if __name__ == '__main__':
	cf = ConeFollower(CENTER)
	def kill():
		print("unsubscribe")
		cf.unsubscribe()
	rospy.on_shutdown(kill)
	rospy.spin()

