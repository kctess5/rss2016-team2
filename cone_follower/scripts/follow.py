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
	def __init__(self, visualize=False):
		super(ConeFollower, self).__init__("cone_weaver")

		self.SHOW_VIS = visualize

		self.node = rospy.init_node('cone_weaver', anonymous=True)

		self.img_sub = rospy.Subscriber('/camera/zed/rgb/image_rect_color', Image, self.img_callback)
		self.scan_sub = rospy.Subscriber('/scan', numpy_msg(LaserScan), self.scan_callback)
		self.keypoint_sub = rospy.Subscriber('/cone/key_points', numpy_msg(PolarPoints), self.keypoint_callback)

		self.scan = None
		self.img = None
		self.keypoints = None
		self.first_laser_recieved = False 
		self.last_process = time.clock()

		self.CONE_DIST = 0.2 # desired cone distance on each side (meters)
		self.CONE_COLOR = (4,20,200)
		self.COLOR_DISTANCE_CUTTOFF = 50
		self.CONE_PERCENTAGE_CUTTOFF = .18
		self.CAR_WHEELBASE = 0.33 # (meters)
		self.CAR_DRIVE_SPEED = 0.5

		self.bridge = CvBridge()
		
		if self.SHOW_VIS:
			self.viz = DynamicPlot()
			self.viz.initialize()
			while not rospy.is_shutdown():
				self.loop()
				rospy.sleep(0.1)

	def img_callback(self, data):
		if self.scan and self.keypoints:
			self.process(self.scan, data, self.keypoints)
			self.scan = None
			self.img = None
			self.keypoints = None
		else:
			self.img = data

	def scan_callback(self, data):
		if self.img and self.keypoints:
			self.process(data, self.img, self.keypoints)
			self.scan = None
			self.img = None
			self.keypoints = None
		else:
			if self.scan == None:
				self.scan = data
	
	def keypoint_callback(self, data):
		if self.img and self.scan:
			self.process(self.scan, self.img, data)
			self.scan = None
			self.img = None
			self.keypoints = None
		else:
			if self.keypoints == None:
				self.keypoints = data

	def ros_to_cvimg(self, rosimg):
		ENC_GRAYSCALE = "mono8"
		ENC_COLOR = "bgr8"
		# return self.bridge.toCvShare(rosimg, ENC_GRAYSCALE)
		return self.bridge.imgmsg_to_cv2(rosimg, desired_encoding=ENC_COLOR)

	def cone_in_roi(self, roi, quality=2):
		width, height, depth = roi.shape
		dists = []
		num_pixels = float(width) * float(height) / float(quality * quality)
			
		cone_pixels = 0

		for x in xrange(0, width, quality):
			for y in xrange(0, height, quality):
				data = roi[x, y]
				r, g, b = int(data[0]), int(data[1]), int(data[2])
				d = dist([r,g,b], self.CONE_COLOR)

				if (d < self.COLOR_DISTANCE_CUTTOFF):
					cone_pixels += 1

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
				self.drive_to(cones[0], RIGHT)
			else:
				self.drive_to(None)
			print (cones)

			self.first_laser_recieved = True

	def find_steer_goal(self, cone, side):
		cone_theta = cone[2].angle
		cone_d = cone[2].distance
		
		dist = np.sqrt(cone_d * cone_d + self.CONE_DIST * self.CONE_DIST)
		theta = cone_theta - np.arcsin(self.CONE_DIST / dist)

		return (theta, dist)

	def find_steer_actuation(self, goal_theta, goal_d):
		return np.arctan2(self.CAR_WHEELBASE * np.sin(2.0 * goal_theta), \
			              goal_d * np.sin(math.pi / 2.0 - goal_theta))

	def drive_to(self, cone, side=RIGHT):
		control_msg = self.make_message("direct_drive")

		control_msg.drive_msg.speed = 0
		control_msg.drive_msg.steering_angle = 0

		if cone == None:
			self.control_pub.publish(control_msg)
			return

		control_msg.drive_msg.speed = self.CAR_DRIVE_SPEED
		control_msg.drive_msg.steering_angle = self.find_steer_actuation(*self.find_steer_goal(cone, side))

		self.control_pub.publish(control_msg)

	def loop(self):
		# update visualization 
		if self.SHOW_VIS and self.first_laser_recieved:

			self.viz.laser_angular.set_data(self.laser_angles, self.laser_ranges)
			self.viz.laser_euclid.set_data(self.laser_x, self.laser_y)
			self.viz.ax2.imshow(self.image)
			
			self.viz.redraw()

if __name__ == '__main__':
	cf = ConeFollower()
	def kill():
		print("unsubscribe")
		cf.unsubscribe()
	rospy.on_shutdown(kill)
	rospy.spin()

