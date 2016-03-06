#!/usr/bin/env python
from __future__ import print_function
import rospy
from geometry_msgs.msg import Quaternion
from car_controller.helpers import CoordinateHelpers
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from rospy.numpy_msg import numpy_msg
import time
import matplotlib.pyplot as plt
import math


# class Visualization(object):
# 	"""docstring for Visualization"""
# 	def __init__(self):
# 		plt.ion()
# 		#Set up plot
# 		self.fig = plt.figure(figsize=plt.figaspect(2.))
		
# 		self.ax0 = self.fig.add_subplot(1,2,1)

# 		self.laser_scanner = self.ax0.plot([],[], '.')

# 		self.ax0.set_ylim(-1, 25)
# 		self.ax0.set_xlim(-math.pi, +math.pi)
# 		self.ax0.invert_xaxis()
# 		self.ax0.grid()
		
# 		self.ax1 = self.fig.add_subplot(1,2,2) 
# 		self.ax1.set_ylim(-1, 25)
# 		self.ax1.set_xlim(-10, 10)
# 		self.ax1.invert_xaxis()
# 		self.ax1.grid()
# 		self.laser_euclid, = self.ax1.plot([],[], '.')
# 		self.laser_euclid_eroded, = self.ax1.plot([],[], 'r.')
# 		self.ax1.plot(0,0, 'ko')
# 		self.current_heading_angle, = self.ax1.plot([0,0],[0,1], 'k-')
# 		self.desired_heading_angle, = self.ax1.plot([],[], 'g-')
# 		self.selected_maneuver, = self.ax1.plot([],[], 'c-')
# 		#self.costmap = self.ax1.pcolormesh(np.ones((10,10)))
		
# 		self.redraw()
		
# 	def redraw(self):
# 		#Need both of these in order to rescale
# 		self.ax0.relim()
# 		self.ax0.autoscale_view()
		
# 		self.ax1.relim()
# 		self.ax1.autoscale_view()
		
# 		#We need to draw *and* flush
# 		self.fig.canvas.draw()
# 		self.fig.canvas.flush_events()
		
# 	def update_scan(self, scan_data):
# 		laser_angles = np.linspace(scan_data.angle_min, scan_data.angle_max, math.ceil((scan_data.angle_max - scan_data.angle_min) / scan_data.angle_increment))


# 		self.laser_scanner.set_data(laser_angles, scan_data.ranges)
# 		pass

# 	def update_image(self, img_data):
# 		pass

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

class CoordPlayground():
	""" A place to test coordinate system related things """
	def __init__(self):
		self.SHOW_VIS = True

		self.node = rospy.init_node('coord_playground', anonymous=True)

		self.img_sub = rospy.Subscriber('/camera/zed/rgb/image_rect_color', Image, self.img_callback)
		self.scan_sub = rospy.Subscriber('/scan', numpy_msg(LaserScan), self.scan_callback)

		self.scan = None
		self.img = None

		self.first_laser_recieved = False 

		self.last_process = time.clock()

		self.bridge = CvBridge()
		# self.viz = Visualization()
		self.viz = DynamicPlot()
		self.viz.initialize()

		if self.SHOW_VIS:
			while not rospy.is_shutdown():
				self.loop()
				rospy.sleep(0.1)

	def img_callback(self, data):
		if self.scan:
			self.process(self.scan, data)
			self.scan = None
			self.img = None
		else:
			self.img = data

	def scan_callback(self, data):
		if self.img:
			self.process(data, self.img)
			self.scan = None
			self.img = None
		else:
			self.scan = data
		
	def ros_to_cvimg(self, rosimg):
		ENC_GRAYSCALE = "mono8"
		ENC_COLOR = "bgr8"
		# return self.bridge.toCvShare(rosimg, ENC_GRAYSCALE)
		return self.bridge.imgmsg_to_cv2(rosimg, desired_encoding=ENC_COLOR)

	def process(self, scan, image):
		if time.clock() - self.last_process > 0.01:
			self.last_process = time.clock()
			
			self.laser_angles = np.linspace(scan.angle_min, scan.angle_max, math.ceil((scan.angle_max - scan.angle_min) / scan.angle_increment))
			self.laser_ranges = scan.ranges

			self.laser_x, self.laser_y = CH.polar_to_euclid(self.laser_angles, self.laser_ranges)

			self.image = self.ros_to_cvimg(image)
			self.first_laser_recieved = True

	def loop(self):
		# update visualization 
		if self.SHOW_VIS and self.first_laser_recieved:

			self.viz.laser_angular.set_data(self.laser_angles, self.laser_ranges)
			self.viz.laser_euclid.set_data(self.laser_x, self.laser_y)
			self.viz.ax2.imshow(self.image)
			
			self.viz.redraw()

if __name__ == '__main__':

	print(CH.keypoint_to_image(-.29, 3.25))
	# cp = CoordPlayground()

