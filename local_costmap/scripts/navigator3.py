#!/usr/bin/env python
from __future__ import print_function
import rospy

import numpy as np
from helpers import *
from scipy import signal
import matplotlib.pyplot as plt


class DynamicPlot():

	def initialize(self):
		plt.ion()
		#Set up plot
		self.fig = plt.figure(figsize=plt.figaspect(2.))
		
		self.ax0 = self.fig.add_subplot(1,2,1)
		self.laser_angular, = self.ax0.plot([],[], '.')
		
		self.laser_filtered, = self.ax0.plot([],[], 'r-')
		self.laser_maxima, = self.ax0.plot([],[], 'wo')
		self.desired_heading_point, = self.ax0.plot([],[], 'ro')
		
		self.ax0.set_ylim(-1, 40)
		self.ax0.set_xlim(-math.pi, +math.pi)
		self.ax0.grid()
		
		self.ax1 = self.fig.add_subplot(1,2,2) 
		self.ax1.set_ylim(-10, 10)
		self.ax1.set_xlim(-10, 10)
		self.ax1.grid()
		self.laser_euclid, = self.ax1.plot([],[], '.')
		self.ax1.plot(0,0, 'ko')
		self.current_heading_angle, = self.ax1.plot([0,1],[0,0], 'k-')
		self.desired_heading_angle, = self.ax1.plot([],[], 'g-')
		self.selected_maneuver, = self.ax1.plot([],[], 'c-')
		#self.costmap = self.ax1.pcolormesh(np.ones((10,10)))
		
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

class ScanMaximaNavigator(object):
	""" This class directly filters laser scanner data to detect corridors. It provides
		a goal point for the path search module
	"""
	def __init__(self, viz):
		self.viz = viz

		self.window = None
		self.median_filter_width = None
		self.gaussian_filter_width = None

		self.local_viz = None
		self.angles = None
		self.filtered_ranges = None
		self.peaks = None
		self.picked_peak = None

		if param("navigator3.show_local_viz"):
			self.local_viz = DynamicPlot()
			self.local_viz.initialize()

	def scan_callback(self, laser):

		# filter the laser data to only include a subset of useful data
		num_scans = len(laser.ranges)
		trim_length = int(num_scans * param("navigator3.trim_percentage"))

		ranges = np.array(laser.ranges[trim_length:-trim_length])
		# have to account for trimmed section in the angles calculation
		angles = (np.arange(ranges.shape[0]) * laser.angle_increment) \
				 + laser.angle_min + laser.angle_increment * trim_length

		less_ranges = ranges[::param("navigator3.downsample")]
		less_angles = angles[::param("navigator3.downsample")]

		less_ranges[less_ranges < laser.range_min] = laser.range_min
		less_ranges[less_ranges > param("navigator3.laser_max")] = param("navigator3.laser_max")
		
		if self.window == None:
			# only compute these constants once
			self.gaussian_filter_width = int(param("navigator3.gaussian_width") * less_ranges.shape[0])
			self.median_filter_width = int(param("navigator3.median_width") * less_ranges.shape[0])

			if self.median_filter_width % 2 == 0:
				self.median_filter_width += 1

			win = signal.hann(self.gaussian_filter_width)
			self.window = win / np.sum(win)

			self.laser_steps_per_radian = less_angles.shape[0] / (less_angles[-1] - less_angles[0])

		median_ranges = signal.medfilt(less_ranges, self.median_filter_width)
		smoothed_ranges = signal.convolve(median_ranges, self.window, mode='same')

		self.find_corridor(smoothed_ranges, less_angles)

		if self.local_viz:
			self.angles = less_angles
			self.filtered_ranges = smoothed_ranges

	def goalpoint(self):
		return self.goal_point

	def find_corridor(self, filtered_ranges, angles):
		# given prefiltered scanner data, find corridors by finding local maxima
		
		peaks = signal.argrelextrema(filtered_ranges, np.greater_equal)[0]

		angle_bounds = param("navigator3.goal_angle_bounds")
		candidate_peak_indices = np.where((angles[peaks] > angle_bounds[0])
			& (angles[peaks] < angle_bounds[1])
			& (filtered_ranges[peaks] > param("navigator3.distance_threshold")))
		candidate_peaks = peaks[candidate_peak_indices]

		if candidate_peaks.shape[0] == 0:
			print("Failed to find corridors")
			self.goal_point = None
			return None

		if param("navigator3.turn_right"):
			picked_peak = candidate_peaks[0]
		else:
			picked_peak = candidate_peaks[-1]

		point = polar_to_euclid(angles[picked_peak], filtered_ranges[picked_peak] - 0.5)
		
		self.goal_point = (point[0], point[1], angles[picked_peak])

		if self.local_viz:
			self.peaks = peaks
			self.picked_peak = picked_peak


	def visualize(self):
		if self.local_viz and not self.angles == None:
			self.local_viz.laser_filtered.set_data(self.angles, self.filtered_ranges)
			if (self.peaks.shape[0] > 1):
				self.local_viz.laser_maxima.set_data(self.angles[self.peaks], \
					self.filtered_ranges[self.peaks])
			else:
				self.local_viz.laser_maxima.set_data([], [])

			if not self.picked_peak == None:	
				chosen_angle = self.angles[self.picked_peak]
				chosen_range = self.filtered_ranges[self.picked_peak]

				self.local_viz.desired_heading_point.set_data(chosen_angle, chosen_range)
				tip_len = 2.5
				self.local_viz.desired_heading_angle.set_data([0, tip_len * np.cos(chosen_angle) ],[0, tip_len * np.sin(chosen_angle)])

				if self.viz.should_visualize("goals.corridor_goal") and self.goal_point:
					self.viz.publish_corridor_goal(self.goal_point)

			self.local_viz.redraw()

