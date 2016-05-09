#!/usr/bin/env python
from __future__ import print_function
import rospy

import numpy as np
from helpers import *
from scipy import signal
import matplotlib.pyplot as plt
import threading


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
	def __init__(self, viz, opt_mode=False):
		self.opt_mode = opt_mode
		self.viz = viz

		self.window = None
		self.median_filter_width = None
		self.gaussian_filter_width = None

		self.local_viz = None
		self.angles = None
		self.filtered_ranges = None
		self.peaks = None
		self.picked_peak = None
		self.goal_point = None
		self.opt_params = None

		self.goal_buffer = np.zeros([int(param("navigator.smooth_samples")), 3])
		self.goal_index = 0

		self.lock = threading.Lock()

		if self.get_nav_param("show_local_viz") and not self.opt_mode:
			self.local_viz = DynamicPlot()
			self.local_viz.initialize()
	
	def set_params(self, opt_params):
		self.opt_params = opt_params

	def get_nav_param(self, key):
		if not self.opt_params == None and key in self.opt_params:
			return self.opt_params[key]
		else:
			return param("navigator." + key)

	def scan_callback(self, laser):
		# filter the laser data to only include a subset of useful data
		num_scans = len(laser.ranges)
		trim_length = int(num_scans * self.get_nav_param("trim_percentage"))

		ranges = np.array(laser.ranges[trim_length:-trim_length])
		# have to account for trimmed section in the angles calculation
		angles = (np.arange(ranges.shape[0]) * laser.angle_increment) \
				 + laser.angle_min + laser.angle_increment * trim_length

		if self.get_nav_param("use_prefiltering"):
			bad_indices = np.where(ranges > self.get_nav_param("error_range"))[0]
			obviously_good_indices = np.where(ranges < self.get_nav_param("error_range"))[0]

			contiguous_min = self.get_nav_param("contiguous_min")
			good_indices = []
			num_contiguous = 0
			last_index = -1
			for i in bad_indices:
				if i - last_index == 1:
					num_contiguous += 1
				else:
					num_contiguous = 0

				if num_contiguous > contiguous_min:
					good_indices.append(i)
				if num_contiguous == contiguous_min:
					# print(num_contiguous, list(np.arange(i - num_contiguous+1, i+1, 1)))
					good_indices = good_indices + list(np.arange(i-num_contiguous+2, i+1, 1))
				last_index = i

			good_indices = np.array(sorted(list(obviously_good_indices) + good_indices))

			ranges = ranges[good_indices]
			angles = angles[good_indices]

		less_ranges = ranges[::self.get_nav_param("downsample")]
		less_angles = angles[::self.get_nav_param("downsample")]

		less_ranges[less_ranges > self.get_nav_param("error_range")] = self.get_nav_param("laser_max")

		if self.window == None:
			# only compute these constants once
			self.gaussian_filter_width = int(self.get_nav_param("gaussian_width") * less_ranges.shape[0])
			self.median_filter_width = int(self.get_nav_param("median_width") * less_ranges.shape[0])

			if self.median_filter_width % 2 == 0:
				self.median_filter_width += 1

			win = signal.hann(self.gaussian_filter_width)
			self.window = win / np.sum(win)

		if self.get_nav_param("median_width") == 0.0:
			median_ranges = less_ranges
		else:
			median_ranges = signal.medfilt(less_ranges, self.median_filter_width)
		
		if self.get_nav_param("gaussian_width") == 0.0:
			smoothed_ranges = median_ranges
		else:
			smoothed_ranges = signal.convolve(median_ranges, self.window, mode='same')

		self.find_corridor(smoothed_ranges, less_angles)

	def goalpoint(self):
		return self.goal_point

	def set_goal_point(self, gp):
		if gp == None:
			self.goal_buffer[self.goal_index % param("navigator.smooth_samples"), 0] = np.inf
			self.goal_buffer[self.goal_index % param("navigator.smooth_samples"), 1] = np.inf
			self.goal_buffer[self.goal_index % param("navigator.smooth_samples"), 2] = np.inf
		else:
			self.goal_buffer[self.goal_index % param("navigator.smooth_samples"), :] = gp
		self.goal_index += 1

		if self.goal_index < param("navigator.smooth_samples"):
			self.goal_point = gp
		else:
			dists = np.linalg.norm(self.goal_buffer[:,0:2], axis=1)
			angles = self.goal_buffer[:,2]

			median_distance = np.median(dists)
			median_angle = np.median(angles)

			if np.isinf(median_angle) or np.isinf(median_distance):
				self.goal_point == None
				return

			pt = polar_to_euclid(median_angle, median_distance)
			self.goal_point = [pt[0], pt[1], median_angle]

	def find_corridor(self, filtered_ranges, angles):
		# given prefiltered scanner data, find corridors by finding local maxima
		peaks = signal.argrelextrema(filtered_ranges, np.greater_equal)[0]

		
		angle_max = self.get_nav_param("goal_angle_max")
		angle_min = self.get_nav_param("goal_angle_min")
		candidate_peak_indices = np.where((angles[peaks] > angle_min)
			& (angles[peaks] < angle_max)
			& (filtered_ranges[peaks] > self.get_nav_param("distance_threshold")))
		candidate_peaks = peaks[candidate_peak_indices]

		if candidate_peaks.shape[0] == 0:
			if not self.opt_mode:
				print("Failed to find corridors")
			self.set_goal_point(None)
			# self.goal_point = None
			return None

		if self.get_nav_param("turn_right"):
			picked_peak = candidate_peaks[0]
		else:
			picked_peak = candidate_peaks[-1]

		point = polar_to_euclid(angles[picked_peak], filtered_ranges[picked_peak] - 1.5)
		
		self.set_goal_point(np.array([float(point[0]), float(point[1]), float(angles[picked_peak])]))
		# print(float(angles[picked_peak]))
		if self.local_viz:
			with self.lock:
				self.peaks = peaks
				self.picked_peak = picked_peak
				self.angles = angles
				self.filtered_ranges = filtered_ranges


	def visualize(self):
		with self.lock:
			if self.viz.should_visualize("goals.corridor_goal") and not self.goal_point == None:
					self.viz.publish_corridor_goal(self.goal_point)

			if self.local_viz and not self.angles == None:
				self.local_viz.laser_filtered.set_data(self.angles, self.filtered_ranges)
				if (not self.peaks == None and self.peaks.shape[0] > 1):
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

					

				self.local_viz.redraw()

