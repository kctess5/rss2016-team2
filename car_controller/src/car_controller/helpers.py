#!/usr/bin/env python

import math
import numpy as np

IN_TO_M = 0.0254

def float_indexer(np_matrix):
	def at(*indices):
		val = np_matrix
		for index in indices:
			val = val[index]

		return float (val)
	return at

def xyz(np_mat):
	ind = float_indexer(np_mat)
	return (ind(0, 0), ind(1, 0), ind(2, 0))

def translation_matrix(x,y,z):
	return np.matrix([[1,0,0,x], [0,1,0,y], [0,0,1,z], [0,0,0,1]])

def in_to_m(dim):
	return dim * IN_TO_M

class CoordinateHelpers(object):
	""" A collection of Python helper functions for RACECAR specific
		coordinate frame conversions
	"""

	def __init__(self):
		# if: 
		# 	+z is forward
		# 	+y is upwards
		# 	+x is rightwards
		# camera is offset by (inches):
		# 	x: -2.36
		# 	y: -1
		# 	z: 2
		self.laser_to_camera_transfo = translation_matrix( \
			in_to_m(-2.36), in_to_m(-1), in_to_m(2))

		# print (self.laser_to_camera_transfo)

	# converts a given polar coordinate (or np array) to euclidean space
	def polar_to_euclid(self, angles, ranges):
		y = np.cos(angles) * ranges
		x = np.sin(angles) * ranges
		return (x,y)

	def euclid_to_polar(self, x, y):
		rho = np.sqrt(x*x + y*y)
		phi = np.arctan2(x, y)
		return (phi, rho)

	# converts from the laser scanner frame to the ZED rgb camera frame
	def laser_to_camera_euclid(self, x, y, z):
		coords = np.matrix([[x],[y],[z],[1]])
		new_coords = self.laser_to_camera_transfo * coords 
		return xyz(new_coords)

	def estimate_camera_polar_to_image(self, angle, dist):
		return (int (-783.73 * angle + 635.63), 350)

	def keypoint_to_image(self, angle, dist):
		laser_x, laser_z = self.polar_to_euclid(angle, dist)
		camera_x, camera_y, camera_z = self.laser_to_camera_euclid(laser_x, 0, laser_z)
		camera_angle, camera_dist = self.euclid_to_polar(camera_x, camera_z)
		image_x, image_y = self.estimate_camera_polar_to_image(camera_angle, camera_dist)
		return (image_x, image_y)

