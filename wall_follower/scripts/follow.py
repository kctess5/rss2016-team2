#!/usr/bin/env python
from __future__ import print_function
import rospy
from sensor_msgs.msg import LaserScan
from rospy.numpy_msg import numpy_msg
from scipy import signal
import math
import numpy as np
from car_controller.control_module import ControlModule

LEFT = 1
RIGHT = 2

def distance_at(angle, data):
	delta = angle - data.angle_min
	index = int (delta / data.angle_increment)
	return data.ranges[index]

class WallFollower(ControlModule):
	def __init__(self, simulate=False, follow=LEFT):
		# initialize control module with name "test_module"
		super(WallFollower, self).__init__("test_module")

		self.follow = follow
		self.simulate = simulate
		self.configure()

		self.scan_subscriber = rospy.Subscriber(self.LASER_SCAN_TOPIC, numpy_msg(LaserScan), self.scan_callback)

	def configure(self):
		if self.simulate:
			self.LASER_SCAN_TOPIC = '/racecar/laser/scan'
		else:
			self.LASER_SCAN_TOPIC = '/scan'
		self.first_laser_recieved = False 
		self.target_dist = 1 # in meters
		self.target_angle = 0 # in radians
		self.MAX_SPEED = 1
		self.speed = 0

	def scan_callback(self, data):
		self.check_obstacles(data)
		self.follow_wall(data)

	def check_obstacles(self, data):
		STRAIGHT_AHEAD = 0 # radians
		TOO_CLOSE = 1 # meter
		if distance_at(STRAIGHT_AHEAD, data) < TOO_CLOSE:
			print("obstacles found")
			self.speed = 0
		else:
			self.speed = self.MAX_SPEED
		# print("check for obstacles and disable driver if necessary")

	def wall_distance(self, data):
		if self.follow == RIGHT:
			angle = - math.pi / 2
		else:
			angle = math.pi / 2

		return distance_at(angle, data)

	def wall_angle(self, fan_angle, data):
		if self.follow == RIGHT:
			center_angle = - math.pi / 2
		else:
			center_angle = math.pi / 2

		a = distance_at(center_angle - fan_angle, data)
		b = distance_at(center_angle + fan_angle, data)
		d = distance_at(center_angle, data)

		delta = a - b

		return np.arcsin(delta / np.sqrt(delta * delta + d*d))

	def follow_wall(self, data):
		# positive angle turns: right

		# wall following constants
		filter_size = 141
		kp = 0.3
		kd = 2

		data.ranges = signal.medfilt(data.ranges, filter_size)

		distance_term = kp * (self.wall_distance(data) - self.target_dist)
		angle_term = kd * ( self.target_angle - self.wall_angle(0.25, data))

		# print("angle:", self.wall_angle(0.25, data))
		# print("distance:", self.wall_distance(data))
		# print("angle term", angle_term)
		# print("distance term", distance_term)

		control = angle_term + distance_term
		control = np.clip(control, -0.3, 0.3)

		control_msg = self.make_message("direct_drive")

		control_msg.drive_msg.speed = self.speed
		control_msg.drive_msg.steering_angle = control

		self.control_pub.publish(control_msg)

import os

TURN_DIR = RIGHT

if __name__ == '__main__':
	wf = WallFollower(True, RIGHT)

	def kill():
		print ("unsubscribe")
		wf.unsubscribe()

	rospy.on_shutdown(kill)
	rospy.spin()

