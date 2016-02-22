#!/usr/bin/env python
from __future__ import print_function
import rospy
from sensor_msgs.msg import LaserScan
from rospy.numpy_msg import numpy_msg
from scipy import signal
import math
import numpy as np
from scipy.signal import argrelextrema
import time


class Car(object):
	def __init__(self, simulate=False):
		self.config()
		self.node = rospy.init_node('icarus_main', anonymous=True)
		self.scan_subscriber = rospy.Subscriber(self.LASER_SCAN_TOPIC, numpy_msg(LaserScan), self.scan_callback)

		if simulate:
			self.motor_driver = SimulationDriver()
		else:
			self.motor_driver = MotorDriver()


	 
		

if __name__ == '__main__':
	rospy.init_node('wall_follower_main', anonymous=True)
	print ("Testx")
	rospy.spin()
	

