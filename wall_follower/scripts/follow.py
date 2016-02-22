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
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


# controls the drive state for both speed and steering
class MotorDriver:
	def __init__(self):
		self.motor_cmd_pub = rospy.Publisher("vesc/ackermann_cmd", AckermannDriveStamped)
		 # subscribe to joy topic
		self.joy_sub = rospy.Subscriber("vesc/joy", Joy, self.joyCallback)

		self.motor_speed = 0
		self.steering_angle = 0

		self.frequency = 0.05
		self.enabled = False
		self.last_toggle = time.clock()
		self.killed = False

		rospy.Timer(rospy.Duration(self.frequency), self.timer_callback)

	# constructs an AckermannDriveStamped message ready for publishing
	def make_drive_msg(self, speed=0, steering_angle=0, acceleration=0, jerk=0, steering_angle_velocity=0):
		drive_msg_stamped = AckermannDriveStamped()
		drive_msg = AckermannDrive()

		drive_msg.speed = speed
		drive_msg.acceleration = acceleration
		drive_msg.jerk = jerk
		drive_msg.steering_angle = steering_angle
		drive_msg.steering_angle_velocity = steering_angle_velocity

		drive_msg_stamped.drive = drive_msg
		return drive_msg_stamped

	def joyCallback(self, joy_msg):
		# debounced enable toggle
		if joy_msg.buttons[Y_BUTTON] == 1 and time.clock() - self.last_toggle > 0.5:
			self.enabled = not self.enabled
			self.last_toggle = time.clock()

	def set_speed(self, speed):
		self.motor_speed = speed

	def set_angle(self, angle):
		self.steering_angle = angle

	def timer_callback(self, event):
		if self.enabled:
			self.killed = False
			self.motor_cmd_pub.publish(self.make_drive_msg(self.motor_speed, -1 * self.steering_angle))
		elif not self.killed:
			self.motor_cmd_pub.publish(self.make_drive_msg(0,0))
			self.killed = True

class SimulationDriver():
	def __init__(self):
		print("Creating simulation driver")
		self.cmd_vel_publisher = rospy.Publisher('/racecar/ackermann_cmd_mux/input/teleop', 
			AckermannDriveStamped, queue_size=10)
		self.motor_speed = 0
		self.steering_angle = 0

		self.frequency = 0.1
		self.enabled = True

		rospy.Timer(rospy.Duration(self.frequency), self.timer_callback)

	# constructs a Ackermann Drive message ready for publishing
	def make_drive_msg(self, speed=0, steering_angle=0, acceleration=0, jerk=0, steering_angle_velocity=0):
		drive_msg_stamped = AckermannDriveStamped()
		drive_msg = AckermannDrive()

		drive_msg.speed = speed
		drive_msg.acceleration = acceleration
		drive_msg.jerk = jerk
		drive_msg.steering_angle = steering_angle
		drive_msg.steering_angle_velocity = steering_angle_velocity

		drive_msg_stamped.drive = drive_msg
		return drive_msg_stamped

	def set_speed(self, speed):
		self.motor_speed = speed

	def set_angle(self, angle):
		self.steering_angle = angle

	def timer_callback(self, event):
		if self.enabled:
			print("timer")
			self.killed = False
			self.cmd_vel_publisher.publish(self.make_drive_msg(self.motor_speed, self.steering_angle))
		elif not self.killed:
			self.cmd_vel_publisher.publish(self.make_drive_msg(0,0))
			self.killed = True

class Car(object):
	def __init__(self, simulate=False):
		self.simulate = simulate
		self.configure()
		
		self.node = rospy.init_node('follower_main', anonymous=True)
		self.scan_subscriber = rospy.Subscriber(self.LASER_SCAN_TOPIC, numpy_msg(LaserScan), self.scan_callback)

		if simulate:
			self.motor_driver = SimulationDriver()
		else:
			self.motor_driver = MotorDriver()

	def configure(self):
		self.LASER_SCAN_TOPIC = '/racecar/laser/scan'
		self.first_laser_recieved = False 

	def scan_callback(self, data):
		print("recieved scan callback")
	 
		

if __name__ == '__main__':
	car = Car(True)
	rospy.spin()
	

