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


LEFT = 1
RIGHT = 2

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
		self.laser_sub = rospy.Subscriber('/racecar/laser/scan', LaserScan, self.laserCallback)
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

        def laserCallback(self, laser_msg):
                straight_ahead = laser_msg.ranges[len(laser_msg.ranges)/2]
                if straight_ahead < 2:
                    self.enabled = False
                else:
                    self.enabled = True #WARNING anyone else setting enabled needs to coordinate

	def timer_callback(self, event):
		if self.enabled:
			self.killed = False
			self.cmd_vel_publisher.publish(self.make_drive_msg(self.motor_speed, self.steering_angle))
		elif not self.killed:
			self.cmd_vel_publisher.publish(self.make_drive_msg(0,0))
			self.killed = True

def distance_at(angle, data):
	delta = angle - data.angle_min
	index = int (delta / data.angle_increment)
	return data.ranges[index]

class Car(object):
	def __init__(self, simulate=False, follow=LEFT):
		self.simulate = simulate
		self.follow = follow

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
		self.target_dist = 1 # in meters
		self.target_angle = 0 # in radians

	def scan_callback(self, data):
		self.check_obstacles(data)
		self.follow_wall(data)

	def check_obstacles(self, data):
		pass
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
		filter_size = 5
		kp = 0.2
		kd = 1.5

		data.ranges = signal.medfilt(data.ranges, filter_size)

		distance_term = kp * (self.wall_distance(data) - self.target_dist)
		angle_term = kd * ( self.target_angle - self.wall_angle(0.25, data))

		# print("angle:", self.wall_angle(0.25, data))
		# print("distance:", self.wall_distance(data))
		# print("angle term", angle_term)
		# print("distance term", distance_term)

		control = angle_term + distance_term

		self.motor_driver.set_angle(control)
		# self.motor_driver.set_angle(0)
		self.motor_driver.set_speed(2)
		

if __name__ == '__main__':
	car = Car(True, RIGHT)
	rospy.spin()
	

