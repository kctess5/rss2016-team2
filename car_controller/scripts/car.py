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
from racecar.car_msgs import HighControl, DirectDrive
from car_msgs

''' The low level driver (LLDriver) classes handle low level racecar interaction
	for whatever runtime environments exist. For now this includes:
		- the real RACECAR
		- the gazebo simulation
	Each one should have set_speed() and set_angle() methods that drive the runtime
	environment accordingly.
'''

class LLDriver(object):
	""" A low level driver for some kind of mobile platform """
	def __init__(self, enabled):
		self.enabled = enabled

		self.motor_speed = 0
		self.steering_angle = 0

		self.motor_cmd_pub = None
		self.killed = False
		self.frequency = 0.05
		rospy.Timer(rospy.Duration(self.frequency), self.timer_callback)

	def disable(self):
		self.enabled = False

	def enable(self):
		self.enabled = True

	def toggle_enabled(self):
		self.enabled = not self.enabled

	def set_speed(self, speed):
		self.motor_speed = speed

	def set_angle(self, angle):
		self.steering_angle = angle

	def timer_callback(self):
		def timer_callback(self, event):
		if self.enabled:
			self.killed = False
			self.publish_drive_command()
		elif not self.killed:
			self.publish_drive_command(0, 0)
			self.killed = True	

class LLMotor(LLDriver):
	""" A low level motor and steering controller for the RACECAR platform """
	def __init__(self, *args, **kwargs):
		super(LLMotor, self).__init__(self, *args, **kwargs)

		self.motor_cmd_pub = rospy.Publisher("vesc/ackermann_cmd", AckermannDriveStamped)

	def publish_drive_command(self, speed=self.motor_speed, steering_angle=self.steering_angle, \
		acceleration=0, jerk=0, steering_angle_velocity=0):

		drive_msg_stamped = AckermannDriveStamped()
		drive_msg = AckermannDrive()

		drive_msg.speed = speed
		drive_msg.acceleration = acceleration
		drive_msg.jerk = jerk
		drive_msg.steering_angle = steering_angle
		drive_msg.steering_angle_velocity = steering_angle_velocity

		drive_msg_stamped.drive = drive_msg
		self.motor_cmd_pub.publish(drive_msg_stamped)
		return

class LLSimulator(LLDriver):
	""" A low level motor and steering controller for the Gazebo simulation platform """
	def __init__(self):
		self.motor_cmd_pub = rospy.Publisher('/racecar/ackermann_cmd_mux/input/teleop', 
			AckermannDriveStamped)

	# constructs a Ackermann Drive message ready for publishing
	def publish_drive_command(self, speed=self.motor_speed, steering_angle=self.steering_angle, \
		acceleration=0, jerk=0, steering_angle_velocity=0):

		drive_msg_stamped = AckermannDriveStamped()
		drive_msg = AckermannDrive()

		drive_msg.speed = speed
		drive_msg.acceleration = acceleration
		drive_msg.jerk = jerk
		drive_msg.steering_angle = steering_angle
		drive_msg.steering_angle_velocity = steering_angle_velocity

		drive_msg_stamped.drive = drive_msg
		self.motor_cmd_pub.publish(drive_msg_stamped)
		return

''' The high level driver (HLDriver) classes allow high level racecar interaction
	Agnostic of runtime environment. Each accepts a custom message from /racecar/car_msgs/
	and drives the low level controller accordingly.
'''

class HLDriver(object):
	def __init__(self, llDriver):
		self.llDriver = llDriver

	# pass through enable/disable commands to the low level driver
	def disable(self):
		self.llDriver.disable()
	def enable(self):
		self.llDriver.enable()
	def toggle_enabled(self):
		self.llDriver.toggle_enabled()

# accepts and executes a high level direct drive command
class DirectDrive(HLDriver):
	def execute(self, direct_drive_msg):
		self.llDriver.set_angle(direct_drive_msg.steering_angle)
		self.llDriver.set_speed(direct_drive_msg.speed)

# accepts and executes a high level spline following command
class SplineDrive(HLDriver):
	def execute(self, direct_drive_msg):
		# TODO: set up a timer callback, set the llDriver to the correct level at each given time
		pass

''' The ModuleManager manages and multiplexes all of the available RACECAR control modules
	Changes modes based off of the joypad input at runtime
'''

class ModuleManager(object):
	def __init__(self):
		env = rospy.get_param('/car/environment')

		self.joy_sub = rospy.Subscriber("vesc/joy", Joy, self.joyCallback)
		self.control_sub = rospy.Subscriber("/car/high_control", HighControl, self.joyCallback)

		self.enabled = False

		if env == 'simulation':
			self.llDriver = LLSimulator(enabled)
		elif env == 'racecar':
			self.llDriver = LLMotor(enabled)

		self.directDriver = DirectDrive(self.llDriver)
		self.directDriver = DirectDrive(self.llDriver)





LEFT = 1
RIGHT = 2

class Driver:
	def __init__(self):
		self.frequency = 0.05

		self.killed = False
		self.enabled = False

		self.last_toggle = time.clock()

		self.joy_sub = rospy.Subscriber("vesc/joy", Joy, self.joyCallback)
		rospy.Timer(rospy.Duration(self.frequency), self.timer_callback)

	def joyCallback(self, joy_msg):
		# debounced enable toggle
		if joy_msg.buttons[Y_BUTTON] == 1 and time.clock() - self.last_toggle > 0.5:
			self.enabled = not self.enabled
			self.last_toggle = time.clock()


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

	def disable(self):
		self.enabled = False

	def enable(self):
		self.enabled = True

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
		self.killed = False

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

	def disable(self):
		self.enabled = False

	def enable(self):
		self.enabled = True

	def timer_callback(self, event):
		if self.enabled:
			self.killed = False
			self.cmd_vel_publisher.publish(self.make_drive_msg(self.motor_speed, self.steering_angle))
		elif not self.killed:
			self.cmd_vel_publisher.publish(self.make_drive_msg(0,0))
			self.killed = True

class Car(object):
	def __init__(self, simulate=False):
		self.simulate = simulate
		self.configure()
		self.node = rospy.init_node('car_main', anonymous=True)

		self.direct_control_callback = rospy.Subscriber(self.DIRECT_CONTROL_TOPIC, numpy_msg(LaserScan), self.scan_callback)

		if simulate:
			self.motor_driver = SimulationDriver()
		else:
			self.motor_driver = MotorDriver()

	def configure(self):
		self.DIRECT_CONTROL_TOPIC = '/car/direct_control'

	def scan_callback(self, data):
		self.check_obstacles(data)
		self.follow_wall(data)

	def check_obstacles(self, data):
		STRAIGHT_AHEAD = 0 # radians
		TOO_CLOSE = 1 # meter
		if distance_at(STRAIGHT_AHEAD, data) < TOO_CLOSE:
			self.motor_driver.disable()
		else:
			self.motor_driver.enable() #WARNING anyone else setting enabled needs to coordinate
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
		kd = 2

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

import os

TURN_DIR = RIGHT

if __name__ == '__main__':
	if os.environ['ROS_ENV'] == 'simulation':
		print("Simulation environment")
		car = Car(True, TURN_DIR)
	elif os.environ['ROS_ENV'] == 'racecar':
		print("Racecar environment")
		car = Car(False, TURN_DIR)
	else:
		print("Unknown execution environment. Use racecar_env.sh or simulation_env.sh to specify.")

	rospy.spin()
	

