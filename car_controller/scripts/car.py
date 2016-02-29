#!/usr/bin/env python

from __future__ import print_function
import rospy
import time
from racecar.car_msgs import HighControl

''' Joystick constants
'''
-
A_BUTTON = 0
B_BUTTON = 1
X_BUTTON = 2
Y_BUTTON = 3


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
	def execute(self, control_msg):
		self.llDriver.set_angle(control_msg.drive_msg.steering_angle)
		self.llDriver.set_speed(control_msg.drive_msg.speed)

# accepts and executes a high level spline following command
class SplineDrive(HLDriver):
	def execute(self, control_msg):
		# TODO: set up a timer callback, set the llDriver to the correct level at each given time
		pass

''' The Car manages and multiplexes all of the available RACECAR control modules
	Changes modes based off of the joypad input at runtime
'''

class Car(object):
	def __init__(self):
		env = rospy.get_param('/car/environment')

		self.joy_sub = rospy.Subscriber("vesc/joy", Joy, self.joyCallback)
		self.control_sub = rospy.Subscriber("/car/high_control", HighControl, self.controlCallback)

		self.enabled = False
		self.last_toggle = time.clock()

		# keep track of all of the various high level modules that are contributing
		self.active_module = None
		self.modules = []

		if env == 'simulation':
			self.llDriver = LLSimulator(enabled)
		elif env == 'racecar':
			self.llDriver = LLMotor(enabled)

		control_map = {
			'direct_drive': DirectDrive(self.llDriver),
			'spline_control': SplineDrive(self.llDriver)
		}

	def joyCallback(self, joy_msg):
		# toggle the the active state of the low level motor driver

		if time.clock() - self.last_toggle > 0.15: # debounce controller buttons

			if joy_msg.buttons[Y_BUTTON] == 1:
				self.llDriver.toggle_enabled()
				
			if joy_msg.buttons[X_BUTTON] == 1:
				self.active_module = (self.active_module + 1) % len(self.modules)

			if joy_msg.buttons[B_BUTTON] == 1:
				self.active_module = (self.active_module - 1) % len(self.modules)

			if joy_msg.buttons[A_BUTTON] == 1:
				log("Active module: " + self.modules[self.active_module])

			self.last_toggle = time.clock()

	def controlCallback(self, control_msg):
		if (control_msg.subscribe):
			self.subscribe(control_msg)
		if (control_msg.unsubscribe):
			self.unsubscribe(control_msg)

		# pass the message through to the correct controller
		if control_msg.module == self.modules[self.active_module]:
			self.control_map[control_msg.msg_type].execute(control_msg)

	def subscribe(self, control_msg):
		# add the given control module from the registry
		if control_msg.module in self.modules:
			log("Module is already subscribed")
		else:
			self.modules.append(control_msg)

	def unsubscribe(self, control_msg):
		# remove the given control module from the registry

		if control_msg.module in self.modules:
			log("Removing module " + control_msg.module + " from the racecar.")
			self.modules.remove(control_msg.module)
		else:
			log("Module " + control_msg.module + " does not already exist in the racecar.")

if __name__ == '__main__':
	car = Car()
	rospy.spin()