#!/usr/bin/env python

from __future__ import print_function
import rospy
import time

from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from car_msgs.msg import HighControl
from sensor_msgs.msg import Joy
from std_msgs.msg import String

''' Joystick constants
'''

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
		rospy.loginfo("Initializing low level driver: %s  enabled: %r", self.__class__.__name__, enabled)
		self.enabled = enabled

		self.motor_speed = 0
		self.steering_angle = 0

		self.motor_cmd_pub = None
		self.killed = False
		self.frequency = 0.05
		rospy.Timer(rospy.Duration(self.frequency), self.timer_callback)

	def publish_drive_command(self, speed=0, angle=0):
		print ("Must be defined by super class!")

	def disable(self):
		rospy.loginfo("Disabling low level driver: %s", self.__class__.__name__)
		self.enabled = False

	def enable(self):
		rospy.loginfo("Disabling low level driver: %s", self.__class__.__name__)
		self.enabled = True

	def toggle_enabled(self):
		self.enabled = not self.enabled
		rospy.loginfo("Toggling low level driver: %s  enabled: %r", self.__class__.__name__, self.enabled)

	def set_speed(self, speed):
		self.motor_speed = speed

	def set_angle(self, angle):
		self.steering_angle = angle

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
		super(LLMotor, self).__init__(*args, **kwargs)

		self.motor_cmd_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=10)

	def publish_drive_command(self, speed=-999, steering_angle=-999, \
		acceleration=0, jerk=0, steering_angle_velocity=0):

		if speed == -999:
			speed = self.motor_speed
		if steering_angle == -999:
			steering_angle = self.steering_angle

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
	""" A low level motor and steering controller for the Gazebo simulation platform
        Negates all steering angles because the underyling simulator is backwards."""
	def __init__(self, *args, **kwargs):
		super(LLSimulator, self).__init__(*args, **kwargs)

		self.motor_cmd_pub = rospy.Publisher('/racecar/ackermann_cmd_mux/input/teleop', 
			AckermannDriveStamped, queue_size=10)

	# constructs a Ackermann Drive message ready for publishing
	def publish_drive_command(self, speed=-999, steering_angle=-999, \
		acceleration=0, jerk=0, steering_angle_velocity=0):

		if speed == -999:
			speed = self.motor_speed
		if steering_angle == -999:
			steering_angle = self.steering_angle

		drive_msg_stamped = AckermannDriveStamped()
		drive_msg = AckermannDrive()

		drive_msg.speed = speed
		drive_msg.acceleration = acceleration
		drive_msg.jerk = jerk
		# Negate the steering angle because the simulator is backwards.
		drive_msg.steering_angle = -steering_angle
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
		rospy.loginfo("Initializing high level driver: %s", self.__class__.__name__)
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
		self.node = rospy.init_node('car_main', anonymous=True)

		env = rospy.get_param('/car/environment')

		rospy.loginfo("Initializing high level RACECAR controller.")

		self.joy_sub = rospy.Subscriber("vesc/joy", Joy, self.joyCallback)
		self.control_sub = rospy.Subscriber("/car/high_control", HighControl, self.controlCallback)
		self.enabled_pub = rospy.Publisher("/car/enabled", String, queue_size=1)

		self.enabled = False
		self.last_toggle = time.clock()

		# keep track of all of the various high level modules that are contributing
		self.active_module = None
		self.modules = []

		if env == 'simulation':
			self.enabled = True
			self.llDriver = LLSimulator(self.enabled)
		elif env == 'racecar':
			self.llDriver = LLMotor(self.enabled)

		self.control_map = {
			'direct_drive': DirectDrive(self.llDriver),
			'spline_control': SplineDrive(self.llDriver)
		}

		rospy.on_shutdown(lambda: self.on_shutdown())

	def getActiveModule(self):
		if self.active_module == None:
			return None
		return self.modules[self.active_module]

	def publishEnabled(self, module, enabled):
		print("PUBLISHING_ENABLED", module, enabled)
		self.enabled_pub.publish(module + (":true" if enabled else ":false"))

	def joyCallback(self, joy_msg):
		# toggle the the active state of the low level motor driver
		if time.clock() - self.last_toggle > 0.02 and sum(joy_msg.buttons): # debounce controller buttons
		# if sum(joy_msg.buttons): # debounce controller buttons

			if joy_msg.buttons[Y_BUTTON] == 1:
				rospy.loginfo("Toggling module: %s", self.getActiveModule())
				self.llDriver.toggle_enabled()
				self.publishEnabled(self.getActiveModule(), self.llDriver.enabled)
				
			if joy_msg.buttons[X_BUTTON] == 1 and len(self.modules):
				self.active_module = (self.active_module + 1) % len(self.modules)
				rospy.loginfo("Set active module: %s", self.getActiveModule())

			if joy_msg.buttons[B_BUTTON] == 1 and len(self.modules):
				self.active_module = (self.active_module - 1) % len(self.modules)
				rospy.loginfo("Set active module: %s", self.getActiveModule())

			if joy_msg.buttons[A_BUTTON] == 1:
				rospy.loginfo("Active module: %s", self.getActiveModule())

			self.last_toggle = time.clock()

	def controlCallback(self, control_msg):
		if (control_msg.subscribe or not control_msg.module in self.modules):
			self.subscribe(control_msg)

		# pass the message through to the correct controller
		if control_msg.module == self.getActiveModule():
			rospy.logdebug("Reciving control message from module: %s", control_msg.module)
			self.control_map[control_msg.msg_type].execute(control_msg)

		if (control_msg.unsubscribe):
			self.unsubscribe(control_msg)

	def subscribe(self, control_msg):
		# add the given control module from the registry
		rospy.loginfo("Adding module %s to the high level controller", control_msg.module)
		if control_msg.module in self.modules:
			rospy.loginfo("Module %s is already subscribed", control_msg.module)
		else:
			self.modules.append(control_msg.module)

			if self.active_module == None:
				self.active_module = 0

		self.publishEnabled(self.getActiveModule(), self.llDriver.enabled)

	def unsubscribe(self, control_msg):
		# remove the given control module from the registry
		rospy.loginfo("Removing module %s to the high level controller", control_msg.module)
		if control_msg.module in self.modules:
			active = self.getActiveModule()
			self.modules.remove(control_msg.module)

			# special logic to reset active module
			if control_msg.module == active:
				if len(self.modules) == 0:
					self.active_module = None
				else:
					self.active_module = 0
		else:
			rospy.loginfo("Module %s does not already exist in the racecar.", control_msg.module)

	def on_shutdown(self):
		print("KILLING")
		self.publishEnabled(self.getActiveModule(), False)


if __name__ == '__main__':
	car = Car()
	rospy.spin()

