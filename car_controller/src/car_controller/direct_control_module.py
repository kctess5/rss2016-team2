#!/usr/bin/env python
from __future__ import print_function
import rospy
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from sensor_msgs.msg import Joy

''' Joystick constants '''
A_BUTTON = 0
B_BUTTON = 1
X_BUTTON = 2
Y_BUTTON = 3

class DirectControlModule(object):
	def __init__(self, module_name, racecar_env=False):
		self.module_name = module_name
		self.node = rospy.init_node(module_name, anonymous=True)
		
		if racecar_env:
			topic = "/vesc/ackermann_cmd_mux/input/navigation"
			self.enabled_state = False
			self.coeff = 1.0
		else:
			topic = "/racecar/ackermann_cmd_mux/input/teleop"
			self.enabled_state = True
			self.coeff = -1.0

		self.motor_cmd_pub = rospy.Publisher(topic, AckermannDriveStamped, queue_size=1)
		

	# called when this module is enabled
	def enabled(self):
		pass

	# called when this module is disabled
	def disabled(self):
		pass

	def is_enabled(self):
		return self.enabled_state

	def toggle_enabled(self):
		self.enabled_state = not self.enabled_state
		if self.is_enabled():
			self.enabled()
		else:
			self.direct_set(speed=0, steering_angle=0)
			self.disabled()

	def joyCallback(self, joy_msg):
		# toggle the the active state of the low level motor driver
		if time.clock() - self.last_toggle > 0.02 and sum(joy_msg.buttons): # debounce controller buttons
			if joy_msg.buttons[Y_BUTTON] == 1:
				self.toggle_enabled()
				rospy.loginfo("Toggling module, active:", self.is_enabled())

			self.last_toggle = time.clock()

	def direct_set(self, speed=0, steering_angle=0):
		# direct pass through to the motor controller, in an attermpt
		# to reduce latency as much as possible
		
		if not self.is_enabled():
			return
		
		drive_msg_stamped = AckermannDriveStamped()
		drive_msg = AckermannDrive()

		drive_msg.speed = speed
		drive_msg.steering_angle = steering_angle * self.coeff
		drive_msg.acceleration = 0
		drive_msg.jerk = 0
		drive_msg.steering_angle_velocity = 0

		drive_msg_stamped.drive = drive_msg
		self.motor_cmd_pub.publish(drive_msg_stamped)
		
