#!/usr/bin/env python
from __future__ import print_function
import rospy
from car_msgs.msg import HighControl, DirectDrive
from std_msgs.msg import String

class ControlModule(object):
	def __init__(self, module_name, publish_topic="/car/high_control"):
		self.module_name = module_name
		self.node = rospy.init_node(module_name, anonymous=True)
		self.control_pub = rospy.Publisher(publish_topic, HighControl, queue_size=10)
		self.enabled_sub = rospy.Subscriber('/car/enabled', String, self._enabled_callback)
		self.enabled_state = False

		rospy.sleep(0.1)
		self.subscribe()

	def _enabled_callback(self, which):
		data = which.data.split(":")
		if data[0] == self.module_name:
			if data[1] == "true":
				self.enabled_state = True
				self.enabled()
			elif data[1] == "false":
				self.enabled_state = False
				self.disabled()
			else:
				print("UNKNOWN STATE:", which)
				self.enabled_state

	# called when this module is enabled
	def enabled(self):
		pass

	# called when this module is disabled
	def disabled(self):
		pass

	def is_enabled(self):
		return self.enabled_state

	def make_message(self, msg_type):
		control_msg = HighControl()

		control_msg.module = self.module_name
		control_msg.msg_type = msg_type

		drive_msg = DirectDrive()

		drive_msg.speed = 0
		drive_msg.steering_angle = 0

		control_msg.drive_msg = drive_msg

		return control_msg

	def subscribe(self):
		control_msg = self.make_message("direct_drive")
		control_msg.subscribe = True
		self.control_pub.publish(control_msg)

	def unsubscribe(self):
		control_msg = self.make_message("direct_drive")
		control_msg.unsubscribe = True
		self.control_pub.publish(control_msg)
