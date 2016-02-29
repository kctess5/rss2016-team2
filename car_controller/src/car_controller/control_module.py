#!/usr/bin/env python
from __future__ import print_function
import rospy
from car_msgs.msg import HighControl, DirectDrive

class ControlModule(object):
	def __init__(self, module_name, publish_topic="/car/high_control"):
		self.module_name = module_name
		self.node = rospy.init_node(module_name, anonymous=True)
		self.control_pub = rospy.Publisher(publish_topic, HighControl, queue_size=10)

		rospy.sleep(0.1)
		self.subscribe()

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
