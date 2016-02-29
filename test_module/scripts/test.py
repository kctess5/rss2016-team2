#!/usr/bin/env python
from __future__ import print_function
import rospy
from car_controller.control_module import ControlModule

class DriveModule(ControlModule):
	def __init__(self, *args, **kwargs):
		# initialize control module with name "test_module"
		super(DriveModule, self).__init__("test_module")

		rospy.Timer(rospy.Duration(0.05), self.timer_callback)

	def timer_callback(self, event):
		control_msg = self.make_message("direct_drive")

		control_msg.drive_msg.speed = 1
		control_msg.drive_msg.steering_angle = 1

		self.control_pub.publish(control_msg)

if __name__ == '__main__':
	dm = DriveModule()

	def kill():
		print ("unsubscribe")
		dm.unsubscribe()

	rospy.on_shutdown(kill)
	rospy.spin()
