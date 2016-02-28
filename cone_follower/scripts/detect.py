#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

def detect():
	rospy.init_node('detect', anonymous=True)
	print "Hello!"
	rospy.spin()

if __name__ == '__main__':
	try:
		detect()
	except rospy.ROSInterruptException:
		pass
