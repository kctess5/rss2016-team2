#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

class ConeDetector(object):
	def __init__(self):
		rospy.init_node('detect', anonymous=True)
		self.sub = rospy.Subscriber('/camera/rgb/image_rect_color', Image, self.recv_image)
		self.pub = rospy.Publisher('/cone', Image)

		rospy.spin()
	def recv_image(self, img):
		self.pub.publish(img)

if __name__ == '__main__':
	try:
		ConeDetector()
	except rospy.ROSInterruptException:
		pass
