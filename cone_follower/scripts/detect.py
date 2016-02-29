#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ConeDetector(object):
	def __init__(self):
		rospy.init_node('detect', anonymous=True)
		self.sub = rospy.Subscriber('/camera/rgb/image_rect_color', Image, self.recv_image)
		self.pub = rospy.Publisher('/cone', Image)
		self.bridge = CvBridge()

		rospy.spin()
	def recv_image(self, img):
		self.pub.publish(img)

	def ros_to_cvimg(self, rosimg):
	        ENC_GRAYSCALE = "mono8"
	        ENC_COLOR = "bgr8"
                return self.bridge.toCvShare(rosimg, ENC_GRAYSCALE)

if __name__ == '__main__':
	try:
		ConeDetector()
	except rospy.ROSInterruptException:
		pass
