#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import numpy as np
import cv2
from abc import ABCMeta

CONE_IMG_PATH = "..." # TODO

FLANN_PARAMS = {
	"algorithm" : 6, # FLANN_INDEX_LSH
        "table_number" : 6,
        "key_size" : 12,
        "multi_probe_level" : 1
}
FLANN_SEARCH_PARAMS = {
	"checks" : 50 # Tune this maybe?
}

MATCH_THRESHOLD = 5 # number of cone features to match
MAX_CONES = 4 # maximum number of cones (we care about) in a picture

class Detector:
	__metaclass__ = ABCMeta
	def __init__(self):
		self.detector = cv2.ORB()
		# self.detector = cv2.SIFT()
		if self.baseline == None:
			# TODO: throw error
			print "Detector must be initialized with a baseline image"
		self.init_matcher()

	def init_matcher(self):
		# TODO: Filter keypoints/descriptors so its only the cone
		# TODO: Use color images?
		self.base_kps, self.base_descs = \
			self.detector.detectAndCompute(self.baseline,None)
		self.matcher = cv2.FlannBasedMatcher(FLANN_INDEX_PARAMS,
						     FLANN_SEARCH_PARAMS)

	def match(self, img):
		kps, descs = self.detector.detectAndCompute(img,None)
		matches = flann.knnMatch(self.base_descs,descs,k=MAX_CONES+1)
		
		# Lowe's ratio test to find valid match points
		# TODO: Try a static threshold value instead
		#valid = []
		#for match_list in matches:
		#    if m.distance < 0.7 * n.distance:
		#	valid.append(m)
		#return valid

	@abstractmethod
	def recv_image(self, img):
		pass
	

class ConeDetector(Detector):
	def __init__(self):
		# Load baseline cone image
		self.baseline = cv2.imread(CONE_IMG_PATH)
		super(ConeDetector, self).__init__()
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
