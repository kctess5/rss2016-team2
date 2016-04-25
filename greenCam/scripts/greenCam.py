#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import cv2
import numpy
from cv_bridge import CvBridge, CvBridgeError

from __future__ import print_function

C_y = 3.0 	# measured real world distance of calibration marker
C_r = 135. 	# measured row where calibration marker was located
C_w = .5	# measured real world width of calibration marker
C_c = 100.	# measured pixel width corresponding to calibration marker
C_a = 100.  # measured pixel area corresponding to calibration


HSV_lower_thresh = (29,86,6)
HSV_upper_thresh = (64,255,255)
MIN_AREA_THRESH  = 200.
area_error_thresh= .1

#row starts at 0 in center of ZED frame and increments when going down in the frame
#col starts at 0 in middle of ZED frame
def pixel2world(row,col):
	y = C_y*C_r/r
	x = C_w*C_r*c/(C_c*r)
	return (x,y)

def find_green(image):
	height = image.shape[0]
	roi = image[height/2:,:]
	hsv_img = cv2.cvtColor(roi, cv2.COLOR_RGB2HSV)
	mask = cv2.inRange(hsv_img, HSV_lower_thresh, HSV_upper_thresh)
	mask = cv2.erode(mask, None, iterations = 2)
	mask = cv2.dilate(mask, None, iterations = 2)

	centroids = []
	countours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
	for c in contours:
		M = cv2.moments(c)
		px,py,area = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]), M["m00"])
		if area >= MIN_AREA_THRESH:
			x,y = pixel2world(px,py)
			expected_area = ((C_y/(float)y)**2.)*C_a
			if (expected_area*(1.-area_error_thresh))<= area <=(expected_area*(1.+area_error_thresh)):
				centroids.append((x,y,0.0))
				pixels.append((px,py,0.0))
	return centroids, pixels


class greenCam:
	def __init__(self):
		self.bridge = CvBridge()
		self.img = None
		self.sub = rospy.Subscriber('/camera/rgb/image_rect_color', Image, self.recv_image)
		self.pub_points = rospy.Publisher('/waypoint_markers', Point) #TODO: make waypoint rostopic
		self.pub_pixels = rospy.Publisher('/pixel_centroids', Point) #TODO: make pixel_centroids rostopic

	def recv_image(self, img):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
			waypoint_markers, pixels = find_green(cv_image)
			self.pub_points.publish(waypoint_markers)
			self.pub_pixels.publish(pixels)
		except CvBridgeError as e:
    		print(e)


if __name__ == '__main__':
	try:
		greenCam()
		rospy.spin()
	except rospy.ROSInterruptException:
		print ("greenCam shutting down")

