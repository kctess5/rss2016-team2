#!/usr/bin/env python
from __future__ import print_function

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Polygon
from helpers import param
import cv2
import numpy
from helpers import param
from cv_bridge import CvBridge, CvBridgeError

#given row, this 4pl returns y
# A_y = 125.9304
# B_y = 1.333925
# C_y = 21.72133
# D_y = 7.336942
#A_y = 323.9
#B_y = .4321479
#C_y = 2.967
#D_y = -46.14
A_y,B_y,C_y,D_y = tuple([float(i) for i in param("greenCam.row2y")])
A_x,B_x,C_x,D_x = tuple([float(i) for i in param("greenCam.row2width")]) 


#given y, this 4PL returns how many pixels are in inch
# A_x = 89.24841
# B_x = 1.143443
# C_x = 16.35531
# D_x = -6.433097
# given row, how many inches are per pixel
#A_x = 11450.0
#B_x = .2874673
#C_x = 3.5e-16
#D_x = -.07852111

inch2meter 		 = .0254
#HSV_lower_thresh = (90,40,6)
HSV_lower_thresh = tuple([int(i) for i in param("greenCam.lower_thresh")])
HSV_upper_thresh = tuple([int(i) for i in param("greenCam.upper_thresh")])
#HSV_upper_thresh = (130,255,255)
#MIN_AREA_THRESH  = 200.
MIN_AREA_THRESH = float(param("greenCam.min_area_thresh"))

#row starts at 0 in center of ZED frame and increments when going down in the frame
#col starts at 0 left of ZED frame, x is 0 in middle of frame
#units are inches
def pixel2world(row,col,width):
	y = fourPL(A_y,B_y,C_y,D_y,row)*inch2meter
	x = fourPL(A_x,B_x,C_x,D_x,row)*(col-width/2.0)*inch2meter
	return (x,y)

def fourPL(A,B,C,D,x):
	return ((A-D)/(1.0+((x/C)**B))) + D

def find_green(image):
	height,width = image.shape[:2]
	roi = image[height/2:,:]
	hsv_img = cv2.cvtColor(roi, cv2.COLOR_RGB2HSV)

	cv2.imshow("colors",hsv_img)

	mask = cv2.inRange(hsv_img, HSV_lower_thresh, HSV_upper_thresh)
	mask = cv2.erode(mask, None, iterations = 2)
	mask = cv2.dilate(mask, None, iterations = 2)

	cv2.imshow("debug", mask)
	cv2.waitKey(1)	

	centroids = []
	pixels = []
	closest_y = float("inf")
	closest_centroid = [0.0, 0.0, 0.0]
	contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
	for c in contours:
		M = cv2.moments(c)
		px,py,area = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]), M["m00"])
		if area >= MIN_AREA_THRESH:
			x,y = pixel2world(py,px,width)
			centroids.append([x,y,0.0])
			pixels.append([px,py+height/2,0.0])
			if y < closest_y:
				closest_y = y
				closest_centroid = [x,y,0.0]
	return Point(*closest_centroid), centroids


class greenCam:
	def __init__(self):
		self.bridge = CvBridge()
		self.img = None
		self.sub = rospy.Subscriber('/camera/rgb/image_rect_color', Image, self.recv_image)
		#self.pub_points = rospy.Publisher('/waypoint_markers', Polygon)
		self.pub_green_goal = rospy.Publisher('/closest_green_goal', Point)
		rospy.init_node('greenCam')
	def recv_image(self, img):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(img, "rgb8")
			green_goal,waypoint_markers = find_green(cv_image)
			#self.pub_points.publish(waypoint_markers)
			self.pub_green_goal.publish(green_goal)
		except CvBridgeError as e:
    			print(e)


if __name__ == '__main__':
	try:
		greenCam()
		rospy.spin()
	except rospy.ROSInterruptException:
		print ("greenCam shutting down")

