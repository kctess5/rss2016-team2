#!/usr/bin/env python
from __future__ import print_function

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped,Point
from helpers import param
import cv2
import numpy
from cv_bridge import CvBridge, CvBridgeError

#43090100, 7.9442, 32.0097, 2.50733
#line = m = .1462143 and -28.59338

A_y,B_y,C_y,D_y = tuple([float(i) for i in param("greenCam.row2y")])
#A_x,B_x,C_x,D_x = tuple([float(i) for i in param("greenCam.row2width")])
M_x, B_x = tuple([float(i) for i in param("greenCam.row2width")])


inch2meter 		 = .0254
HSV_lower_thresh = tuple([int(i) for i in param("greenCam.lower_thresh")])
HSV_upper_thresh = tuple([int(i) for i in param("greenCam.upper_thresh")])
MIN_AREA_THRESH = float(param("greenCam.min_area_thresh"))
DEBUG		= param("greenCam.debug")
HORIZON 	= int(param("greenCam.horizon_row"))

def create_lookup(height,width):
	pass

def debug_info(mask, img, hsv, pixels, vis = False):
	if vis == True:
		res = cv2.bitwise_and(img, img, mask=mask)
		imshow("maskedRoi", res)
	hsv_masked = cv2.bitwise_and(hsv,hsv,mask = mask)
	hsv_channels = cv2.split(hsv_masked)
	H_info = HminVal, HmaxVal, HminLoc, HmaxLoc = cv2.minMaxLoc(hsv_channels[0])
	S_info = SminVal, SmaxVal, SminLoc, SmaxLoc = cv2.minMaxLoc(hsv_channels[1])
	V_info = VminVal, VmaxVal, VminLoc, VmaxLoc = cv2.minMaxLoc(hsv_channels[2])
	print ("\n valid points:", len(pixels))
	print ("\n minVal, maxVal, minLoc, maxLoc \n")
	print ("H:", H_info)
	print ("S:", S_info)
	print ("V:", V_info)
	print ("____________________________________")
	return
	

def pixel2world(row,col,width):
	y = fourPL(A_y,B_y,C_y,D_y,row) # in meters
	#x = fourPL(A_x,B_x,C_x,D_x,row)*(col-width/2.0)*inch2meter
	x = (M_x*y + B_x)*(col-width/2.0)*inch2meter
	return (x,y)

def fourPL(A,B,C,D,x):
	return ((A-D)/(1.0+((x/C)**B))) + D

def find_green(image):
	height,width = image.shape[:2]
	roi = image[HORIZON/2:,:]
	hsv_img = cv2.cvtColor(roi, cv2.COLOR_RGB2HSV)

	mask = cv2.inRange(hsv_img, HSV_lower_thresh, HSV_upper_thresh)
	mask = cv2.erode(mask, None, iterations = 2)
	mask = cv2.dilate(mask, None, iterations = 2)

	centroids = []
	pixels = []
	closest_y = float("inf")
	closest_centroid = [0.0, 0.0, 0.0]
	contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
	for c in contours:
		M = cv2.moments(c)
		px,py,area = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]), M["m00"])
		if area >= MIN_AREA_THRESH:
			x,y = pixel2world(py+HORIZON,px,width)
			centroids.append([y,-1*x,0.0])
			pixels.append([px,py+HORIZON/2,0.0])
			if y < closest_y:
				closest_y = y
				closest_centroid = [y,-1*x,0.0]
	if DEBUG:
		debug_info(mask ,roi, hsv_img, pixels, vis=False)
	return Point(*closest_centroid), centroids


class greenCam:
	def __init__(self):
		self.bridge = CvBridge()
		self.img = None
		self.sub = rospy.Subscriber('/camera/rgb/image_rect_color', Image, self.recv_image)
		self.pub_green_goal = rospy.Publisher('/closest_green_goal', PointStamped)
		rospy.init_node('greenCam')
	def recv_image(self, img):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(img, "rgb8")
			green_goal = PointStamped()
			green_goal.point,waypoint_markers = find_green(cv_image)
			green_goal.header.stamp = img.header.stamp
			self.pub_green_goal.publish(green_goal)
		except CvBridgeError as e:
    			print(e)


if __name__ == '__main__':
	try:
		greenCam()
		rospy.spin()
	except rospy.ROSInterruptException:
		print ("greenCam shutting down")

