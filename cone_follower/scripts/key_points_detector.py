#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from cone_follower.msg import PolarPoints, PolarPoint
from rospy.numpy_msg import numpy_msg
from scipy import signal
import math
import numpy as np

"""
Uses Laser scan data to publish points of local minima (possible cone)
"""
class KeyPointsDetector(object):
	def __init__(self):
		rospy.init_node('key_points_detector', anonymous=True)
		LASER_SCAN_TOPIC = "/scan"
		self.sub = rospy.Subscriber(LASER_SCAN_TOPIC, numpy_msg(LaserScan), self.scan_callback)
		self.pub = rospy.Publisher('/cone/key_points', PolarPoints, queue_size=10)
		self.pub_viz = rospy.Publisher('/cone/key_points_viz', Marker, queue_size=10)

                # Configuration
		self.filter_width = 11
		self.max_distance = 3.5 #meters
		self.max_angle = math.pi/3 #rad

		rospy.spin()

	def scan_callback(self, data):
	        filtered = self.filter(data)
                key_angles = []
                for ((a1, d1), (a2, d2), (a3, d3)) in zip(filtered, filtered[1:], filtered[2:]):
                        if d1 > d2 < d3 and d2 < self.max_distance and abs(a2) < self.max_angle:
                                key_angles.append((a2, d2))
                self.publish_1d_array(key_angles, data.header)

        def publish_1d_array(self, array, header):
                data = PolarPoints()
                data.points = [PolarPoint(angle=angle, distance=distance) for angle, distance in array]
                self.pub.publish(data)

                # publish points to rviz
                marker = Marker()
                marker.header = header
                marker.type = Marker.POINTS
                marker.action = Marker.ADD
                marker.color.b = 1.
                marker.color.a = 1.
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.points = [self.make_point(angle, distance) for angle, distance in array]
                self.pub_viz.publish(marker)

        def make_point(self, angle, distance):
                """ Polar to Cartesian Point """
                return Point(x=distance*math.cos(angle), y=distance*math.sin(angle), z=0)


        def filter(self, data):
                """
                Input: data: LaserScan
                Output: List of (angle, distance) tuples after a median filter has been applied
                        Also, removes duplicate distances (keeping the middle angle at that distance)
                """
                medfiltered_ranges = signal.medfilt(data.ranges, self.filter_width)

                # Pass over angles not in range (as instructed by LaserScan documentation)
                all_distances_in_range = []
                angle = data.angle_min
                for distance in medfiltered_ranges:
                        if distance > data.range_min and distance < data.range_max:
                                all_distances_in_range.append((angle, distance))
                        angle += data.angle_increment

                output = []

                # Collapse blocks of same-distance points to a single point
                # To make local-minima detection easy
                start_angle = None
                for ((aa, da), (ab, db)) in zip(all_distances_in_range, all_distances_in_range[1:]):
                        if abs(da - db) < 0.00001:
                                # Points are basically the same
                                if start_angle == None:
                                        # Record this as the start_angle
                                        start_angle = aa
                        else:
                                # Points are different enough for their own entry
                                if start_angle == None:
                                        # We aren't bunching points together
                                        output.append((aa, da))
                                else:
                                        angle = (start_angle + aa)/2.
                                        output.append((angle, da))
                                        start_angle = None

                output.append(all_distances_in_range[-1])

                return output


if __name__ == '__main__':
	try:
		KeyPointsDetector()
	except rospy.ROSInterruptException:
		pass
