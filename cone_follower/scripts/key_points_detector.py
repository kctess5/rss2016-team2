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
		self.components_viz = rospy.Publisher('/cone/components_viz', Marker, queue_size=10)

                # Configuration
		self.filter_width = 11
		self.max_distance = 3.5 #meters
		self.max_angle = math.pi/3 #rad
		self.component_min_width = 4 #data points
		self.component_max_width = 0.25 # meters
		self.distance_breaking_component = 0.1 # meters

		rospy.spin()

	def scan_callback(self, data):
	        filtered = self.filter(data)
	        collapsed = self.collapse_points(filtered)
                components = self.components(filtered)
                self.publish_components_viz(components, data.header)
                key_angles = []
                for ((a1, d1), (a2, d2), (a3, d3)) in zip(collapsed, collapsed[1:], collapsed[2:]):
                        if d1 > d2 < d3 \
                                        and d2 < self.max_distance \
                                        and abs(a2) < self.max_angle \
                                        and self.angle_is_in_component(components, a2):
                                key_angles.append((a2, d2))
                self.publish_1d_array(key_angles, data.header)

        def angle_is_in_component(self, components, angle):
                for component in components:
                        start_angle, start_distance, end_angle, end_distance = component
                        if angle > start_angle and angle < end_angle:
                            components.remove(component) # sketchy evil bad practice
                            return True
                return False

        def publish_1d_array(self, array, header):
                data = PolarPoints()
                data.header = header
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

        def publish_components_viz(self, components, header):
                marker = Marker()
                marker.header = header
                marker.type = Marker.LINE_LIST
                marker.action = Marker.ADD
                marker.color.r = 1.
                marker.color.a = 1.
                marker.scale.x = 0.05
                for a1, d1, a2, d2 in components:
                        marker.points.append(self.make_point(a1, d1))
                        marker.points.append(self.make_point(a2, d2))
                self.components_viz.publish(marker)

        def make_point(self, angle, distance):
                """ Polar to Cartesian Point """
                return Point(x=distance*math.cos(angle), y=distance*math.sin(angle), z=0)


        def filter(self, data):
                """
                Input: data: LaserScan
                Output: List of (angle, distance) tuples after a median filter has been applied
                """
                medfiltered_ranges = signal.medfilt(data.ranges, self.filter_width)

                # Pass over angles not in range (as instructed by LaserScan documentation)
                all_distances_in_range = []
                angle = data.angle_min
                for distance in medfiltered_ranges:
                        if distance > data.range_min and distance < data.range_max:
                                all_distances_in_range.append((angle, distance))
                        angle += data.angle_increment

                return all_distances_in_range

        def collapse_points(self, all_distances_in_range):
                """
                Input: all_distances_in_range: List of (angle, distance) tuples
                Output: input list but collapsing entries at the same distance
                """
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

        def components(self, data):
                """
                Input: data: list of (angle, distance) points
                Output: List of (start_angle, start_distance, end_angle, end_distance)
                        connected components which are roughly between 2cm and 0.5m
                """
                components = []
                start_angle = data[0][0]
                distances = [data[0][1]]
                for ((aa, da), (ab, db)) in zip(data, data[1:]):
                        if abs(da - db) < self.distance_breaking_component:
                                # Continue component
                                distances.append(db)
                        else:
                                # Finish this component
                                #distance = sum(distances)/float(len(distances))
                                end_angle = aa
                                angle = abs(start_angle - end_angle)
                                leg1 = distances[0]
                                leg2 = distances[-1]
                                # Formula: a**2 = b**2 + c**2 - 2bc cosA
                                width = leg1**2 + leg2**2 - (2*leg1*leg2*math.cos(angle))
                                # if width > 0.01 and width < 1:
                                if len(distances) > self.component_min_width \
                                                    and width < self.component_max_width:
                                            components.append((start_angle, leg1, \
                                                            end_angle, leg2))

                                # Start new component
                                start_angle = ab
                                distances = [db]

                return components

if __name__ == '__main__':
	try:
		KeyPointsDetector()
	except rospy.ROSInterruptException:
		pass
