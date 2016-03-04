#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
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
		self.pub = rospy.Publisher('/cone/key_points', Float32MultiArray)
		self.pub_viz = rospy.Publisher('/cone/key_points_viz', Marker)
		self.pub_medfilter = rospy.Publisher('/cone/scan_filtered', numpy_msg(LaserScan))

		rospy.spin()

	def scan_callback(self, data):
	        #rospy.loginfo("Scan callback")
	        filtered = self.filter(data)
                #last_distance = filtered[0][1]
                key_angles = []
                #for i, (angle, distance) in enumerate(filtered[1:-1]):
                for ((a1, d1), (a2, d2), (a3, d3)) in zip(filtered, filtered[1:], filtered[2:]):
		        #last_distance = filtered[i-1][1]
		        #next_distance = filtered[i+1][1]
                        #rospy.loginfo("{} {} {}".format(d1, d2, d3))
                        if d1 > d2 < d3 and d2 < 2.5:
                                #rospy.loginfo("It is a key point")
                                key_angles.append((a2, d2))
                self.publish_1d_array(key_angles, data.header)

        def publish_1d_array(self, array, header):
                data = Float32MultiArray()
                data.data = [angle for angle, distance in array]
                # Need to do something to data.layout?
                self.pub.publish(data)

                #remove_old_markers = Marker()
                #remove_old_markers.header = header
                #remove_old_markers.action = Marker.DELETEALL
                #self.pub_viz.publish(remove_old_markers)

                marker = Marker()
                marker.header = header
                marker.type = Marker.POINTS
                marker.action = Marker.ADD
                marker.color.b = 1.
                marker.color.a = 1.
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                #marker.lifetime = rospy.Duration(secs=0, nsecs=10**8)
                marker.points = [self.make_point(angle, distance) for angle, distance in array]
                self.pub_viz.publish(marker)

        def make_point(self, angle, distance):
                return Point(x=distance*math.cos(angle), y=distance*math.sin(angle), z=0)


        def filter(self, data):
                """
                Input: data: LaserScan
                Output: List of (angle, distance) tuples after a median filter has been applied
                        Also, removes duplicate distances (keeping the middle angle at that distance)
                """
                filter_size = 21 # Where'd this come from?
                medfiltered_ranges = signal.medfilt(data.ranges, filter_size)
                data.ranges = medfiltered_ranges
                all_distances_in_range = []
                angle = data.angle_min
                for distance in medfiltered_ranges:
                        if distance > data.range_min and distance < data.range_max:
                                all_distances_in_range.append((angle, distance))
                        angle += data.angle_increment

                output = []

                """last_angle, last_distance = all_distances_in_range[0]
                output = [(last_angle, last_distance)]
                in_same_distance_block = False
                first_angle_at_this_distance = last_angle #not needed unless in_same_distance_block is True
                for (angle, distance) in all_distances_in_range[1:]:
                        #rospy.loginfo("(" + str(angle) + ", " + str(distance) + ")")
                        if not in_same_distance_block:
                                #Check if we've entered a same distance block
                                #ros.loginfo("Comparing " + str(last_distance) + " to " + str(distance))
                                if abs(last_distance - distance) < 0.00001:
                                        #Assuming the medfilter got rid of small wiggles
                                        #rospy.loginfo("ENETR SAME DISTANCE BLOCK")
                                        in_same_distance_block = True
                                        first_angle_at_this_distance = last_angle
                                        # Don't add anything to output for now...
                                else:
                                        output.append((angle, distance))
                        else: #in_same_distance_block
                                if last_distance != distance:
                                        #rospy.loginfo("END SAME DISTANCE BLOCK")
                                        in_same_distance_block = False
                                        #Ended same-distance block
                                        angle_to_report = (first_angle_at_this_distance + last_angle) / 2.
                                        #Modify the entry for the start of the block
                                        output[-1] = (angle_to_report, last_distance)
                                        #Add this new entry as normal
                                        output.append((angle, distance))
                                # else: continue...
                        last_angle, last_distance = angle, distance
                """
                for ((aa, da), (ab, db)) in zip(all_distances_in_range, all_distances_in_range[1:]):
                        if abs(da - db) < 0.00001:
                                #rospy.loginfo("Skipping " + str(da))
                                pass
                        else:
                                #rospy.loginfo(str(da))
                                output.append((aa, da))
                output.append(all_distances_in_range[-1])

                #rospy.loginfo("Filter output: " + str(output))
                return output


if __name__ == '__main__':
	try:
		KeyPointsDetector()
	except rospy.ROSInterruptException:
		pass
