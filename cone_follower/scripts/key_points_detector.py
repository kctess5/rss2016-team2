#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
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

		rospy.spin()

	def scan_callback(self, data):
	        filtered = self.filter(data)
                #last_distance = filtered[0][1]
                key_angles = []
                for i, (angle, distance) in enumerate(filtered[1:-1]):
		        last_distance = filtered[i-1][1]
		        next_distance = filtered[i+1][1]
                        if distance > last_distance and distance > next_distance:
                                key_angles.append((angle, distance))
                self.publish_1d_array(key_angles)

        def publish_1d_array(self, array):
                data = Float32MultiArray()
                data.data = array
                # Need to do something to data.layout?
                self.pub.Publish(data)


        def filter(self, data):
                """
                Input: data: LaserScan
                Output: List of (angle, distance) tuples after a median filter has been applied
                        Also, removes duplicate distances (keeping the middle angle at that distance)
                """
                filter_size = 141 # Where'd this come from?
                medfiltered_ranges = signal.medfilt(data.ranges, filter_size)

                all_distances_in_range = []
                angle = data.angle_min
                for distance in medfiltered_ranges:
                        if distance > data.range_min and distance < data.range_max:
                                all_distances_in_range.append((angle, distance))
                        angle += data.angle_increment

                output = []
                last_angle, last_distance = all_distances_in_range[0]
                in_same_distance_block = False
                first_angle_at_this_distance = last_angle #not needed unless in_same_distance_block is True
                for (angle, distance) in all_distances_in_range[1:]:
                        if not in_same_distance_block:
                                #Check if we've entered a same distance block
                                if last_distance == distance:
                                        #Assuming the medfilter got rid of small wiggles
                                        in_same_distance_block = True
                                        first_angle_at_this_distance = last_angle
                                        # Don't add anything to output for now...
                                else:
                                        output.append((angle, distance))
                        else: #in_same_distance_block
                            if last_distance != distance:
                                    #Ended same-distance block
                                    angle_to_report = (first_angle_at_this_distance + last_angle) / 2.
                                    #Modify the entry for the start of the block
                                    output[-1] = (angle_to_report, last_distance)
                                    #Add this new entry as normal
                                    output.append((angle, distance))
                            # else: continue...
                        last_angle, last_distance = angle, distance

                return output


if __name__ == '__main__':
	try:
		KeyPointsDetector()
	except rospy.ROSInterruptException:
		pass
