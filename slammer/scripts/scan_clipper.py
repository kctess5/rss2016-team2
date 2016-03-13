#!/usr/bin/env python
"""
Node which clips LaserScan's to a certain range.
Reads LaserScan's from /scan and posts cleaned up ones to /scan_clipped.

This node should be replaced with a better solution. Modify the urg_node config
to limit the sweep the laser scanner does in the first place. That approach
would have 2 advantages:
- Maintain just one topic: /scan
- Possibly speed up laser scanning rate. Because the scanner could do less work.

Params:
- scan         (default: /scan)         The topic to subscribe to.
- scan_clipped (default: /scan_clipped) The topic to publish to.
"""
import rospy
from sensor_msgs.msg import LaserScan
from rospy.numpy_msg import numpy_msg
import math
import numpy as np
import copy

class ScanClipper(object):
    def __init__(self):
        rospy.init_node('scan_clipper', anonymous=True)
        self.sub_scan = rospy.Subscriber(rospy.get_param("~scan", "/scan"),
                                         numpy_msg(LaserScan), self.on_scan, queue_size=10)
        self.pub_scan = rospy.Publisher(rospy.get_param("~scan_clipped", "/scan_clipped"),
                                        numpy_msg(LaserScan), queue_size=10)

    def on_scan(self, msg):
        """Take the scan and make a copy with the bounds clipped."""
        # Calculate angles.
        angles = (np.arange(msg.ranges.shape[0]) * msg.angle_increment) + msg.angle_min

        msg2 = copy.copy(msg)
        msg2.ranges = msg2.ranges.copy()

        # Set angles out of bounds to nan.
        # These bounds were selected by watching scan data.
        # Any further out and the robot sees itself.
        # I suspect that it is so asymmetric because of the white wire sticking out of
        # the structure.io. I think that is the interference at ~85deg.
        min_angle = np.deg2rad(-115)
        max_angle = np.deg2rad(80)
        msg2.ranges[np.where((angles < min_angle) | (angles > max_angle))] = msg2.range_max + 1

        # Publish the same message (modified in place).
        self.pub_scan.publish(msg2)

if __name__ == '__main__':
    try:
        ScanClipper()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
