#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
import numpy as np

from __future__ import print_function

GREEN_GP_TOPIC = "/waypoint_markers"
CORRIDOR_GP_TOPIC = "/???"
GP_PUB_TOPIC = "/goal"

class GPManager:

	def __init__(self):
		self.green_sub = rospy.Subscriber(GREEN_GP_TOPIC, Point, self.cb_green)
		self.corridor_sub = rospy.Subscriber(CORRIDOR_GP_TOPIC, Point, self.cb_corridor)
		self.gp_pub = rospy.Publisher(GP_PUB_TOPIC, Point)

        self.green_gps = []
        self.corr_gp = None

    def cb_green(self, pts):

    def cb_corridor(self, pt):
        # Should only be 1 corridor navigation point at a time
        # TODO: To do Bayesian filtering, may need to keep track of past points
        self.corr_gp = pt

    def pub_gp(self):
        assert(self.corr_gp != None)

        # TODO: Change if we ever want to explore corridor before a green patch
        ordered_gps = self.green_gps + [self.corr_gp]

        target_x, target_y = ordered_gps[0]
        next_x, next_y = target_x, target_y if len(ordered_gps) < 2 \
            else ordered_gps[1]
        dx, dy = (next_x-target_x, next_y-target_y)

        # Publish target coordinates with direction of next goal point
        gp = Point(target_x, target_y, np.arctan(dy/dx))
        self.gp_pub.publish(gp)


if __name__ == '__main__':
	try:
		GPManager()
		rospy.spin()
	except rospy.ROSInterruptException:
		print ("Goal Point Manager shutting down")

