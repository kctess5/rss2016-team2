#!/usr/bin/env python
from __future__ import print_function
import rospy
from geometry_msgs.msg import Quaternion
from car_controller.control_module import ControlModule

def distance_at(angle, data):
	delta = angle - data.angle_min
	index = int (delta / data.angle_increment)
	return data.ranges[index]

class ConeFollower(ControlModule):
	def __init__(self, *args, **kwargs):
	        super(ConeFollower, self).__init__()

		self.CONE_POINTS_TOPIC = '/temp'
		#self.first_laser_recieved = False
		self.golden_tl_x = 5
		self.golden_tl_y = 6
		self.golden_height = 7
		self.golden_width = 8
		self.golden_center_x = self.golden_tl_x + (self.golden_width * 0.5)

		#self.scan_subscriber = rospy.Subscriber(self.LASER_SCAN_TOPIC, numpy_msg(LaserScan), self.scan_callback)
                self.conepos_sub = rospy.Subscriber(self.CONE_POS_TOPIC, Quaternion, self.conepos_callback)
                rospy.Timer(rospy.Duration(0.5), self.timeout) #Timeout if not seen new conepoints

	#def scan_callback(self, data):
	#	self.check_obstacles(data)
	#	self.follow_wall(data)

        def conepoints_callback(self, data):
                # Assume data is an array just like self.golden_points
                tl_x, tl_y, width, height = (data.x, data.y, data.z, data.w)
                center = tl_x + (width * 0.5)
                control_msg = self.make_message("direct_drive")
                speed = 0
                angle = 0
                if tl_x == float('nan'):
                    # Can't see the cone, so stop.
                    speed = 0
                else:
                    horizontal_offset = self.golden_center_x - center
                    height_offset = self.golden_height - height

                    #Speed based on height. Maybe make this nicer.
                    if height < self.golden_height * 0.5: #far away
                        speed = 1 #full speed!!
                    elif height < self.golden_height * 0.75: #getting close
                        speed = 0.5
                    elif height < self.golden_height * 0.9: #real close
                        speed = 0.25
                    else: # close enough!!
                        speed = 0
                        #TODO what if the height is good but the angle is wack?

                    #Angle based on x offset
                    MAX_OFFSET = 3. #TODO based on golden
                    proportion_off = min(MAX_OFFSET, abs(horizontal_offset))/MAX_OFFSET
                    if horizontal_offset < 0:
                        proportion_off *= -1
                    MAX_ANGLE = 0.3
                    angle = MAX_ANGLE * proportion_off

                    #Speed based on angle
                    #Goal: Speed will be cut in half if angle is at MAX_ANGLE
                    speed -= 0.5*speed*angle/MAX_ANGLE) #TODO is this right?????


                control_msg.drive_msg.speed = speed
                control_msg.drive_msg.angle = angle
                self.control_pub.publish(control_msg)

if __name__ == '__main__':
        cf = ConeFollower()
        def kill():
            print("unsubscribe")
            cf.unsubscribe()
        rospy.on_shutdown(kill)
	rospy.spin()

