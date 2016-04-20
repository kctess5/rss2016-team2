#!/usr/bin/env python
from __future__ import print_function
import rospy
from car_controller.control_module import ControlModule
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time

class DriveModule(ControlModule):
	def __init__(self, *args, **kwargs):
		# initialize control module with name "test_module"
		super(DriveModule, self).__init__("test_module")
		self.frames = 0.0
		self.starttime = time.time()
		self.started = False
		image_sub = rospy.Subscriber('/camera/rgb/image_rect_color', Image, self.img_callback)
		self.bridge = CvBridge()

	def ros_to_cvimg(self, rosimg):
		ENC_GRAYSCALE = "mono8"
		ENC_COLOR = "bgr8"
		# return self.bridge.toCvShare(rosimg, ENC_GRAYSCALE)
		return self.bridge.imgmsg_to_cv2(rosimg, desired_encoding=ENC_COLOR)

	def start(self):
		self.starttime = time.time()
		self.started = True

	def framerate(self):
		return self.frames / (time.time()-self.starttime)

	def img_callback(self, img):
		if not self.started:
			self.start()
		image = self.ros_to_cvimg(img)
		self.frames += 1.0
		print("IMG", np.sum(image), self.framerate())
		pass
		# rospy.Timer(rospy.Duration(0.05), self.timer_callback)

if __name__ == '__main__':
	dm = DriveModule()

	def kill():
		print ("unsubscribe")
		dm.unsubscribe()

	rospy.on_shutdown(kill)
	rospy.spin()






# #!/usr/bin/env python
# from __future__ import print_function
# import rospy
# from car_controller.control_module import ControlModule
# from sensor_msgs.msg import Joy
# import time

# A_BUTTON = 0

# class DriveModule(ControlModule):
# 	def __init__(self, *args, **kwargs):
# 		# initialize control module with name "test_module"
# 		super(DriveModule, self).__init__("test_module")

# 		rospy.Timer(rospy.Duration(0.05), self.timer_callback)
# 		self.last_toggle = time.clock()

# 		self.angle = 0.13
# 		# self.angle_increment = 0.03

# 		self.speed = 0.0
# 		self.max_accel = 0.1
# 		# self.speed_target = 0.0
# 		# self.steering_angle = -.27

# 		self.joy_sub = rospy.Subscriber("vesc/joy", Joy, self.joyCallback)
# 		self.record("\n NEW SCRIPT RUNNING!")

# 	def record(self, string):
# 		with open("log.txt", "a") as f:
# 			f.write(string + "\n")

# 	def joyCallback(self, joy_msg):
# 		if time.clock() - self.last_toggle > 0.01 and sum(joy_msg.buttons): # debounce controller buttons
# 			if joy_msg.buttons[A_BUTTON] == 1:
# 				self.speed += self.max_accel
# 				self.record("Changed drive state to:"+ str((self.speed, self.angle)) + "at t=" + str(time.time()))
# 				pass # increase speed
# 			self.last_toggle = time.clock()

# 	# def cur_time(self):
# 		# return time.strftime(%a, )

# 	def enabled(self):
# 		self.record("Enable toggled at t="+ str(time.time()))
# 		self.speed = 0.1
	
# 	def max_speed(self, angle):
# 		return -115.37*angle*angle*angle+107.42*angle*angle-34.535*angle+5.8628

# 	def timer_callback(self, event):
# 		control_msg = self.make_message("direct_drive")
# 		# print(self.angle, self.speed)

# 		# accel = 0
# 		# if not self.speed_target == self.speed:
# 		# 	accel = self.speed_target - self.speed
# 		# 	if accel > 0:
# 		# 		accel = min(accel, self.max_accel)
# 		# 	elif accel < 0:
# 		# 		accel = max(accel, -1*self.max_accel)
		
# 		# self.speed = self.speed + accel

# 		# self.angle = self.angle + anglular_accel
# 		# self.speed_target = self.max_speed(self.angle)
# 		# self.steering_angle = self.angle

# 		control_msg.drive_msg.speed = self.speed
# 		control_msg.drive_msg.steering_angle = self.angle

# 		self.control_pub.publish(control_msg)

# if __name__ == '__main__':
# 	dm = DriveModule()

# 	def kill():
# 		print ("unsubscribe")
# 		dm.unsubscribe()

# 	rospy.on_shutdown(kill)
# 	rospy.spin()




# #!/usr/bin/env python
# from __future__ import print_function
# import rospy
# from car_controller.control_module import ControlModule
# from sensor_msgs.msg import Joy

# class DriveModule(ControlModule):
# 	def __init__(self, *args, **kwargs):
# 		# initialize control module with name "test_module"
# 		super(DriveModule, self).__init__("test_module")

# 		rospy.Timer(rospy.Duration(0.05), self.timer_callback)

# 		self.speed = 0.0
# 		self.max_accel = 0.15
# 		self.speed_target = 0.0
# 		# self.steering_angle = -.27

# 		self.max_anglular_accel = 0.0003
# 		self.min_angle = 0.1
# 		self.max_angle = 0.3
# 		self.angle = self.min_angle

# 		self.joy_sub = rospy.Subscriber("vesc/joy", Joy, self.joyCallback)

# 	def joyCallback(self, joy_msg):
# 		if time.clock() - self.last_toggle > 0.01 and sum(joy_msg.buttons): # debounce controller buttons
# 			if joy_msg.buttons[A_BUTTON] == 1:
# 				pass # increase speed

# 	def enabled(self):
# 		print("ENABLED")
# 		self.speed = 0.1
# 		self.angle = self.min_angle
	
# 	def max_speed(self, angle):
# 		return -115.37*angle*angle*angle+107.42*angle*angle-34.535*angle+5.8628

# 	def timer_callback(self, event):
# 		control_msg = self.make_message("direct_drive")
# 		print(self.angle, self.speed)

# 		accel = 0
# 		if not self.speed_target == self.speed:
# 			accel = self.speed_target - self.speed
# 			if accel > 0:
# 				accel = min(accel, self.max_accel)
# 			elif accel < 0:
# 				accel = max(accel, -1*self.max_accel)
		
# 		self.speed = self.speed + accel

# 		anglular_accel = 0
# 		if self.max_angle > self.angle:
# 			anglular_accel = self.max_angle - self.angle
# 			if anglular_accel > 0:
# 				anglular_accel = min(anglular_accel, self.max_anglular_accel)
# 			elif anglular_accel < 0:
# 				anglular_accel = max(anglular_accel, -1*self.max_anglular_accel)
		
# 		self.angle = self.angle + anglular_accel
# 		self.speed_target = self.max_speed(self.angle)
# 		self.steering_angle = self.angle

# 		control_msg.drive_msg.speed = self.speed
# 		control_msg.drive_msg.steering_angle = self.steering_angle

# 		self.control_pub.publish(control_msg)

# if __name__ == '__main__':
# 	dm = DriveModule()

# 	def kill():
# 		print ("unsubscribe")
# 		dm.unsubscribe()

# 	rospy.on_shutdown(kill)
# 	rospy.spin()
