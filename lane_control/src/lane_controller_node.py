#!/usr/bin/env python

import math
import rospy
from duckietown_msgs.msg import LanePose, Twist2DStamped
from duckietown_msgs.srv import SetParam

enable=True

class lane_controll_node:
	def __init__(self):
		self.target_v = 0.25	
		self.target_d = -0.00
		self.target_phi = 0.00
		self.kh = -4.0
#		self.kh = 0.0
		self.khh = -0.0
		self.kd = -2.0
#		self.kd = 0.0

		self.d_integral = 0.5
		self.d_running_integral = 0.0
#		self.trim = -0.025
		self.trim = -0.175
		self.max_angle_deviation = 0.15
		self.acceptable_d = 0.08
		self.in_lane_d_scalar = 0.1
		
		self.prev_angle = 0.0
	
		# set up subscriber for lane pose
		rospy.Subscriber("~lane_pose", LanePose, self.recieve_pose, queue_size=1)

		# set up publisher for wheel commands
		self.pub = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
		
		# set up service to change kh
		rospy.Service("/set_k", SetParam, self.on_set_param)


	def recieve_pose(self, pose_msg):
		# filteringr

		if abs(pose_msg.d - self.target_d) > self.acceptable_d:
			pose_msg.d = math.copysign(pose_msg.d - self.target_d, self.acceptable_d) + self.target_d
		else:
			pose_msg.d = (pose_msg.d - self.target_d) * self.in_lane_d_scalar + self.target_d
		self.d_running_integral = self.d_integral*pose_msg.d + (1-self.d_integral) * self.d_running_integral
		offset = self.d_running_integral
		angle = pose_msg.phi
#		angle = self.prev_angle + math.copysign(min(math.fabs(angle - self.prev_angle), self.max_angle_deviation), angle - self.prev_angle)


		vx = self.target_v
		w = self.kh * (angle - self.target_phi) + self.kd * (offset - self.target_d)

		w += self.trim
		twist = Twist2DStamped()
		twist.v = vx
		twist.omega = w
		twist.header.stamp = pose_msg.header.stamp
	        if enable:
		    self.pub.publish(twist)
	
		self.prev_angle = angle

		self.print_data(offset, angle)


	def print_data(self, offset, angle):
		return
		offset_min = 0.3
		offset_max = -0.3
		angle_min = 1.3
		angle_max = -1.3
		display_length = 61

		offset_index = int(map_to(offset, offset_min, offset_max, 0, display_length))
		angle_index = int(map_to(angle, angle_min, angle_max, 0, display_length))

		offset_str = ""
		angle_str =  ""

		for i in range(display_length):
			if i == offset_index:
				offset_str = offset_str + "0"
			else:
				offset_str = offset_str + " "
                        if i == angle_index:
                                angle_str = angle_str + "A"
                        else:
                                angle_str = angle_str + " "
		offset_str = offset_str + ""
		angle_str = angle_str + ""

#		for i in range(1, abs(angle_deriv_index) + 1):
#			ind = int(angle_index + math.copysign(i, angle_deriv_index))
#			if 0 <= angle_index + ind and angle_index + ind < display_length:
#				angle_str = angle_str[:angle_index + ind] + "-" + angle_str[angle_index + ind+1:]
#		angle_str = angle_str[:angle_index + int(angle_deriv_index)] + "-" + angle_str[angle_index + int(angle_deriv_index)+1:]
#		angle_str = angle_str[:angle_index] + "A" + angle_str[angle_index +1:]


		print "offset: |" + offset_str + "|"
		print "angle:  |" + angle_str + "|"
		print "offset % 2.5f, angle: % 2.5f" % (offset, angle)
		print


	def on_set_param(self, req):
		if req.name == "target_v":
			self.target_v = req.value
		elif req.name == "kh":	
			self.kh = req.value
		elif req.name == "kd":	
			self.kd = req.value
		return

		
def map_to(x, in_min, in_max, out_min, out_max):
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

	
def min(a, b):
	if a > b:
		return b
	return a


if __name__ == "__main__":
	rospy.init_node("lane_controller", anonymous=False)
	lane_controll_node()
	rospy.spin()

	
	
	
	"""
	last used parameters:
		self.target_v = 0.25
		self.target_d = -0.00
		self.target_phi = 0.00
		self.kh = -4.0
	#		self.kh = 0.0
		self.khh = -0.1
		self.kd = -2.0
		self.kd = 0.0

		self.d_integral = 0.15
		self.d_running_integral = 0.0
		self.trim = 0.0
		self.max_angle_deviation = 0.1
		
		self.prev_angle = 0.0
	"""
