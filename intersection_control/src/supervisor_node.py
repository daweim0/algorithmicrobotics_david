#!/usr/bin/env python

import math
import rospy
import rosparam
from duckietown_msgs.srv import SetParam
from duckietown_msgs.msg import SegmentList, Segment, BoolStamped, StopLineReading
from duckietown_msgs.msg import LanePose , Twist2DStamped
import time

# the at stop sign messages seem to come in pairs so it will try to do two turns 
# when there is only one stop sign. This makes it disregard the second stop sign message
min_turn_cooldown = 0.05
acceptable_turn_angle = 0.05
kpw = -0.125
kIw = -0.125/2


class lane_controll_node:
	def __init__(self):
		
		# set up subscriber for lane pose
		rospy.Subscriber("~lane_pose_in", LanePose, self.recieve_pose, queue_size=1)

		# set up subscriber for the stop line
		rospy.Subscriber("~stop_line_in", BoolStamped, self.recieve_stop_line, queue_size=1)
		
		# set up publisher to send lane pose to the lane controller
		self.controller_pub = rospy.Publisher("~lane_pose_out", LanePose, queue_size=1)
		
		# read twist2d from lane controller
		rospy.Subscriber("~twist2d_in", Twist2DStamped, self.recieveTwist, queue_size=1)
		
		# publish to ik node
		self.motor_pub = rospy.Publisher("~twist2d_out", Twist2DStamped, queue_size=1)
		
		# set up service to change kh
		rospy.Service("/set_k", SetParam, self.on_set_param)
		
		self.in_turn = False
		self.straightening = False
		
		self.last_input_pose = None
		self.last_turn_time = 0.0


	def recieve_stop_line(self, msg):
		if msg.data == True and time.clock() - self.last_turn_time > min_turn_cooldown:
			self.in_turn = True
			stop_msg = Twist2DStamped()
			stop_msg.v = 0
			stop_msg.omega = 0
			self.motor_pub.publish(stop_msg)
			
			last_angle = 100
			
			# straighten out the duckiebot
			start_time = time.clock()
			last_sign_positive = True
			I = 0
			while time.clock() - start_time < 0.0:
				while self.last_input_pose is None:
					pass
				last_angle = self.last_input_pose.phi
				I += last_angle
				self.last_input_pose = None
				
#				if last_angle > 0:
#					current_sign_positive = True
#				else:
#					current_sign_positive = False
#				if current_sign_positive != last_sign_positive:
#					I = 0.0
#				last_sign_positive = current_sign_positive

				if abs(last_angle) < acceptable_turn_angle:
					I = 0.0
				
				turn_msg = Twist2DStamped()
				turn_msg.v = 0
				turn_msg.omega = last_angle * kpw + I * kIw
				print "turning command:", last_angle, I, turn_msg.omega
				self.motor_pub.publish(turn_msg)
		
			self.motor_pub.publish(stop_msg)
			
			command = raw_input("command (f, l, r):")
			if command == "f":
				print "f"
				maneuver = rospy.get_param("~turn_forward")
			elif command == "l":
				print "l"
				maneuver = rospy.get_param("~turn_left")
			elif command == "r":
				print "r"
				maneuver = rospy.get_param("~turn_right")
			else:
				print "incorrect command"
				self.in_turn = False
				return
			print maneuver
						
			for command in maneuver:
				msg = Twist2DStamped()
				msg.v = command[1]
				msg.omega = command[2]
				
				start_time = time.clock()
				self.motor_pub.publish(msg)
				while time.clock() - start_time < command[0]:
					pass
			self.motor_pub.publish(stop_msg)
			self.last_turn_time = time.clock()
		
		self.in_turn = False
		

	def recieve_pose(self, pose):
		self.last_input_pose = pose
		if not self.in_turn:
			self.controller_pub.publish(pose)
		
		
	def recieveTwist(self, msg):
		if not self.in_turn:
			self.motor_pub.publish(msg)

		
	def on_set_param(self, req):
		if req.name == "target_v":
			self.target_v = req.value
		elif req.name == "kh":	
			self.kh = req.value
		elif req.name == "kd":	
			self.kd = req.value
		return


if __name__ == "__main__":
	rospy.init_node("lane_controller", anonymous=False)
	lane_controll_node()
	rospy.spin()


