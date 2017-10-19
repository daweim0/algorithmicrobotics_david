#!/usr/bin/env python

import math
import rospy
import rosparam
from duckietown_msgs.srv import SetParam
from duckietown_msgs.msg import SegmentList, Segment, BoolStamped, StopLineReading
from duckietown_msgs.msg import LanePose , Twist2DStamped
import time

# The recieve_stop_sign topic seem to send multiple messages each time a stop line is reached. supervisor_node will
# disregard any messages from the recieve_stop_sign topic that comes less than min_turn_cooldown after another message.
min_turn_cooldown = 0.05

# used when trying to straighten out in front of a stop sign
straighten_before_turn = False  # The straightening feature hurts more than it helps right now, so it's turned off.
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


		self.in_turn = False
		self.straightening = False
		
		self.last_input_pose = None
		self.last_turn_time = 0.0


	def recieve_stop_line(self, msg):
		# first decide whether this stop sign message is a duplicate (we already stopped at this stop sign)
		if msg.data == True and time.clock() - self.last_turn_time > min_turn_cooldown:

			#stop the duckiebot
			self.in_turn = True
			stop_msg = Twist2DStamped()
			stop_msg.v = 0
			stop_msg.omega = 0
			self.motor_pub.publish(stop_msg)

			# straighten out the duckiebot
			if straighten_before_turn:
				last_angle = 100
				start_time = time.clock()
				last_sign_positive = True
				I = 0
				while time.clock() - start_time < 0.0:
					while self.last_input_pose is None:
						pass
					last_angle = self.last_input_pose.phi
					I += last_angle
					self.last_input_pose = None

					# set the integral term to zero if the duckiebot is straight (to prevent overshooting)
					if abs(last_angle) < acceptable_turn_angle:
						I = 0.0

					turn_msg = Twist2DStamped()
					turn_msg.v = 0
					turn_msg.omega = last_angle * kpw + I * kIw
					print "turning command:", last_angle, I, turn_msg.omega
					self.motor_pub.publish(turn_msg)

				self.motor_pub.publish(stop_msg)

			# get user input
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

			# execute the maneuver: (which is a lsit of durations and twist2d messages)
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

		# stop intercepting messages going to the in-lane driving controller
		self.in_turn = False
		
	# intercept lane pose messages, copy them, and send them to the in-lane controller.
	def recieve_pose(self, pose):
		self.last_input_pose = pose
		# intercept data going to the in-lane driving controller if we're in a turn right now
		if not self.in_turn:
			self.controller_pub.publish(pose)
		
	# forward twist messages from the in-lane controller to the kinematics node
	def recieveTwist(self, msg):
		if not self.in_turn:
			self.motor_pub.publish(msg)


if __name__ == "__main__":
	rospy.init_node("lane_controller", anonymous=False)
	lane_controll_node()
	rospy.spin()


