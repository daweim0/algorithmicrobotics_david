#!/usr/bin/env python

import rospy
import numpy as np
from duckietown_msgs.msg import Twist2DStamped
from sensor_msgs.msg import Range
import pdb

STANDOFF_DISTANCE = 25  # will turn if it gets closer than this to any wall
TURNING_STANDOFF_DISTANCE = STANDOFF_DISTANCE + 10  # will keep turning untill it is at least this far from a wall

TURNING_BEHAVIOR = "maxDist"

class searcher_node:
    def __init__(self):

        np.set_printoptions(precision=8)
        np.set_printoptions(suppress=True)

        self.last_dist = 50.0
        self.last_w = 0.0
        self.last_v = 0.0
        self.last_tag = None
        self.in_loop = False
        self.dist_recording_list = None

        rospy.Subscriber("~dist_in", Range, self.on_dist, queue_size=1)
        self.vel_pub = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        rospy.Timer(rospy.Duration(0.05), self.run_loop)

    def on_tag(self, msg):
        rospy.loginfo("got tag")
        self.last_tag = msg

    def on_vel(self, msg):
        rospy.loginfo("got vel")
        self.last_w = msg.omega
        self.last_v = msg.v

    def on_dist(self, msg):
        self.last_dist = msg.range
        if self.dist_recording_list is not None:
            self.dist_recording_list.append(msg.range)

    def run_loop(self, time_msg):
        if self.in_loop:
            return
        self.in_loop = True
        if self.last_dist < STANDOFF_DISTANCE:  # (in centimeters)
            if TURNING_BEHAVIOR == "random":
                self.set_velocity(-0.2, 0)  # get a little away from the wall
                rospy.sleep(0.25)
                self.set_velocity(0, 4.0)
                while self.last_dist < TURNING_STANDOFF_DISTANCE:
                    pass
                rospy.sleep(0.1)
                self.set_velocity(0, 0)
            elif TURNING_BEHAVIOR == "maxDist":
                self.set_velocity(-0.2, 0)  # get a little away from the wall
                rospy.sleep(0.15)
                self.set_velocity(0, 0)
                self.dist_recording_list = list()
                self.set_velocity(0, 3.5)
                rospy.sleep(2.0)  # this should be enough time for the duckiebot to go in a full circle
                self.set_velocity(0, 0)
                max_dist = np.max(np.asarray(self.dist_recording_list))
                self.dist_recording_list = None
                self.set_velocity(0, 3.5)
                while self.last_dist < 0.8 * max_dist:
                    rospy.sleep(0.01)
                self.set_velocity(0, 0)
        else:
            self.set_velocity(0.3, 0)
        self.in_loop = False

    def set_velocity(self, v, w):
        twist_msg = Twist2DStamped()
        twist_msg.v = v
        twist_msg.omega = w
        self.vel_pub.publish(twist_msg)


if __name__ == "__main__":
    rospy.init_node('searcher_node', anonymous=False)
    node = searcher_node()
    rospy.spin()
