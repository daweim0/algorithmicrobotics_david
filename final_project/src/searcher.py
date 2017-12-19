#!/usr/bin/env python

import rospy
import numpy as np
from duckietown_msgs.msg import Twist2DStamped
from sensor_msgs.msg import Range
from duckietown_msgs.msg import Tag
import pdb

STANDOFF_DISTANCE = 25  # will turn if it gets closer than this to any wall
TURNING_STANDOFF_DISTANCE = STANDOFF_DISTANCE + 10  # will keep turning untill it is at least this far from a wall

SPEED_COEF = 1.0

TURNING_BEHAVIOR = "maxDist"

class searcher_node:
    def __init__(self):

        np.set_printoptions(precision=8)
        np.set_printoptions(suppress=True)

        self.TURN_ON_MOVEMENT = True

        self.last_dist = 50.0
        self.last_w = 0.0
        self.last_v = 0.0
        self.last_tag_id = None
        self.last_tag_point = None
        self.in_loop = False
        self.dist_recording_list = None

        self.tag_detected = False  # stop wandering and search for the tag if this is true

        rospy.Subscriber("~dist_in", Range, self.on_dist, queue_size=1)
        rospy.Subscriber("~tags_in", Tag, self.on_tag, queue_size=1)
        self.vel_pub = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        rospy.Timer(rospy.Duration(0.05), self.run_loop)

    def on_tag(self, msg):
        rospy.loginfo("got tag" + str(msg.id))
        self.last_tag_id = msg.id
        self.last_tag_point = msg.point
        self.tag_detected = True

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
        if self.tag_detected:
            # turn in circles looking for the tag again
            self.set_velocity(0, 0)
            self.tag_detected = False
            for i in range(15):
                self.set_velocity(0, -3)
                rospy.sleep(0.2)
                self.set_velocity(0, 0)
                rospy.sleep(2)  # there's a considerable delay between taking pictures and finding the tag
                if self.tag_detected == True:  # the duckie is pointing to the tag
                    break
            if self.tag_detected == True:  # the duckie is pointing to the tag
                self.home_torwards_tag()
            else:
                pass  # couldn't find the tag again :(
        else:
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
                    max_dist = np.min([np.max(np.asarray(self.dist_recording_list)), 70])
                    self.dist_recording_list = None
                    self.set_velocity(0, 3.5)
                    while self.last_dist < 0.8 * max_dist:
                        rospy.sleep(0.01)
                    self.set_velocity(0, 0)
            else:
                self.set_velocity(0.3, 0)
            self.in_loop = False


    def home_torwards_tag(self):
        centerline = -0.8
        while self.last_dist > 45:
            if self.last_tag_id is None:
                self.set_velocity(-2, 0)  # back up a little bit, maybe it will help
            else:
                if self.last_tag_point.y < centerline * 0.7:
                    self.set_velocity(2, 1)
                elif self.last_tag_point.y > centerline * 1.3:
                    self.set_velocity(2, -1)
            self.last_tag_id = None
            self.last_tag_point = None
            rospy.sleep(0.1)
            self.set_velocity(0, 0)
            rospy.sleep(4.0)  # time to take another picture of the ar tag
            if self.last_tag_id is None:
                rospy.sleep(2.0)  # wait a little longer to get a photo of the tag
        rospy.loginfo("arrived at tag")
        self.TURN_ON_MOVEMENT = False


    def set_velocity(self, v, w):
        if self.TURN_ON_MOVEMENT:
            twist_msg = Twist2DStamped()
            twist_msg.v = SPEED_COEF * v
            twist_msg.omega = SPEED_COEF * w
            self.vel_pub.publish(twist_msg)


if __name__ == "__main__":
    rospy.init_node('searcher_node', anonymous=False)
    node = searcher_node()
    rospy.spin()
