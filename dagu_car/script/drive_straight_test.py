#!/usr/bin/env python

# Author: PCH 2017

"""
Description: Drive straight the distance of 1 mat.
"""

import rospy
from duckietown_msgs.msg import Twist2DStamped

if __name__ == '__main__':
    rospy.init_node('drive_straight_test', anonymous=False)

    vel = 0.38 # m/s
    mat_len = 23.0 + 5.0/8.0 # inch
    dist = 1*mat_len/39.3701 # m


    cmd_go = Twist2DStamped(v=vel, omega=0.0)
    cmd_stop = Twist2DStamped(v=0.0, omega=0.0)
    pub = rospy.Publisher("/pi/dagu_car/vel_cmd", Twist2DStamped, queue_size=1)
    rospy.sleep(0.2)

    cmd_go.header.stamp = rospy.Time.now()
    pub.publish(cmd_go)
    rospy.sleep(1.2*dist/vel) # fudge factor
    cmd_stop.header.stamp = rospy.Time.now()
    pub.publish(cmd_stop)
