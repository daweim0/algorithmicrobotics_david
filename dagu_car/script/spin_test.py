#!/usr/bin/env python

# Author: PCH 2017

"""
Description: Rotate left 360 deg and then right 360 deg.
"""

import rospy
from duckietown_msgs.msg import Twist2DStamped

if __name__ == '__main__':
    rospy.init_node('spin_test', anonymous=False)

    ang_vel = 2.0 # rad/s
    angle = 2.0*3.1415 # rad

    cmd_go = Twist2DStamped(v=0.0, omega=ang_vel)
    cmd_stop = Twist2DStamped(v=0.0, omega=0.0)
    pub = rospy.Publisher("/pi/dagu_car/vel_cmd", Twist2DStamped, queue_size=1)
    rospy.sleep(0.2)

    cmd_go.header.stamp = rospy.Time.now()
    pub.publish(cmd_go)
    rospy.sleep(angle/ang_vel)
    cmd_stop.header.stamp = rospy.Time.now()
    pub.publish(cmd_stop)

    cmd_go.omega = -ang_vel
    rospy.sleep(2.0)

    cmd_go.header.stamp = rospy.Time.now()
    pub.publish(cmd_go)
    rospy.sleep(angle/ang_vel)
    cmd_stop.header.stamp = rospy.Time.now()
    pub.publish(cmd_stop)
