#!/usr/bin/env python

# TODO

import rospy
import numpy as np
from duckietown_msgs.msg import *
from geometry_msgs.msg import Point

class localization_node:
    def __init__(self):
	
		# subscribe to stuff
        rospy.Subscriber("~tags", Tag, self.on_tag, queue_size=1)
        rospy.Subscriber("~vel_executed", Twist2DStamped, self.on_vel, queue_size=1)
        self.pose_pub = rospy.Publisher("/pi/localization_node/pose", Pose2DStamped, queue_size=1)
        print "hi from localization_node"
		
    def on_tag(self, msg):
        print "saw tag", msg
	
    def on_vel(self, msg):
        print "got vel", msg8
	
	
if __name__ == "__main__":
    rospy.init_node('localization_node', anonymous = False)
    node = localization_node()
    rospy.spin()