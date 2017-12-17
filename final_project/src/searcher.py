#!/usr/bin/env python

# TODO

import rospy
import numpy as np
from duckietown_msgs.msg import *
# from geometry_msgs.msg import Point
# import numdifftools as nd
import pdb
import time


class Event():
    def __init__(self):
        self.last_real = 0.0
        self.current_real = 0.1


class searcher_node:
    def __init__(self):
    
        np.set_printoptions(precision=8)
        np.set_printoptions(suppress=True)
    
        self.last_w = 0
        self.last_v = 0

        # subscribe to stuff, pretty standard
        # rospy.Subscriber("~tags", Tag, self.on_tag, queue_size=1)
        # rospy.Subscriber("~vel_executed", Twist2DStamped, self.on_vel, queue_size=1)
        self.pose_pub = rospy.Publisher("/pi/pose", Pose2DStamped, queue_size=1)
        
        self.mu_k = np.mat('0; 0; 0', dtype=np.float64)
        self.u_k = np.mat('0; 0', dtype=np.float64)
        self.P_k = np.mat('0, 0, 0; 0, 0, 0; 0, 0, 0', dtype=np.float64)
        self.last_tag = None
        self.in_EKF_calculation = False
#        pdb.set_trace()
#        mu_k, P_k = EKF(mu_k1=np.mat('0; 0; 0', dtype=np.float64), P_k1=P_k1, u_k=np.mat('1; 0', dtype=np.float64), f=calculate_state_update, h=None)
#        print mu_k, P_k
        self.last_output = None
        # self.all_tags = rospy.get_param('/pi/localization_node/landmarks')

        rospy.Timer(rospy.Duration(0.1), self.run_EKF)
        
		
    def on_tag(self, msg):
        self.last_tag = msg
	
    def on_vel(self, msg):
        self.last_w = msg.omega
        self.last_v = msg.v

    def run_EKF(self, time):
        pass

	
if __name__ == "__main__":
    rospy.init_node('searcher_node', anonymous = False)
    node = searcher_node()
    rospy.spin()
