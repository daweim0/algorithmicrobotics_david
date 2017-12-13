#!/usr/bin/env python

# TODO

import rospy
import numpy as np
from duckietown_msgs.msg import *
from geometry_msgs.msg import Point
import numdifftools as nd
import pdb
import time


class Event():
    def __init__(self):
        self.last_real = 0.0
        self.current_real = 0.1


class localization_node:
    def __init__(self):
    
        np.set_printoptions(precision=8)
        np.set_printoptions(suppress=True)
    
        self.last_w = 0
        self.last_v = 0
	
		# subscribe to stuff, pretty standard
        rospy.Subscriber("~tags", Tag, self.on_tag, queue_size=1)
        rospy.Subscriber("~vel_executed", Twist2DStamped, self.on_vel, queue_size=1)
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
        self.all_tags = rospy.get_param('/pi/localization_node/landmarks')

        rospy.Timer(rospy.Duration(0.1), self.run_EKF)
        
		
    def on_tag(self, msg):
        self.last_tag = msg
	
    def on_vel(self, msg):
        self.last_w = msg.omega
        self.last_v = msg.v

    
    
    def run_EKF(self, event):
        if self.in_EKF_calculation:
            return
        self.in_EKF_calculation = True
    
        # time since the EKF was last run
        if event.last_real is not None:
            try:
                duration_object = event.current_real - event.last_real
                dt = duration_object.secs + duration_object.nsecs * 10 ** -9
            except:
                dt = event.current_real - event.last_real
        else:
            dt = 0.1
            
        if self.last_tag is not None:
            tag = self.last_tag
            self.last_tag = None
            h = lambda pos: self.calulate_sensor_input(pos, tag)
        else:
            h = None
            tag = None
        
        mu_k_new, self.P_k = EKF(mu_k1=self.mu_k, P_k1=self.P_k, u_k=np.mat([[self.last_v], [self.last_w]]), f=calculate_state_update, h=h, dt=dt, tag=tag, all_tags=self.all_tags)
        print mu_k_new, '\n', self.P_k, '\n'
        self.mu_k = mu_k_new
        
        pose_msg = Pose2DStamped()
        pose_msg.x = self.mu_k[0, 0]
        pose_msg.y = self.mu_k[1, 0]
        pose_msg.theta = self.mu_k[2, 0]
        self.pose_pub.publish(pose_msg)
        self.in_EKF_calculation = False
    
    
    def calulate_sensor_input(self, mu_k, tag):
    #    pdb.set_trace()
        tags = self.all_tags
        tag_x = tags[str(tag.id)]["x"]
        tag_y = tags[str(tag.id)]["y"]
        out = np.mat('0; 0; 0', dtype=np.float64)
        out[0, 0] = np.cos(mu_k[2, 0]) * (tag_x - mu_k[0, 0]) + np.sin(mu_k[2, 0]) * (tag_y - mu_k[1, 0])
        out[1, 0] = -1 * np.sin(mu_k[2, 0]) * (tag_x - mu_k[0, 0]) + np.cos(mu_k[2, 0]) * (tag_y - mu_k[1, 0])
        out[2, 0] = mu_k[2, 0]
        return out
    
    
# returns new state from last state and input
def calculate_state_update(u_k, mu_k1, input_noise, dt):
    if input_noise is None:
        input_noise = np.mat('0; 0; 0', dtype=np.float64)
    mu_k = np.mat('0; 0; 0', dtype=np.float64)
    mu_k[0, 0] = mu_k1[0, 0] + np.cos(mu_k1[2, 0]) * u_k[0, 0] * (input_noise[0, 0] + 1) * dt
    mu_k[1, 0] = mu_k1[1, 0] + np.sin(mu_k1[2, 0]) * u_k[0, 0] * (input_noise[1, 0] + 1) * dt
    mu_k[2, 0] = mu_k1[2, 0] + u_k[1, 0] * (input_noise[2, 0] + 1) * dt
    return mu_k

    
def EKF(mu_k1=None, P_k1=None, u_k=None, f=None, h=None, dt=0.1, tag=None, all_tags=None):    
    # jacobians
    f_fun = lambda mu: f(u_k, mu, None, dt)
    F_k1 = nd.Jacobian(f_fun)(mu_k1)
    
    # other stuff
    Q = np.eye(3)/10    # noise estimate
    
    w_fun = lambda noise: f(u_k, mu_k1, noise, dt)    # jacobian of f with respect to noise
    W_k = nd.Jacobian(w_fun)(np.mat('0; 0; 0', dtype=np.float64))
    
    # predict
    bar_mu_k = f(u_k, mu_k1, None, dt)    # predicted state estimate
    bar_P_k = F_k1 * P_k1 * F_k1.T + W_k * Q * W_k.T    # predicted covariance estimate
    
    if h is None:
        return bar_mu_k, bar_P_k
    
#    pdb.set_trace()
    
    H = nd.Jacobian(h)(bar_mu_k)    # jacobian of sensor input?
    
    # update
    R = np.eye(3) * 0.01
    K = bar_P_k * H.T * (H * bar_P_k * H.T + R).I
    P_k = (np.eye(3) - K * H) * P_k1
    z = np.mat('0; 0; 0', dtype=np.float64)
#    tags = rospy.get_param('/pi/localization_node/landmarks')
    z[0, 0] = tag.point.x
    z[1, 0] = tag.point.y
    h_bar = h(bar_mu_k)
    epsilon = z - h_bar
    mu_k = bar_mu_k + K * epsilon  # updated state estimate
    
    return mu_k, P_k
        
	
if __name__ == "__main__":
    rospy.init_node('localization_node', anonymous = False)
    node = localization_node()
    rospy.spin()