#!/usr/bin/env python

# Author: Robert Katzschmann, Shih-Yuan Liu
# Modified by PCH 2017

"""
Description: Converts Twist2DStamped msgs (linear and angular car velocities)
to WheelSpeedsStamped msgs and vice versa. Car kinematic parameters and gains
can be set through a service.
"""

import rospy
from duckietown_msgs.msg import WheelSpeedsStamped, Twist2DStamped
from duckietown_msgs.srv import SetParam, SetParamResponse
from std_srvs.srv import Empty, EmptyResponse
import rospkg
import yaml
import time
import os.path

class KinematicsNode(object):
    def __init__(self):
        # Get node name and vehicle name
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]

        # Default parameters
        self.gain = 1.0
        self.trim = 0.0
        self.baseline = 0.1 # Distance between wheels (m)
        self.radius = 0.0318 # Wheel radius (m)
        self.k = 27.0
        self.limit = 1.0

        # Read parameters from yaml file
        self.read_param_file()

        # Prepare services
        rospy.Service("~set_param", SetParam, self.on_set_param)
        rospy.Service("~save_params", Empty, self.on_save_params)

        # Inverse kinematics
        rospy.Subscriber("~car_vel_in", Twist2DStamped, self.inverse_kinematics, queue_size=1)
        self.pub_wheels = rospy.Publisher("~wheel_speeds_out", WheelSpeedsStamped, queue_size=1)

        # Forward kinematics
        rospy.Subscriber("~wheel_speeds_in", WheelSpeedsStamped, self.forward_kinematics, queue_size=1)
        self.pub_vel = rospy.Publisher("~car_vel_out", Twist2DStamped, queue_size=1)

        rospy.loginfo("[%s] Initialized.", self.node_name)
        self.print_params()

    def read_param_file(self):
        """
        Read kinematic parameters from a yaml file.
        """
        file_name = self.get_config_path(self.veh_name)
        # Check file existence
        if not os.path.isfile(file_name):
            rospy.logwarn("[%s] %s does not exist. Not loading." % (
                self.node_name, file_name))
            return
        # Read yaml file
        with open(file_name, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
            except yaml.YAMLError as exc:
                rospy.logwarn("[%s] YAML syntax error. File: %s. Exc: %s" % (
                    self.node_name, file_name, exc))
                return
        if yaml_dict is None:
            return
        for name in ["gain", "trim", "baseline", "k", "radius", "limit"]:
            value = yaml_dict.get(name)
            if value is not None:
                self.set_param(name, value)

    def get_config_path(self, name):
        rospack = rospkg.RosPack()
        return rospack.get_path('duckietown')+'/config/dagu_car/kinematics/' + name + ".yaml"        

    def on_save_params(self, req):
        """
        Write kinematic parameters to a yaml file.
        """
        data = {
            "calibration_time": time.strftime("%Y-%m-%d-%H-%M-%S"),
            "gain": self.gain,
            "trim": self.trim,
            "baseline": self.baseline,
            "radius": self.radius,
            "k": self.k,
            "limit": self.limit,
        }

        # Write to file
        file_name = self.get_config_path(self.veh_name)
        with open(file_name, 'w') as out_file:
            out_file.write(yaml.dump(data, default_flow_style=False))
        # Printout
        self.print_params()
        rospy.loginfo("[%s] Saved to %s" %(self.node_name, file_name))
        return EmptyResponse()

    def on_set_param(self, req):
        """
        Service callback to set parameters.
        """
        success = self.set_param(req.name, req.value)
        if not success:
            rospy.logwarn("[%s] Unrecognized parameter %s or invalid value %s." % (
                self.node_name, name, value))
        else:
            self.print_params()
        return SetParamResponse(success)

    def print_params(self):
        rospy.loginfo("[%s] gain: %s trim: %s baseline: %s radius: %s k: %s limit: %s" % (
            self.node_name, self.gain, self.trim, self.baseline, self.radius, self.k, self.limit))

    def set_param(self, name, value):
        """
        Set a parameter of the car kinematics by name.
        """
        if (name == "gain"):
            self.gain = value
        elif (name == "trim"):
            self.trim = value
        elif (name == "baseline") and (0.0 < value):
            self.baseline = value
        elif (name == "radius") and (0.0 < value):
            self.radius = value
        elif (name == "k") and (0.0 < value):
            self.k = value
        elif (name == "limit") and (0.0 <= value) and (value <= 1.0):
            self.limit = value
        else:
            return False
        return True

    def forward_kinematics(self, msg_wheels):
        """
        Convert wheel speeds to car linear and angular velocities.
        """
        # Adjust k by gain and trim
        k_r_inv = (self.gain + self.trim) / self.k
        k_l_inv = (self.gain - self.trim) / self.k

        # Conversion from motor duty to rotation rates
        omega_r = msg_wheels.right / k_r_inv
        omega_l = msg_wheels.left / k_l_inv

        # Compute linear and angular velocities of car
        v = self.radius * ( omega_r + omega_l) / 2.0
        omega = self.radius * (omega_r - omega_l) / self.baseline

        # Put the car velocities into a message and publish
        msg_vel = Twist2DStamped()
        msg_vel.header = msg_wheels.header
        msg_vel.v = v
        msg_vel.omega = omega
        self.pub_vel.publish(msg_vel)

    def inverse_kinematics(self, msg_vel):
        """
        Convert car linear and angular velocities to wheel speeds.
        """
        # Adjust k by gain and trim
        k_r_inv = (self.gain + self.trim) / self.k
        k_l_inv = (self.gain - self.trim) / self.k
        
        # Compute motor rotation rates from linear and angular car velocities
        omega_r = (msg_vel.v + 0.5 * msg_vel.omega * self.baseline) / self.radius
        omega_l = (msg_vel.v - 0.5 * msg_vel.omega * self.baseline) / self.radius
        
        # Convert from motor rotation rates to duty
        u_r = omega_r * k_r_inv
        u_l = omega_l * k_l_inv

        # Limit output
        u_r_limited = max(min(u_r, self.limit), -self.limit)
        u_l_limited = max(min(u_l, self.limit), -self.limit)

        # Put the wheel speeds in a message and publish
        msg_wheels = WheelSpeedsStamped()
        msg_wheels.header.stamp = msg_vel.header.stamp
        msg_wheels.right = u_r_limited
        msg_wheels.left = u_l_limited
        self.pub_wheels.publish(msg_wheels)

if __name__ == '__main__':
    rospy.init_node('kinematics_node', anonymous=False)
    KinematicsNode()
    rospy.spin()
