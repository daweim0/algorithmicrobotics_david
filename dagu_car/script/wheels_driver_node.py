#!/usr/bin/env python

# Modified by PCH 2017

"""
Description: Listens for WheelSpeedsStamped msgs, sets the motor inputs
accordingly, and publishes WheelSpeedsStamped msgs to announce when it
sets the motors.
"""

import rospy
from duckietown_msgs.msg import WheelSpeedsStamped
from dagu_wheels_driver import DaguWheelsDriver

class WheelsDriverNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        # Wheels driver object
        self.driver = DaguWheelsDriver()
        # Reuse message for sending
        self.msg_out = WheelSpeedsStamped()
        # Publisher for executed commands
        self.pub_wheels = rospy.Publisher("~wheels_cmd_executed", WheelSpeedsStamped, queue_size=1)
        # Listen for new commands
        rospy.Subscriber("~wheels_cmd", WheelSpeedsStamped, self.on_wheels_cmd, queue_size=1)

        rospy.loginfo("[%s] Initialized.", self.node_name)

    def on_wheels_cmd(self, msg):
        self.driver.set_wheel_speeds(right=msg.right, left=msg.left)
        # Put the wheel commands in a message and publish
        self.msg_out.header = msg.header
        # Record the time the command was given to the wheels_driver
        self.msg_out.header.stamp = rospy.get_rostime()  
        self.msg_out.left = msg.left
        self.msg_out.right = msg.right
        self.pub_wheels.publish(self.msg_out)

    def on_shutdown(self):
        self.driver.set_wheel_speeds(right=0.0, left=0.0)
        rospy.loginfo("[%s] Shutting down." % self.node_name)

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('wheels_driver_node', anonymous=False)
    # Create the object
    node = WheelsDriverNode()
    # Specify proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    # Keep the node alive
    rospy.spin()
