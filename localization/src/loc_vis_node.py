#!/usr/bin/env python

# Author: PCH

# Publish RViz markers with car pose (from localization)

import rospy
import numpy as np
from duckietown_msgs.msg import Pose2DStamped
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class LocVisNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        self.landmarks = rospy.get_param("~landmarks")

        # RViz car marker
        self.carmrkr = Marker()
        self.carmrkr.header.frame_id = "map"
        self.carmrkr.header.stamp = rospy.Time.now()
        self.carmrkr.ns = "pi"
        self.carmrkr.id = 0
        self.carmrkr.type = 0 # arrow
        self.carmrkr.action = 0
        self.carmrkr.pose.position = Point()
        self.carmrkr.pose.orientation.x = 0
        self.carmrkr.pose.orientation.y = 0
        self.carmrkr.pose.orientation.z = 0
        self.carmrkr.pose.orientation.w = 1.0
        self.carmrkr.scale.x = 0.1
        self.carmrkr.scale.y = 0.05
        self.carmrkr.scale.z = 0.05
        self.carmrkr.color.r = 1.0
        self.carmrkr.color.g = 0.0
        self.carmrkr.color.b = 0.0
        self.carmrkr.color.a = 1.0

        # RViz landmark markers
        self.landmrkr = Marker()
        self.landmrkr.header.frame_id = "map"
        self.landmrkr.header.stamp = rospy.Time.now()
        self.landmrkr.ns = "pi"
        self.landmrkr.id = 1
        self.landmrkr.type = 6 # cube list
        self.landmrkr.action = 0
        self.landmrkr.pose.position = Point()
        self.landmrkr.pose.orientation.x = 0
        self.landmrkr.pose.orientation.y = 0
        self.landmrkr.pose.orientation.z = 0
        self.landmrkr.pose.orientation.w = 1.0
        self.landmrkr.scale.x = 0.12
        self.landmrkr.scale.y = 0.12
        self.landmrkr.scale.z = 0.001
        self.landmrkr.color.r = 0.0
        self.landmrkr.color.g = 0.0
        self.landmrkr.color.b = 1.0
        self.landmrkr.color.a = 1.0
        for tid in self.landmarks:
            tag = self.landmarks[tid]
            self.landmrkr.points.append(Point(tag['x'], tag['y'], 0.0))

        self.pub_marker = rospy.Publisher('~carmrkr', Marker, queue_size=10)
        self.pub_map = rospy.Publisher('~landmrkr', Marker, queue_size=1, latch=True)
        rospy.Subscriber("~pose", Pose2DStamped, self.on_pose, queue_size=1)

        self.pub_map.publish(self.landmrkr)

    def on_pose(self, msg):
        # Update car pose
        self.carmrkr.header.stamp = msg.header.stamp
        self.carmrkr.pose.position = Point(msg.x, msg.y, 0.05)
        self.carmrkr.pose.orientation.z = np.sin(msg.theta/2.0)
        self.carmrkr.pose.orientation.w = np.cos(msg.theta/2.0)
        self.pub_marker.publish(self.carmrkr)

if __name__ == '__main__': 
    rospy.init_node('loc_vis_node', anonymous=False)
    node = LocVisNode()
    rospy.spin()

