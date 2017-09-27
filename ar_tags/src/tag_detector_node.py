#!/usr/bin/env python

# Author: PCH


import rospy
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from duckietown_msgs.msg import Tag
from hampy import detect_markers

class TagDetectorNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.bridge = CvBridge()

        h = rospy.get_param("~homography")
        self.H = np.matrix([h[0:3], h[3:6], h[6:9]])

        self.pub_tags = rospy.Publisher("~tags", Tag, queue_size=1)
        rospy.Subscriber("~image_rect", Image, self.on_image, queue_size=1)


    def on_image(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, img = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)

        markers = detect_markers(img)

        tag = Tag()
        tag.header.stamp = msg.header.stamp
        for m in markers:
            tag.id = m.id
            tag.point = self.image2ground(m.center)
            self.pub_tags.publish(tag)

    def image2ground(self, pixel):
        pt = self.H*np.matrix([[pixel[0]], [pixel[1]], [1]])
        point = Point()
        point.x = pt[0]/pt[2];
        point.y = pt[1]/pt[2];
        point.z = 0.0;
        return point

if __name__ == '__main__': 
    rospy.init_node('tag_detector_node', anonymous=False)
    node = TagDetectorNode()
    rospy.spin()
