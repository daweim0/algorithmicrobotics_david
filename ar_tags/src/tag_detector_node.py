#!/usr/bin/env python

# Author: PCH


import rospy
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import Image
from hampy import detect_markers

class TagDetectorNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.bridge = CvBridge()
        
        #self.pub_raw_img = rospy.Publisher("~tags", HammingMarker, queue_size=1)
        rospy.Subscriber("~image_rect", Image, self.on_image, queue_size=1)


    def on_image(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "mono8")
        markers = detect_markers(img)

        for m in markers:
            m.draw_contour(img)

            cv2.putText(img, str(m.id), tuple(int(p) for p in m.center),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)

        cv2.imshow('live', img)
        cv2.waitKey(20)


if __name__ == '__main__': 
    rospy.init_node('tag_detector_node', anonymous=False)
    node = TagDetectorNode()
    rospy.spin()

