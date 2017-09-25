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
        self.im = None
        
        #self.pub_raw_img = rospy.Publisher("~tags", HammingMarker, queue_size=1)
        rospy.Subscriber("~image_rect", Image, self.on_image, queue_size=1)


    def on_image(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        img[:,:,:] = 255.0*np.round(img[:,:,:]/(510.0*0.5))

        markers = detect_markers(img)
        print len(markers)

        for m in markers:
            m.draw_contour(img)
            print "%d: %s %s" % (m.id, m.center[0], m.center[1])
        self.im = img


if __name__ == '__main__': 
    rospy.init_node('tag_detector_node', anonymous=False)
    node = TagDetectorNode()
    while node.im is None:
        rospy.sleep(0.1)
    while not rospy.is_shutdown():
        cv2.imshow('live', node.im)
        cv2.waitKey(20)
    #rospy.spin()

