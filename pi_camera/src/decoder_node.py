#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage,Image
from duckietown_msgs.msg import BoolStamped
import time

class DecoderNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.enabled = True
        self.bridge = CvBridge()
        
        self.pub_raw_img = rospy.Publisher("~raw_image", Image, queue_size=1)
        rospy.Subscriber("~compressed_image", CompressedImage, self.on_image, queue_size=1)
        rospy.Subscriber("~enable", BoolStamped, self.on_enable, queue_size=1)

    def on_enable(self, msg):
        self.enabled = msg.data

    def on_image(self, msg):
        if not self.enabled:
            return
        np_arr = np.fromstring(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        msg_out = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        msg_out.header.stamp = msg.header.stamp
        msg_out.header.frame_id = msg.header.frame_id
        self.pub_raw_img.publish(msg_out)

if __name__ == '__main__': 
    rospy.init_node('decoder_node', anonymous=False)
    node = DecoderNode()
    rospy.spin()

