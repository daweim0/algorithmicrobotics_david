#!/usr/bin/env python

# Author: PCH, David Michelman

import time
import rospy
import rospkg
import yaml
from geometry_msgs.msg import Point
from duckietown_msgs.msg import Tag
import numpy as np
import cv2
from hampy import detect_markers
import threading
import os.path
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as PIL_Image
import scipy.misc

import zxing  # barcode reading library


class TagDetectorNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]

        self.bridge = CvBridge()

        self.pub_tags = rospy.Publisher("~tags", Tag, queue_size=1)

	self.tag_scanner = zxing.BarCodeReader("/home/david/Documents/14_algorithmic_robotics/src/ar_tags/include/zxing")


        # Parameters
        h = rospy.get_param("~homography")
        self.H = np.matrix([h[0:3], h[3:6], h[6:9]])

        rospy.Subscriber("~image", Image, self.process_tag, queue_size=1, buff_size=2**24)


    def process_tag(self, image_message):
        print "startint to read image at time difference", (rospy.get_rostime() - image_message.header.stamp).to_sec()
        img = self.bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")

        # grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # _, grey = cv2.threshold(grey, 127, 255, cv2.THRESH_BINARY)

        # markers = detect_markers(grey)

	
        markers = []
        try:
            os.remove('/media/ramdisk/duckiebot/artag_image.jpg')
        except:
            pass
        scipy.misc.imsave('/media/ramdisk/duckiebot/artag_image.jpg', img)
        markers = self.tag_scanner.decode("/media/ramdisk/duckiebot/artag_image.png")
        # print "markers:", markers, "time delta:", (rospy.get_rostime() - image_message.header.stamp).to_sec()

        if markers is not None:
            tag = Tag()
            tag.header.stamp = image_message.header.stamp
            try:
                some_object_iterator = iter(markers)
            except TypeError, te:
                markers = [markers]
            for m in markers:
                # print "found AR tag", m.data, m.points
                tag.id = m.data
                # find the approximate center of the tag:
                tag.point = self.image2ground(np.asarray(m.points, dtype=np.float32).mean(axis=0))
                self.pub_tags.publish(tag)


    def get_config_path(self, name):
        rospack = rospkg.RosPack()
        return rospack.get_path('duckietown') + '/config/pi_camera/' + name + ".yaml"


    def read_camera_info(self):
        # Read camera calibration from a yaml file.
        file_name = self.get_config_path(self.veh_name)
        # Check file existence
        if not os.path.isfile(file_name):
            rospy.logwarn("[%s] %s does not exist. Using default calibration instead." % (self.node_name, file_name))
            file_name = self.get_config_path("default")
        if not os.path.isfile(file_name):
            rospy.logwarn("[%s] Default calibration not found." % (self.node_name))
            return None

        # Read yaml file
        with open(file_name, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
            except yaml.YAMLError as exc:
                rospy.logwarn("[%s] YAML syntax error. File: %s. Exc: %s" % (
                    self.node_name, file_name, exc))
                return None
        if yaml_dict is None:
            return None

        self.width = yaml_dict['image_width']
        self.height = yaml_dict['image_height']
        tmp = yaml_dict['distortion_coefficients']
        self.D = np.reshape(tmp['data'], (tmp['rows'], tmp['cols']))
        tmp = yaml_dict['camera_matrix']
        self.K = np.reshape(tmp['data'], (tmp['rows'], tmp['cols']))
        tmp = yaml_dict['rectification_matrix']
        self.R = np.reshape(tmp['data'], (tmp['rows'], tmp['cols']))
        tmp = yaml_dict['projection_matrix']
        self.P = np.reshape(tmp['data'], (tmp['rows'], tmp['cols']))
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            self.K, self.D, self.R, self.P,
            (self.width, self.height), cv2.CV_16SC2)  # size=1?

        rospy.loginfo("[%s] Read calibration file %s" % (self.node_name, file_name))


# begin coppied segment from line_detector_node.py
    def cbImage(self, image_msg):
        self.processImage(image_msg)
        # # stuff commented out to make debugging easier
        # if not self.active:
        #     return
        #     # Start a daemon thread to process the image
        # thread = threading.Thread(target=self.processImage, args=(image_msg,))
        # thread.setDaemon(True)
        # thread.start()
        # # Returns rightaway

    def processImage(self, image_msg):
        if not self.thread_lock.acquire(False):
            self.stats.skipped()
            # Return immediately if the thread is locked
            return

        try:
            self.processImage_(image_msg)
        finally:
            # Release the thread lock
            self.thread_lock.release()

    def processImage_(self, image_msg):
        # Decode from compressed image with OpenCV
        image_cv = cv2.imdecode(np.fromstring(image_msg.data, np.uint8), cv2.IMREAD_COLOR)
        if image_cv is None:
            self.loginfo('Could not decode image: %s' % e)
            return
        self.process_tag(image_cv)


    def image2ground(self, pixel):
        pt = self.H*np.matrix([[pixel[0]], [pixel[1]], [1]])
        point = Point()
        point.x = pt[0]/pt[2]
        point.y = pt[1]/pt[2]
        point.z = 0.0
        return point


if __name__ == '__main__':
    rospy.init_node('tag_detector', anonymous=False)
    node = TagDetectorNode()
    rospy.spin()
