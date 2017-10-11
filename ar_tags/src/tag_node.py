#!/usr/bin/env python

# Author: PCH

import rospy
import rospkg
import yaml
from geometry_msgs.msg import Point
from duckietown_msgs.msg import Tag
import numpy as np
import cv2
from hampy import detect_markers
from picamera.array import PiRGBArray
from picamera import PiCamera
import threading
import os.path

from cv_bridge import CvBridge      # DEBUG
from sensor_msgs.msg import Image   # DEBUG

class TagNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]

        self.pub_tags = rospy.Publisher("~tags", Tag, queue_size=1)

        # Parameters
        h = rospy.get_param("~homography")
        self.H = np.matrix([h[0:3], h[3:6], h[6:9]])
        self.framerate = rospy.get_param("~framerate",30) # Hz
        self.read_camera_info()

        # Setup PiCamera
        self.camera = PiCamera()
        self.camera.framerate = self.framerate
        self.camera.resolution = (self.width, self.height)
        self.stream = PiRGBArray(self.camera, size=(self.width, self.height))

        self.bridge = CvBridge()                                        # DEBUG
        self.frame_id = rospy.get_namespace() + "camera_optical_frame"  # DEBUG
        self.pub_img = rospy.Publisher("~image", Image, queue_size=1)   # DEBUG
        self.pub_tags = rospy.Publisher("~tags", Tag, queue_size=1)

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
            (self.width, self.height), cv2.CV_16SC2) # size=1?

        rospy.loginfo("[%s] Read calibration file %s" % (self.node_name, file_name))
 
    def capture(self):
        rospy.loginfo("[%s] Start capturing." % (self.node_name))

        for frame in self.camera.capture_continuous(self.stream, format="bgr", use_video_port=True):
            stamp = rospy.Time.now()

            img = cv2.remap(frame.array, self.map1, self.map2, cv2.INTER_LINEAR)
            # img = cv2.undistort(frame.array, self.K, self.D, self.R, self.P)

            grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            _, grey = cv2.threshold(grey, 127, 255, cv2.THRESH_BINARY)

            markers = detect_markers(grey)

            tag = Tag()
            tag.header.stamp = stamp
            for m in markers:
                m.draw_contour(img) # DEBUG
                tag.id = m.id
                tag.point = self.image2ground(m.center)
                self.pub_tags.publish(tag)

            image_msg= self.bridge.cv2_to_imgmsg(img)   # DEBUG
            image_msg.encoding = "bgr8"                 # DEBUG
            image_msg.header.stamp = stamp              # DEBUG
            image_msg.header.frame_id = self.frame_id   # DEBUG
            self.pub_img.publish(image_msg)             # DEBUG

            self.stream.truncate(0)

            if rospy.is_shutdown():
                break

        self.camera.close()
        rospy.loginfo("[%s] Capture ended." %(self.node_name))

    def image2ground(self, pixel):
        pt = self.H*np.matrix([[pixel[0]], [pixel[1]], [1]])
        point = Point()
        point.x = pt[0]/pt[2];
        point.y = pt[1]/pt[2];
        point.z = 0.0;
        return point

if __name__ == '__main__': 
    rospy.init_node('tag_node',anonymous=False)
    node = TagNode()
    # Node (PCH): thread is used so that node responds to shutdown signal.
    thread = threading.Thread(target=node.capture)
    thread.setDaemon(True)
    thread.start()
    rospy.spin()
