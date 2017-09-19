#!/usr/bin/env python

# Modified by PCH 2017

"""
Description: Publishes camera calibration info every time an images is
received. New calibration parameters can be saved to file through a service.

Publications
    ~camera_info (CameraInfo)

Subscriptions
    ~compressed_image (CompressedImage)

Services (advertised)
    ~set_camera_info (SetCameraInfo)
"""

import rospy
import rospkg
import yaml
from sensor_msgs.msg import CameraInfo, CompressedImage
from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
import os.path

class CamInfoNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]

        # Load and print camera info message
        self.camera_info_msg = self.read_camera_info()
        if self.camera_info_msg is None:
            rospy.signal_shutdown("Failed to load calibration. Aborting.")
        self.camera_info_msg.header.frame_id = rospy.get_namespace() + "camera_optical_frame"

        # Publish info on camera_info each time an image is received on compressed_image
        self.pub_cam_info = rospy.Publisher("~camera_info", CameraInfo, queue_size=1)
        rospy.Subscriber("~compressed_image", CompressedImage, self.on_image, queue_size=1)
    
        # Create service to save camera calibration
        rospy.Service("~set_camera_info", SetCameraInfo, self.on_save_camera_info)

        rospy.loginfo("[%s] Initialized.", self.node_name)

    def on_image(self, msg):
        # Note (PCH): the camera info is only set during launch.
        if self.camera_info_msg is not None:
            self.camera_info_msg.header.stamp = msg.header.stamp
            self.pub_cam_info.publish(self.camera_info_msg)

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

        # Construct calibration message from parameter file
        cam_info = CameraInfo()
        cam_info.width = yaml_dict['image_width']
        cam_info.height = yaml_dict['image_height']
        cam_info.distortion_model = yaml_dict['distortion_model']
        cam_info.D = yaml_dict['distortion_coefficients']['data']
        cam_info.K = yaml_dict['camera_matrix']['data']
        cam_info.R = yaml_dict['rectification_matrix']['data']
        cam_info.P = yaml_dict['projection_matrix']['data']

        rospy.loginfo("[%s] Read calibration file %s" % (self.node_name, file_name))
        return cam_info

    def get_config_path(self, name):
        rospack = rospkg.RosPack()
        return rospack.get_path('duckietown') + '/config/pi_camera/' + name + ".yaml"  

    def on_save_camera_info(self, req):
        # Save camera info (calibration) to a yaml file.
        cam_info = req.camera_info
        calib = {
            'image_width': cam_info.width,
            'image_height': cam_info.height,
            'camera_name': rospy.get_name().strip("/"),
            'distortion_model': cam_info.distortion_model,
            'distortion_coefficients': {'data': cam_info.D, 'rows':1, 'cols':5},
            'camera_matrix': {'data': cam_info.K, 'rows':3, 'cols':3},
            'rectification_matrix': {'data': cam_info.R, 'rows':3, 'cols':3},
            'projection_matrix': {'data': cam_info.P,'rows':3, 'cols':4}
        }
        
        # Prepare response
        file_name = self.get_config_path(self.veh_name)
        response = SetCameraInfoResponse()
        response.success = False
        response.status_message = "Failed to save calibration to %s" % file_name

        # Write to file
        try:
            out_file = open(file_name, 'w')
            yaml.safe_dump(calib, out_file)
        except IOError:
            return response
        rospy.loginfo("[%s] Saved calibration to %s." % (self.node_name, file_name))
        response.success = True
        response.status_message = "Saved calibration to %s" % file_name
        return response

if __name__ == '__main__': 
    rospy.init_node('cam_info_node', anonymous=False)
    node = CamInfoNode()
    rospy.spin()
