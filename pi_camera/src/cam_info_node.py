#!/usr/bin/env python
import rospy
import rospkg
import yaml
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
import os.path

class CamInfoNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        # Load parameters
        # self.pub_freq       = self.setupParam("~pub_freq",1.0)
        self.config         = self.setupParam("~config","baseline")
        self.cali_file_name = self.setupParam("~cali_file_name","default")
        self.image_type = self.setupParam("~image_type", "compressed")

        # Setup publisher
        self.pub_cam_info = rospy.Publisher("~camera_info",CameraInfo,queue_size=1)
        # Get path to calibration yaml file
        rospack = rospkg.RosPack()
        self.cali_file = rospack.get_path('duckietown') + "/config/" + self.config + "/calibration/camera_intrinsic/" +  self.cali_file_name + ".yaml" 
        self.camera_info_msg = None

        # Load calibration yaml file
        if not os.path.isfile(self.cali_file):
            rospy.logwarn("[%s] Can't find calibration file: %s.\nUsing default calibration instead." %(self.node_name,self.cali_file))
            self.cali_file = rospack.get_path('duckietown') + "/config/" + self.config + "/calibration/camera_intrinsic/default.yaml" 

        # Shutdown if no calibration file not found
        if not os.path.isfile(self.cali_file):
            rospy.signal_shutdown("Found no calibration file ... aborting")

        # Print out and prepare message
        rospy.loginfo("[%s] Using calibration file: %s" %(self.node_name,self.cali_file))
        self.camera_info_msg = self.loadCameraInfo(self.cali_file)
        self.camera_info_msg.header.frame_id = rospy.get_namespace() + "camera_optical_frame"
        rospy.loginfo("[%s] CameraInfo: %s" %(self.node_name,self.camera_info_msg))

        rospy.Subscriber("~compressed_image", CompressedImage, self.cbCompressedImage,queue_size=1)
    
        # Create service (for camera_calibration)
        rospy.Service("~set_camera_info", SetCameraInfo, self.cbSrvSetCameraInfo)

    def cbCompressedImage(self,msg):
        # Note (PCH): camera info is only set once during launch.
        if self.camera_info_msg is not None:
            self.camera_info_msg.header.stamp = msg.header.stamp
            self.pub_cam_info.publish(self.camera_info_msg)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def loadCameraInfo(self, filename):
        stream = file(filename, 'r')
        calib_data = yaml.load(stream)
        cam_info = CameraInfo()
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']
        cam_info.K = calib_data['camera_matrix']['data']
        cam_info.D = calib_data['distortion_coefficients']['data']
        cam_info.R = calib_data['rectification_matrix']['data']
        cam_info.P = calib_data['projection_matrix']['data']
        cam_info.distortion_model = calib_data['distortion_model']
        return cam_info

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

    def cbSrvSetCameraInfo(self,req):
        # TODO: save req.camera_info to yaml file
        rospy.loginfo("[cbSrvSetCameraInfo] Callback!")
        filename = self.cali_file_folder + rospy.get_namespace().strip("/") + ".yaml"
        response = SetCameraInfoResponse()
        response.success = self.saveCameraInfo(req.camera_info,filename)
        response.status_message = "Write to %s" %filename #TODO file name
        return response

    def saveCameraInfo(self, camera_info_msg, filename):
        # Convert camera_info_msg and save to a yaml file
        rospy.loginfo("[saveCameraInfo] filename: %s" %(filename))
        file = open(filename, 'w')

        # Converted from camera_info_manager.py
        calib = {'image_width': camera_info_msg.width,
        'image_height': camera_info_msg.height,
        'camera_name': rospy.get_name().strip("/"), #TODO check this
        'distortion_model': camera_info_msg.distortion_model,
        'distortion_coefficients': {'data': camera_info_msg.D, 'rows':1, 'cols':5},
        'camera_matrix': {'data': camera_info_msg.K, 'rows':3, 'cols':3},
        'rectification_matrix': {'data': camera_info_msg.R, 'rows':3, 'cols':3},
        'projection_matrix': {'data': camera_info_msg.P,'rows':3, 'cols':4}}
        
        rospy.loginfo("[saveCameraInfo] calib %s" %(calib))

        try:
            rc = yaml.safe_dump(calib, file)
            return True
        except IOError:
            return False

if __name__ == '__main__': 
    rospy.init_node('cam_info_node',anonymous=False)
    node = CamInfoNode()
    rospy.on_shutdown(node.on_shutdown)
    rospy.spin()
