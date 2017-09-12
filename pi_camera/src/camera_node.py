#!/usr/bin/env python

# Modified by PCH 2017

"""
Description: Publish compressed images from the pi camera at 30 Hz (default).

Publications
    ~compressed_image (CompressedImage)

Services (advertised)
    ~set_param (SetParam)
"""

import rospy
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.srv import SetParam, SetParamResponse
import io
import thread
from picamera import PiCamera

class CameraNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        # Default parameters
        self.framerate = 10 # Hz
        self.res_w = 640
        self.res_h = 480

        # Setup PiCamera
        self.camera = PiCamera()
        self.camera.framerate = self.framerate
        self.camera.resolution = (self.res_w, self.res_h)

        self.frame_id = rospy.get_namespace() + "camera_optical_frame"

        self.pub_img= rospy.Publisher("~compressed_image", CompressedImage, queue_size=1)

        rospy.Service("~set_param", SetParam, self.on_set_param)

        # Node (PCH): PiRGBArray extends io.BytesIO with a numpy array, but we
        # we don't need it here.
        self.stream = io.BytesIO()
 
        # self.camera.exposure_mode = 'off'
        # self.camera.awb_mode = 'off'

        self.needs_update = False
        rospy.loginfo("[%s] Initialized." % (self.node_name))

    def on_set_param(self, msg):
        if (msg.name == "framerate") and (msg.value > 0):
            self.framerate = msg.value
            self.needs_update = True
            success = True
        else:
            success = False
        return SetParamResponse(success)
 
    def start_capturing(self):
        rospy.loginfo("[%s] Start capturing." %(self.node_name))
        while not rospy.is_shutdown():
            # Node (PCH): the stream and publisher are passed as local
            # variables to speed up access (no lookup required).
            generator =  self.grab_and_publish(self.stream, self.pub_img)
            try:
                self.camera.capture_sequence(generator, 'jpeg',
                    use_video_port=True, splitter_port=0)
            except StopIteration:
                pass
            rospy.loginfo("[%s] Updating framerate to %s Hz." % (
                self.node_name, self.framerate))
            self.camera.framerate = self.framerate
            self.needs_update = False

        self.camera.close()
        rospy.loginfo("[%s] Capture ended." %(self.node_name))

    def grab_and_publish(self, stream, publisher):
        # Can't change parameters during sequence so may have to break out
        while not self.needs_update and not rospy.is_shutdown(): 
            yield stream
            # Construct image_msg
            # Grab image from stream
            stamp = rospy.Time.now()
            stream.seek(0)
            stream_data = stream.getvalue()

            # Generate compressed image
            image_msg = CompressedImage()
            image_msg.format = "jpeg"
            image_msg.data = stream_data

            image_msg.header.stamp = stamp
            image_msg.header.frame_id = self.frame_id
            publisher.publish(image_msg)
                        
            # Clear stream
            stream.seek(0)
            stream.truncate()

            rospy.sleep(0.001) # 1ms

if __name__ == '__main__': 
    rospy.init_node('camera_node',anonymous=False)
    camera_node = CameraNode()
    # Node (PCH): thread is used so that node responds to shutdown signal.
    thread.start_new_thread(camera_node.start_capturing, ())
    rospy.spin()
