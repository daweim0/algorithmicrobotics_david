#!/usr/bin/env python
import rospy
import numpy as np
from duckietown_msgs.msg import SegmentList, Segment, BoolStamped, StopLineReading, LanePose
from geometry_msgs.msg import Point
import time
import math

class StopLineFilterNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.lane_pose = LanePose()

        # Parameters
        self.stop_distance = 0.2 # distance from the stop line that we should stop 
        self.min_segs      = 1 # minimum number of red segments that we should detect to estimate a stop
        self.lane_width = 0.2

        # Topics
        self.pub_stop_line_reading = rospy.Publisher("~stop_line_reading", StopLineReading, queue_size=1)
        self.pub_at_stop_line = rospy.Publisher("~at_stop_line", BoolStamped, queue_size=1)
        self.sub_segs      = rospy.Subscriber("~segment_list", SegmentList, self.on_segment_list)
        self.sub_lane      = rospy.Subscriber("~lane_pose", LanePose, self.on_lane_pose)


    def on_lane_pose(self, lane_pose_msg):
        self.lane_pose = lane_pose_msg

    def on_segment_list(self, segment_list_msg):
        good_seg_count=0
        stop_line_x_accumulator=0.0
        stop_line_y_accumulator=0.0
        for segment in segment_list_msg.segments:
            if segment.color != segment.RED:
                continue
            if segment.points[0].x < 0 or segment.points[1].x < 0: # the point is behind us 
                continue

            p1_lane = self.to_lane_frame(segment.points[0])
            p2_lane = self.to_lane_frame(segment.points[1])
            avg_x = 0.5*(p1_lane[0] + p2_lane[0])
            avg_y = 0.5*(p1_lane[1] + p2_lane[1])
            stop_line_x_accumulator += avg_x
            stop_line_y_accumulator += avg_y # TODO output covariance and not just mean
            good_seg_count += 1.0

        stop_line_reading_msg = StopLineReading()
        stop_line_reading_msg.header.stamp = segment_list_msg.header.stamp
        if (good_seg_count < self.min_segs):
            stop_line_reading_msg.stop_line_detected = False
            stop_line_reading_msg.at_stop_line = False
            self.pub_stop_line_reading.publish(stop_line_reading_msg)
            return
        
        stop_line_reading_msg.stop_line_detected = True
        stop_line_point = Point()
        stop_line_point.x = stop_line_x_accumulator/good_seg_count
        stop_line_point.y = stop_line_y_accumulator/good_seg_count
        stop_line_reading_msg.stop_line_point = stop_line_point
        stop_line_reading_msg.at_stop_line = stop_line_point.x < self.stop_distance and math.fabs(stop_line_point.y) < self.lane_width/3 
        self.pub_stop_line_reading.publish(stop_line_reading_msg)    
        if stop_line_reading_msg.at_stop_line:
            msg = BoolStamped()
            msg.header.stamp = stop_line_reading_msg.header.stamp
            msg.data = True
            self.pub_at_stop_line.publish(msg)
   
    def to_lane_frame(self, point):
        p_homo = np.array([point.x,point.y,1])
        phi = self.lane_pose.phi
        d   = self.lane_pose.d
        T = np.array([[math.cos(phi), -math.sin(phi), 0],
                      [math.sin(phi), math.cos(phi) , d],
                      [0,0,1]])
        p_new_homo = T.dot(p_homo)
        p_new = p_new_homo[0:2]
        return p_new

if __name__ == '__main__': 
    rospy.init_node('stop_line_filter_node',anonymous=False)
    lane_filter_node = StopLineFilterNode()
    rospy.spin()

